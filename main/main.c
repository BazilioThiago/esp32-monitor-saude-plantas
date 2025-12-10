#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include <stdlib.h>
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "freertos/semphr.h"

static const char *TAG = "SENSOR";

#define CANAL_UMIDADE_ADC ADC1_CHANNEL_7 // GPIO35 -> ADC1_CHANNEL_7
#define BRUTO_UMIDADE_SECO 4095 // seco
#define BRUTO_UMIDADE_MOLHADO 1700 // submerso
#define PINO_DHT 21 // Sensor de temperatura (assumido DHT11 HW-036) no GPIO21
#define ADC_MAXIMO 4095.0f
#define VREF 3.3f
#define DHT_MAX_WAIT 1000

#define CANAL_LUZ_ADC ADC1_CHANNEL_4
#define PINO_LUZ 32 // ADC1_CHANNEL_4 -> GPIO32

// WiFi AP
#define WIFI_SSID "thiago_e_guilherme"
#define WIFI_PASS "12345678"

// histórico
#define HIST_TAMANHO 10
#define DELTA_MAX_TEMPERATURA 15

typedef struct {
    int bruto_umidade;
    float pct_umidade;
    int bruto_luz;
    float pct_luz;
    int temperatura;
    int umidade_ar;
    uint32_t ts_ms;
} leitura_t;

static leitura_t historico[HIST_TAMANHO];
static int hist_idx = 0;
static int hist_cnt = 0;
static leitura_t leitura_atual;
static SemaphoreHandle_t mutex_leituras = NULL;
static httpd_handle_t servidor_http = NULL;
static int ultima_temperatura = 0;
static int ultima_umidade_ar = 0;


static void wait(uint32_t us) {
    int64_t inicio = esp_timer_get_time();
    while ((uint32_t)(esp_timer_get_time() - inicio) < us) {
        ;
    }
}

static bool ler_temperatura(int gpio, int *out_temp, int *out_umi) {
    uint8_t bits[5] = {0};

    gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(gpio, 0);
    vTaskDelay(pdMS_TO_TICKS(20));

    
    gpio_set_level(gpio, 1);
    wait(30);
    gpio_set_direction(gpio, GPIO_MODE_INPUT);
    gpio_set_pull_mode(gpio, GPIO_PULLUP_ONLY);

    // aguarda resposta do sensor
    int64_t tstart = esp_timer_get_time();
    while (gpio_get_level(gpio) == 1) {
        if ((esp_timer_get_time() - tstart) > DHT_MAX_WAIT * 100) return false;
    }
    
    tstart = esp_timer_get_time();
    while (gpio_get_level(gpio) == 0) {
        if ((esp_timer_get_time() - tstart) > DHT_MAX_WAIT * 100) return false;
    }
    tstart = esp_timer_get_time();
    while (gpio_get_level(gpio) == 1) {
        if ((esp_timer_get_time() - tstart) > DHT_MAX_WAIT * 100) return false;
    }

    for (int i = 0; i < 40; i++) {
        tstart = esp_timer_get_time();
        while (gpio_get_level(gpio) == 0) {
            if ((esp_timer_get_time() - tstart) > DHT_MAX_WAIT * 100) return false;
        }
        int64_t t0 = esp_timer_get_time();
        while (gpio_get_level(gpio) == 1) {
            if ((esp_timer_get_time() - t0) > DHT_MAX_WAIT * 100) break;
        }
        int pulse_len = (int)(esp_timer_get_time() - t0);
        int byte_idx = i / 8;
        bits[byte_idx] <<= 1;
        if (pulse_len > 50) bits[byte_idx] |= 1;
    }

    // soma
    uint8_t checksum = bits[0] + bits[1] + bits[2] + bits[3];
    if (checksum != bits[4]) return false;

    *out_umi = 100 - bits[0];
    *out_temp = bits[2];
    return true;
}

// ler o canal e retorna a média (valor bruto)
static int adc_ler_media(int canal, int amostras) {
    int soma = 0;
    for (int i = 0; i < amostras; i++) {
        soma += adc1_get_raw(canal);
        vTaskDelay(pdMS_TO_TICKS(3));
    }
    return soma / amostras;
}

// ler umidade do solo: retorna bruto e percentual (0=seco,100=molhado)
static void ler_umidade_solo(int amostras, int *out_raw, float *out_pct) {
    int bruto = adc_ler_media(CANAL_UMIDADE_ADC, amostras);
    float pct = ((float)(BRUTO_UMIDADE_SECO - bruto) * 100.0f) / (float)(BRUTO_UMIDADE_SECO - BRUTO_UMIDADE_MOLHADO);
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    if (out_raw) *out_raw = bruto;
    if (out_pct) *out_pct = pct;
}

// ler luminosidade: retorna bruto e percentual (0=escuro, 100=claro)
static void ler_luminosidade(int amostras, int *out_raw, float *out_pct) {
    int bruto = adc_ler_media(CANAL_LUZ_ADC, amostras);
    float pct = (bruto / ADC_MAXIMO) * 100.0f;
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    if (out_raw) *out_raw = bruto;
    if (out_pct) *out_pct = pct;
}

static void tarefa_sensores(void *arg) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(CANAL_UMIDADE_ADC, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(CANAL_LUZ_ADC, ADC_ATTEN_DB_11);

    gpio_set_direction(PINO_LUZ, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PINO_LUZ, GPIO_PULLDOWN_ONLY);

    while (1) {
        const int samples = 8;
        int bruto_umidade = 0;
        float pct_umidade = 0.0f;
        ler_umidade_solo(samples, &bruto_umidade, &pct_umidade);
        ESP_LOGI(TAG, "Umidade do solo: %.1f%%", pct_umidade);

        int bruto_luz = 0;
        float pct_luz = 0.0f;
        ler_luminosidade(samples, &bruto_luz, &pct_luz);
        // ESP_LOGI(ETIQUETA, "Sensor de Luminosidade - bruto: %d  Brilho: %.1f%%", bruto_luz, pct_luz);
        ESP_LOGI(TAG, "Brilho: %.1f%%", pct_luz);

        int t = ultima_temperatura;
        int h = ultima_umidade_ar;
        bool ok = false;
        // tentar até 3 vezes ler, em caso de falha manter último valor válido
        for (int tentativa = 0; tentativa < 3; tentativa++) {
            if (ler_temperatura(PINO_DHT, &t, &h)) {
                ok = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        if (ok) {
            // se já tivermos um último valor válido, rejeitar saltos muito grandes
            //fix: nos casos de salto muito grande (>15) a temperatura é exatamente o dobro do valor real (aparentemente)
            if (ultima_temperatura != 0 && abs(t - ultima_temperatura) > DELTA_MAX_TEMPERATURA) {
                int metade = t / 2;
                if (abs(metade - ultima_temperatura) <= DELTA_MAX_TEMPERATURA) {
                    t = metade;
                    ultima_temperatura = t;
                    ultima_umidade_ar = h;
                } else {
                    t = ultima_temperatura;
                    h = ultima_umidade_ar;
                }
            } else {
                ultima_temperatura = t;
                ultima_umidade_ar = h;
            }
        } else {
            t = ultima_temperatura;
            h = ultima_umidade_ar;
        }
        ESP_LOGI(TAG, "Temperatura: %dC", t);

        // atualizar leitura atual e histórico
        leitura_t nova;
        nova.bruto_umidade = bruto_umidade;
        nova.pct_umidade = pct_umidade;
        nova.bruto_luz = bruto_luz;
        nova.pct_luz = pct_luz;
        nova.temperatura = t;
        nova.umidade_ar = h;
        nova.ts_ms = (uint32_t)(esp_timer_get_time() / 1000);

        if (mutex_leituras) {
            if (xSemaphoreTake(mutex_leituras, pdMS_TO_TICKS(200))) {
                leitura_atual = nova;
                // inserir no histórico circular
                historico[hist_idx] = nova;
                hist_idx = (hist_idx + 1) % HIST_TAMANHO;
                if (hist_cnt < HIST_TAMANHO) hist_cnt++;
                xSemaphoreGive(mutex_leituras);
            }
        }

        ESP_LOGI(TAG, "-------------------------------------");
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

// página HTML
static esp_err_t index_get_handler(httpd_req_t *req) {
    const char* pagina =
        "<!doctype html><html><head><meta charset=\"utf-8\">"
        "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
        "<title>Leituras</title></head><body><h1>Leituras sensores</h1>"
        "<div><strong>Atual</strong><div id=\"atual\">carregando...</div></div>"
        "<h2>Ultimas 10 leituras</h2><div id=\"historico\">carregando...</div>"
        "<script>"
        "function fmt(v){return v==null?'-':v}"
        "function atualizar(){"
        "  fetch('/values').then(r=>r.json()).then(j=>{"
        "    const a = j.atual;"
        "    document.getElementById('atual').innerHTML = 'Umidade solo: ' + a.pct_umidade.toFixed(1) + '% <br> Luminosidade: ' + a.pct_luz.toFixed(1) + '% <br> Temperatura: ' + a.temperatura + 'C <br>';"
        "    let html = '';"
        "    j.historico.forEach(function(h){"
        "      var d = new Date(h.ts_ms);"
        "      html += '<div>' + d.toLocaleTimeString() + ':  Umidade do solo: ' + h.pct_umidade.toFixed(1) + '%  Luminosidade: ' + h.pct_luz.toFixed(1) + '%  Temp: ' + h.temperatura + 'C</div>';"
        "    });"
        "    document.getElementById('historico').innerHTML = html;"
        "  });"
        "}"
        "atualizar();setInterval(atualizar,3000);"
        "</script></body></html>";
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, pagina, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// handler: valores JSON
static esp_err_t values_get_handler(httpd_req_t *req) {
    char *buf = malloc(4096);
    if (!buf) return ESP_ERR_NO_MEM;
    int len = 0;
    if (mutex_leituras && xSemaphoreTake(mutex_leituras, pdMS_TO_TICKS(200))) {
        len += snprintf(buf + len, 4096 - len, "{\"atual\":{\"bruto_umidade\":%d,\"pct_umidade\":%.2f,\"bruto_luz\":%d,\"pct_luz\":%.2f,\"temperatura\":%d,\"umidade_ar\":%d,\"ts_ms\":%u},\"historico\":[",
            leitura_atual.bruto_umidade, leitura_atual.pct_umidade, leitura_atual.bruto_luz, leitura_atual.pct_luz, leitura_atual.temperatura, leitura_atual.umidade_ar, (unsigned)leitura_atual.ts_ms);
        for (int i = 0; i < hist_cnt; i++) {
            int idx = (hist_idx - hist_cnt + i + HIST_TAMANHO) % HIST_TAMANHO;
            leitura_t *h = &historico[idx];
            len += snprintf(buf + len, 4096 - len, "{\"bruto_umidade\":%d,\"pct_umidade\":%.2f,\"bruto_luz\":%d,\"pct_luz\":%.2f,\"temperatura\":%d,\"umidade_ar\":%d,\"ts_ms\":%u}%s",
                h->bruto_umidade, h->pct_umidade, h->bruto_luz, h->pct_luz, h->temperatura, h->umidade_ar, (unsigned)h->ts_ms, (i+1< hist_cnt)?",":"");
        }
        len += snprintf(buf + len, 4096 - len, "]}");
        xSemaphoreGive(mutex_leituras);
    } else {
        len = snprintf(buf, 4096, "{\"error\":\"mutex_busy\"}");
    }
    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_send(req, buf, len);
    free(buf);
    return ESP_OK;
}

void app_main(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // criar mutex para proteger leituras/histórico
    mutex_leituras = xSemaphoreCreateMutex();
    if (mutex_leituras == NULL) {
        ESP_LOGE(TAG, "Falha ao criar mutex de leituras");
    }

    // inicializar WiFi em modo AP
    esp_netif_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = 0,
            .password = WIFI_PASS,
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen(WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Ponto de acesso iniciado. SSID:%s PIN:%s", WIFI_SSID, WIFI_PASS);

    // iniciar servidor web
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    if (httpd_start(&servidor_http, &config) == ESP_OK) {
        httpd_uri_t uri_index = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = index_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(servidor_http, &uri_index);
        httpd_uri_t uri_values = {
            .uri = "/values",
            .method = HTTP_GET,
            .handler = values_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(servidor_http, &uri_values);
        ESP_LOGI(TAG, "Servidor HTTP iniciado");
    } else {
        ESP_LOGE(TAG, "Falha ao iniciar servidor HTTP");
    }

    xTaskCreate(tarefa_sensores, "tarefa_sensores", 4096, NULL, 5, NULL);
}
