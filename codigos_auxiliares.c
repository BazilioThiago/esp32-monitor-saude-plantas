//
//
//Código Exemplo ESPIDF

#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "lora.h"

#if CONFIG_SENDER
void task_tx(void *pvParameters)
{
	ESP_LOGI(pcTaskGetName(NULL), "Start");
	uint8_t buf[255]; // Maximum Payload size of SX1276/77/78/79 is 255
	while(1) {
		TickType_t nowTick = xTaskGetTickCount();
		int send_len = sprintf((char *)buf,"Hello World!! %"PRIu32, nowTick);
		lora_send_packet(buf, send_len);
		ESP_LOGI(pcTaskGetName(NULL), "%d byte packet sent...", send_len);
		int lost = lora_packet_lost();
		if (lost != 0) {
			ESP_LOGW(pcTaskGetName(NULL), "%d packets lost", lost);
		}
		vTaskDelay(pdMS_TO_TICKS(1000));
	} // end while

	// never reach here
	vTaskDelete( NULL );
}
#endif // CONFIG_SENDER

#if CONFIG_RECEIVER
void task_rx(void *pvParameters)
{
	ESP_LOGI(pcTaskGetName(NULL), "Start");
	uint8_t buf[255]; // Maximum Payload size of SX1276/77/78/79 is 255
	while(1) {
		lora_receive(); // put into receive mode
		if (lora_received()) {
			int rxLen = lora_receive_packet(buf, sizeof(buf));
			ESP_LOGI(pcTaskGetName(NULL), "%d byte packet received:[%.*s]", rxLen, rxLen, buf);
		}
		vTaskDelay(1); // Avoid WatchDog alerts
	} // end while

	// never reach here
	vTaskDelete( NULL );
}
#endif // CONFIG_RECEIVER

void app_main()
{
	// Initialize LoRa
	if (lora_init() == 0) {
		ESP_LOGE(pcTaskGetName(NULL), "Does not recognize the module");
		while(1) {
			vTaskDelay(1);
		}
	}

#if CONFIG_433MHZ
	ESP_LOGI(pcTaskGetName(NULL), "Frequency is 433MHz");
	lora_set_frequency(433e6); // 433MHz
#elif CONFIG_866MHZ
	ESP_LOGI(pcTaskGetName(NULL), "Frequency is 866MHz");
	lora_set_frequency(866e6); // 866MHz
#elif CONFIG_915MHZ
	ESP_LOGI(pcTaskGetName(NULL), "Frequency is 915MHz");
	lora_set_frequency(915e6); // 915MHz
#elif CONFIG_OTHER
	ESP_LOGI(pcTaskGetName(NULL), "Frequency is %dMHz", CONFIG_OTHER_FREQUENCY);
	long frequency = CONFIG_OTHER_FREQUENCY * 1000000;
	lora_set_frequency(frequency);
#endif

	lora_enable_crc();

	int cr = 1;
	int bw = 7;
	int sf = 7;
#if CONFIG_ADVANCED
	cr = CONFIG_CODING_RATE;
	bw = CONFIG_BANDWIDTH;
	sf = CONFIG_SF_RATE;
#endif

	lora_set_coding_rate(cr);
	//lora_set_coding_rate(CONFIG_CODING_RATE);
	//cr = lora_get_coding_rate();
	ESP_LOGI(pcTaskGetName(NULL), "coding_rate=%d", cr);

	lora_set_bandwidth(bw);
	//lora_set_bandwidth(CONFIG_BANDWIDTH);
	//int bw = lora_get_bandwidth();
	ESP_LOGI(pcTaskGetName(NULL), "bandwidth=%d", bw);

	lora_set_spreading_factor(sf);
	//lora_set_spreading_factor(CONFIG_SF_RATE);
	//int sf = lora_get_spreading_factor();
	ESP_LOGI(pcTaskGetName(NULL), "spreading_factor=%d", sf);

#if CONFIG_SENDER
	xTaskCreate(&task_tx, "TX", 1024*3, NULL, 5, NULL);
#endif
#if CONFIG_RECEIVER
	xTaskCreate(&task_rx, "RX", 1024*3, NULL, 5, NULL);
#endif
}


//
//
// Código que cria wifi, e envia os valores lidos pra web (194.168.4.1)
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "nvs_flash.h"

// Porta do computador -> COM3
#define TDS_PIN ADC1_CHANNEL_6  // GPIO34 -> canal 6
#define VREF 3.3
#define SAMPLES 30
#define TEMP_CORRECTION 25.0

static const char *TAG = "SALINIDADE";
float temperature = 25.0;
float tdsValue = 0.0;
float voltage = 0.0;

httpd_handle_t server = NULL;

// Função para ler o sensor
void lerSalinidade(void) {
    uint32_t adcSum = 0;
    for (int i = 0; i < SAMPLES; i++) {
        adcSum += adc1_get_raw(TDS_PIN);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    float adcAverage = adcSum / (float)SAMPLES;
    voltage = (adcAverage / 4095.0) * VREF;
    float compensation = 1.0 + 0.02 * (temperature - TEMP_CORRECTION);
    tdsValue = (133.42 * pow(voltage, 3)
              - 255.86 * pow(voltage, 2)
              + 857.39 * voltage) * compensation;
}

// Gera página HTML
static esp_err_t root_get_handler(httpd_req_t *req) {
    lerSalinidade();
    char resp[512];
    snprintf(resp, sizeof(resp),
        "<!DOCTYPE html><html><head>"
        "<meta charset='UTF-8'>"
        "<meta http-equiv='refresh' content='3'>"
        "<title>Monitor de Salinidade</title></head><body>"
        "<h2>Sistema Medidor de Salinidade</h2>"
        "<p><b>Tensão:</b> %.2f V</p>"
        "<p><b>Salinidade (TDS):</b> %.0f ppm</p>"
        "<p>Atualiza automaticamente a cada 3s.</p>"
        "</body></html>", voltage, tdsValue);

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

//Inicia servidor HTTP 
httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = root_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &root);
    }
    return server;
}

// Configura Access Point (cria a rede wifi)
void wifi_init_softap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "ESP32_Salinidade",
            .ssid_len = strlen("ESP32_Salinidade"),
            .channel = 1,
            .password = "12345678",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen("12345678") == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Access Point iniciado: SSID=%s senha=%s", wifi_config.ap.ssid, wifi_config.ap.password);
}


// MAIN
void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(TDS_PIN, ADC_ATTEN_DB_11);

    wifi_init_softap();
    start_webserver();

    ESP_LOGI(TAG, "Servidor Web iniciado");
}


//
//
// Código que envia os valores lidos via Lora
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/adc.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
// #include "sx127x.h"

// Porta do computador -> COM3
#define TDS_PIN ADC1_CHANNEL_6  // GPIO34 -> canal 6
#define VREF 3.3
#define SAMPLES 30
#define TEMP_CORRECTION 25.0
#define PIN_SPI_MISO 19
#define PIN_SPI_MOSI 23
#define PIN_SPI_SCLK 18
#define PIN_SPI_CS   5
#define PIN_SX127X_RESET 14
#define PIN_SX127X_DIO0  26

#define SIMULATE_LORA 1
// 1 = simula envio/recebimento localmente (para testes)
// 0 = usa driver real

static const char *TAG = "SALINIDADE";
float temperature = 25.0;
float tdsValue = 0.0;
float voltage = 0.0;

// fila usada apenas no modo SIMULATE_LORA
static QueueHandle_t lora_sim_queue = NULL;

httpd_handle_t server = NULL;

// Função para ler o sensor
void lerSalinidade(void) {
    uint32_t adcSum = 0;
    for (int i = 0; i < SAMPLES; i++) {
        adcSum += adc1_get_raw(TDS_PIN);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    float adcAverage = adcSum / (float)SAMPLES;
    voltage = (adcAverage / 4095.0) * VREF;
    float compensation = 1.0 + 0.02 * (temperature - TEMP_CORRECTION);
    tdsValue = (133.42 * pow(voltage, 3) - 255.86 * pow(voltage, 2) + 857.39 * voltage) * compensation;
}

// Gera página HTML
static esp_err_t root_get_handler(httpd_req_t *req) {
    lerSalinidade();
    char resp[512];
    snprintf(resp, sizeof(resp),
        "<!DOCTYPE html><html><head>"
        "<meta charset='UTF-8'>"
        "<meta http-equiv='refresh' content='3'>"
        "<title>Monitor de Salinidade</title></head><body>"
        "<h2>Sistema Medidor de Salinidade</h2>"
        "<p><b>Tensão:</b> %.2f V</p>"
        "<p><b>Salinidade (TDS):</b> %.0f ppm</p>"
        "<p>Atualiza automaticamente a cada 3s.</p>"
        "</body></html>", voltage, tdsValue);

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

//Inicia servidor HTTP 
httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = root_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &root);
    }
    return server;
}

// Configura Access Point (cria a rede wifi)
void wifi_init_softap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "ESP32_Salinidade",
            .ssid_len = strlen("ESP32_Salinidade"),
            .channel = 1,
            .password = "12345678",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen("12345678") == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Access Point iniciado: SSID=%s senha=%s", wifi_config.ap.ssid, wifi_config.ap.password);
}

// Inicializa o LORA
static void lora_init(void) {
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_SPI_MISO,
        .mosi_io_num = PIN_SPI_MOSI,
        .sclk_io_num = PIN_SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 255,
    };
    ret = spi_bus_initialize(VSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %d", ret);
        return;
    }

    // configurar CS como gpio (driver do componente normalmente faz isto também)
    gpio_set_direction(PIN_SPI_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_SPI_CS, 1);

    // reset do módulo
    gpio_set_direction(PIN_SX127X_RESET, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_SX127X_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_SX127X_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

#if !SIMULATE_LORA
    // Inicializar driver SX127x (ajuste campos se seu driver tiver nomes diferentes)
    sx127x_cfg_t cfg = {0};
    // selecionar frequência conforme sdkconfig (ex.: 433 MHz)
#ifdef CONFIG_433MHZ
    cfg.freq = 433000000;
#elif defined(CONFIG_866MHZ)
    cfg.freq = 866000000;
#elif defined(CONFIG_915MHZ)
    cfg.freq = 915000000;
#else
    cfg.freq = 433000000; // default
#endif
    cfg.sync_word = 0x34;
    cfg.power = 17; // dBm, ajuste se necessário
    cfg.cs = PIN_SPI_CS;
    cfg.reset = PIN_SX127X_RESET;
    cfg.dio0 = PIN_SX127X_DIO0;
    cfg.spi_host = VSPI_HOST; // ou HSPI_HOST dependendo do seu wiring/config

    if (sx127x_init(&cfg) != ESP_OK) {
        ESP_LOGE(TAG, "sx127x_init falhou");
        return;
    }
    ESP_LOGI(TAG, "LoRa init: sx127x inicializado (freq=%d)", cfg.freq);
#else
    ESP_LOGI(TAG, "LoRa init: SPI bus up (modo SIMULATE_LORA ativo)");
#endif
}


// envia string por LoRa
static esp_err_t lora_send_values(float voltage, float tds)
{
    char payload[64];
    int len = snprintf(payload, sizeof(payload), "V:%.2f,TDS:%.0f", voltage, tds);
    if (len <= 0) return ESP_FAIL;

    ESP_LOGI(TAG, "Enviando por LoRa: %s", payload);

    // Modo de simulação: enfileira payload para uma task receptora local
#if SIMULATE_LORA
    if (lora_sim_queue) {
        payload[sizeof(payload)-1] = '\0'; // garantir terminação
        
        // enviar cópia (tamanho fixo da mensagem)
        if (xQueueSend(lora_sim_queue, payload, pdMS_TO_TICKS(100)) != pdPASS) {
            ESP_LOGW(TAG, "Fila simulada cheia, payload perdido");
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "Payload enfileirado (simulado)");
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "Modo SIMULATE_LORA ativo, mas fila não criada");
        return ESP_FAIL;
    }
#else
    esp_err_t r = sx127x_send((const uint8_t*)payload, len, pdMS_TO_TICKS(5000));
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "sx127x_send falhou: %d", r);
        return r;
    }

    ESP_LOGI(TAG, "Envio LORA concluído (driver sx127x)");
    return ESP_OK;
#endif
}


// Simula um receptor LoRa local (pega mensagens da fila e as imprime)
static void lora_receiver_task(void *arg)
{
    char buf[64];
    while (1) {
        if (xQueueReceive(lora_sim_queue, buf, portMAX_DELAY) == pdPASS) {
            ESP_LOGI(TAG, "[SIM_RX] Mensagem recebida: %s", buf);
            // aqui você pode disparar callbacks de processamento, atualizar estado, etc.
        }
    }
}


// lê e envia periodicamente
static void lora_sender_task(void *arg) {
    while (1) {
        lerSalinidade();
        lora_send_values(voltage, tdsValue);
        vTaskDelay(pdMS_TO_TICKS(5000)); // enviar a cada 5s
    }
}



// MAIN
void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(TDS_PIN, ADC_ATTEN_DB_11);

    wifi_init_softap();
    start_webserver();

    ESP_LOGI(TAG, "Servidor Web iniciado");

    // Inicializa LoRa e cria task de envio
    lora_init();
    // se estiver no modo simulado, criar fila e task receptor local
#if SIMULATE_LORA
    lora_sim_queue = xQueueCreate(8, 64);
    if (lora_sim_queue == NULL) {
        ESP_LOGW(TAG, "Falha ao criar fila simulada");
    } else {
        xTaskCreate(lora_receiver_task, "lora_receiver", 4096, NULL, 5, NULL);
    }
#endif

    xTaskCreate(lora_sender_task, "lora_sender", 4096, NULL, 5, NULL);

}


//
//
// Código final v1.0 (envia por lota, e cria a página web)
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/adc.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
// #include "sx127x.h"
#include "esp_timer.h"
#include "esp_sleep.h"

// Porta do computador -> COM3
#define TDS_PIN ADC1_CHANNEL_6  // GPIO34 -> canal 6
#define VREF 3.3
#define SAMPLES 30
#define TEMP_CORRECTION 25.0
#define PIN_SPI_MISO 19
#define PIN_SPI_MOSI 23
#define PIN_SPI_SCLK 18
#define PIN_SPI_CS   5
#define PIN_SX127X_RESET 14
#define PIN_SX127X_DIO0  26

#define SIMULATE_LORA 1
// 1 = simula envio/recebimento localmente (para testes)
// 0 = usa driver real
#define TEMPO_HIBERNACAO 5 
// em segundos, 4*60*60 para 4 horas

static const char *TAG = "SALINIDADE";
float temperature = 25.0;
float tdsValue = 0.0;
float voltage = 0.0;

// histórico das últimas leituras (TDS)
#define HISTORY_SIZE 50
static float tds_history[HISTORY_SIZE];
static int hist_count = 0; // quantas leituras válidas temos (<= HISTORY_SIZE)
static int hist_idx = 0;   // próximo índice a gravar
// timestamp em micros (esp_timer_get_time) para cada leitura
static int64_t tds_ts_us[HISTORY_SIZE];

// fila usada apenas no modo SIMULATE_LORA
static QueueHandle_t lora_sim_queue = NULL;

httpd_handle_t server = NULL;

// Função para ler o sensor
void lerSalinidade(void) {
    uint32_t adcSum = 0;
    for (int i = 0; i < SAMPLES; i++) {
        adcSum += adc1_get_raw(TDS_PIN);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    float adcAverage = adcSum / (float)SAMPLES;
    voltage = (adcAverage / 4095.0) * VREF;
    float compensation = 1.0 + 0.02 * (temperature - TEMP_CORRECTION);
    tdsValue = (133.42 * pow(voltage, 3) - 255.86 * pow(voltage, 2) + 857.39 * voltage) * compensation;

    // registrar no histórico
    tds_history[hist_idx] = tdsValue;
    tds_ts_us[hist_idx] = esp_timer_get_time();
    hist_idx = (hist_idx + 1) % HISTORY_SIZE;
    if (hist_count < HISTORY_SIZE) hist_count++;
}

// Gerara a página HTML
static esp_err_t root_get_handler(httpd_req_t *req) {
    lerSalinidade();
    // construir array JS com histórico (do mais antigo ao mais recente)
    char js_data[1024];
    int pos = 0;
    for (int i = 0; i < hist_count; i++) {
        int idx = (hist_idx - hist_count + i + HISTORY_SIZE) % HISTORY_SIZE;
        if (i == 0) pos += snprintf(js_data + pos, sizeof(js_data) - pos, "%.2f", tds_history[idx]);
        else pos += snprintf(js_data + pos, sizeof(js_data) - pos, ",%.2f", tds_history[idx]);
    }

    // montar HTML maior para incluir lista e gráfico
    char resp[4096];
    snprintf(resp, sizeof(resp),
        "<!DOCTYPE html>"
        "<html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<meta http-equiv='refresh' content='3'>"
        "<title>Monitor de Salinidade</title>"
        "<style>body{font-family:Arial;margin:12px;} .row{display:flex;flex-wrap:wrap;} .card{padding:8px;margin:8px;border:1px solid #ccc;border-radius:6px;} .current{font-size:1.6em;font-weight:bold;} .history{max-height:220px;overflow:auto;background:#f9f9f9;padding:8px;} hr{margin:18px 0;} canvas{background:#fff;border:1px solid #ddd;}</style>"
        "</head><body>"
        "<h2>Sistema Medidor de Salinidade</h2>"
        "<div class='row'>"
        "  <div class='card' style='flex:1 1 220px;'>"
        "    <p><b>Leitura atual (Tensão):</b> <span class='current'>%.2f V</span></p>"
        "    <p><b>Leitura atual (TDS):</b> <span class='current'>%.0f ppm</span></p>"
        "  </div>"
        "  <div class='card' style='flex:1 1 360px;'>"
        "    <canvas id='chart' width='360' height='180'></canvas>"
        "  </div>"
        "</div>"
        "<hr/>"
        "<h3>Histórico (últimas %d leituras)</h3>"
        "<div class='history'><ol>" , voltage, tdsValue, hist_count);

    // anexar itens do histórico como lista ordenada
    int base_len = strlen(resp);
    char timestr[64];
    for (int i = 0; i < hist_count; i++) {
        int idx = (hist_idx - hist_count + i + HISTORY_SIZE) % HISTORY_SIZE;

        // format timestamp: prioriza wall-clock se disponível, caso contrário tempo desde boot
        time_t t_sec = (time_t)(tds_ts_us[idx] / 1000000);
        if (t_sec > 1600000000) {
            struct tm tm;
            localtime_r(&t_sec, &tm);
            strftime(timestr, sizeof(timestr), "%Y-%m-%d %H:%M:%S", &tm);
        } else {
            long s = (long)(tds_ts_us[idx] / 1000000); // tempo desde o boot
            snprintf(timestr, sizeof(timestr), "boot+%lds", s);
        }
        int n = snprintf(resp + base_len, sizeof(resp) - base_len, "<li>[%s] %.2f ppm</li>", timestr, tds_history[idx]);
        if (n < 0 || n >= (int)(sizeof(resp) - base_len)) break;
        base_len += n;
    }

    // fechar lista e adicionar script para desenhar gráfico
    snprintf(resp + base_len, sizeof(resp) - base_len,
        "</ol></div>"
        "<script>"
        "const data = [%s];"
        "const canvas = document.getElementById('chart');"
        "const ctx = canvas.getContext('2d');"
        "ctx.clearRect(0,0,canvas.width,canvas.height);"
        "if(data.length>0){"
        "  const w = canvas.width; const h = canvas.height;"
        "  const max = Math.max(...data); const min = Math.min(...data);"
        "  const range = (max - min) || 1;"
        "  ctx.beginPath();"
        "  for(let i=0;i<data.length;i++){"
        "    const x = (i/(data.length-1||1))*(w-20)+10;"
        "    const y = h-10-((data[i]-min)/range)*(h-20);"
        "    if(i==0) ctx.moveTo(x,y); else ctx.lineTo(x,y);"
        "  }"
        "  ctx.strokeStyle='#0074D9'; ctx.lineWidth=2; ctx.stroke();"
        "  // draw points\n"
        "  for(let i=0;i<data.length;i++){const x=(i/(data.length-1||1))*(w-20)+10;const y=h-10-((data[i]-min)/range)*(h-20);ctx.fillStyle='#0074D9';ctx.beginPath();ctx.arc(x,y,2,0,2*Math.PI);ctx.fill();}"
        "  // draw min/max labels\n"
        "  ctx.fillStyle='#333'; ctx.font='12px Arial'; ctx.fillText('max:'+max.toFixed(0),10,12); ctx.fillText('min:'+min.toFixed(0),w-70,h-2);"
        "} else { ctx.fillStyle='#666'; ctx.fillText('Sem dados',10,20); }"
        "</script>"
        "</body></html>", js_data);

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

//Inicia servidor HTTP 
httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = root_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &root);
        ESP_LOGI(TAG, "HTTP server started on port %d", config.server_port);
    }
    return server;
}

// Configura Access Point (cria a rede wifi)
void wifi_init_softap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "ESP32_Salinidade",
            .ssid_len = strlen("ESP32_Salinidade"),
            .channel = 1,
            .password = "12345678",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen("12345678") == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Access Point iniciado: SSID=%s senha=%s", wifi_config.ap.ssid, wifi_config.ap.password);

    /* Log the AP IP address so you can open the correct URL on your phone */
    esp_netif_t *ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (ap_netif) {
        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(ap_netif, &ip_info) == ESP_OK) {
            ESP_LOGI(TAG, "AP IP address: " IPSTR, IP2STR(&ip_info.ip));
        } else {
            ESP_LOGW(TAG, "Couldn't get AP IP info");
        }
    } else {
        ESP_LOGW(TAG, "esp_netif_get_handle_from_ifkey returned NULL for WIFI_AP_DEF");
    }
}

// Inicializa o LORA
static void lora_init(void) {
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_SPI_MISO,
        .mosi_io_num = PIN_SPI_MOSI,
        .sclk_io_num = PIN_SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 255,
    };
    ret = spi_bus_initialize(VSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %d", ret);
        return;
    }

    // configurar CS como gpio (driver do componente normalmente faz isto também)
    gpio_set_direction(PIN_SPI_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_SPI_CS, 1);

    // reset do módulo
    gpio_set_direction(PIN_SX127X_RESET, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_SX127X_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_SX127X_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

#if !SIMULATE_LORA
    sx127x_cfg_t cfg = {0};
#ifdef CONFIG_433MHZ
    cfg.freq = 433000000;
#elif defined(CONFIG_866MHZ)
    cfg.freq = 866000000;
#elif defined(CONFIG_915MHZ)
    cfg.freq = 915000000;
#else
    cfg.freq = 433000000; // default
#endif
    cfg.sync_word = 0x34;
    cfg.power = 17; 
    cfg.cs = PIN_SPI_CS;
    cfg.reset = PIN_SX127X_RESET;
    cfg.dio0 = PIN_SX127X_DIO0;
    cfg.spi_host = VSPI_HOST; 

    if (sx127x_init(&cfg) != ESP_OK) {
        ESP_LOGE(TAG, "sx127x_init falhou");
        return;
    }
    ESP_LOGI(TAG, "LoRa init: sx127x inicializado (freq=%d)", cfg.freq);
#else
    ESP_LOGI(TAG, "LoRa init: modo SIMULATE_LORA ativo");
#endif
}


// envia string por LoRa
static esp_err_t lora_send_values(float voltage, float tds)
{
    char payload[64];
    int len = snprintf(payload, sizeof(payload), "V:%.2f,TDS:%.0f", voltage, tds);
    if (len <= 0) return ESP_FAIL;

    ESP_LOGI(TAG, "Enviando por LoRa: %s", payload);

    // Modo de simulação: enfileira payload para uma task receptora local
#if SIMULATE_LORA
    if (lora_sim_queue) {
        payload[sizeof(payload)-1] = '\0'; // garantir terminação
        
        // enviar cópia (tamanho fixo da mensagem)
        if (xQueueSend(lora_sim_queue, payload, pdMS_TO_TICKS(100)) != pdPASS) {
            ESP_LOGW(TAG, "Fila simulada cheia, payload perdido");
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "Payload enfileirado (simulado)");
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "Modo SIMULATE_LORA ativo, mas fila não criada");
        return ESP_FAIL;
    }
#else
    esp_err_t r = sx127x_send((const uint8_t*)payload, len, pdMS_TO_TICKS(5000));
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "sx127x_send falhou: %d", r);
        return r;
    }

    ESP_LOGI(TAG, "Envio LORA concluído (driver sx127x)");
    return ESP_OK;
#endif
}


// Simula um receptor LoRa local (pega mensagens da fila e imprime)
static void lora_receiver_task(void *arg) {
    char buf[64];
    while (1) {
        if (xQueueReceive(lora_sim_queue, buf, portMAX_DELAY) == pdPASS) {
            ESP_LOGI(TAG, "[SIM_RX] Mensagem recebida: %s", buf);
            // seria onde processa a mensagem recebida, mas só temos 1 lora/esp
        }
    }
}


static void enter_deep_sleep_seconds(uint64_t seconds) {
    ESP_LOGI(TAG, "Entering deep sleep for %llu second(s)", seconds);
    // converte microsegundos
    esp_sleep_enable_timer_wakeup(seconds * 1000000ULL);
    esp_deep_sleep_start();
}

// envia periodicamente para manter o dispositivo ativo e a AP visível
static void lora_sender_task(void *arg) {
    while (1) {
        lerSalinidade();
        ESP_LOGI(TAG, "Leitura: %.0f ppm (%.2f V)", tdsValue, voltage);
        lora_send_values(voltage, tdsValue);
        // para testar hibernação, descomentar a linha abaixo manualmente
        // enter_deep_sleep_seconds(TEMPO_HIBERNACAO);
        vTaskDelay(pdMS_TO_TICKS(5000)); // enviar a cada 5s
    }
}



//region MAIN
void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(TDS_PIN, ADC_ATTEN_DB_11);

    wifi_init_softap();
    start_webserver();

    ESP_LOGI(TAG, "Servidor Web iniciado");

    // Inicializa LoRa e cria task de envio
    lora_init();
    
    // se estiver no modo simulado, criar fila e task receptor local
#if SIMULATE_LORA
    lora_sim_queue = xQueueCreate(8, 64);
    if (lora_sim_queue == NULL) {
        ESP_LOGW(TAG, "Falha ao criar fila simulada");
    } else {
        xTaskCreate(lora_receiver_task, "lora_receiver", 4096, NULL, 5, NULL);
    }
#endif

    xTaskCreate(lora_sender_task, "lora_sender", 4096, NULL, 5, NULL);

}




//
//
//
// Código leitor de temperatura-luminosidade-umidade e página web
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
