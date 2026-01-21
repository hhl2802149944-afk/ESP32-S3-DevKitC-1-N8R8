#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "led_strip.h"

// 模拟 Arduino 的 Serial 对象
class ArduinoSerial {
public:
    void begin(unsigned long baud) { /* ESP-IDF 默认初始化了 UART0 */ }
    void println(const char* s) { printf("%s\n", s); }
    void print(const char* s) { printf("%s", s); }
};
static ArduinoSerial Serial;

// 引用嵌入的二进制文件
extern "C" {
    extern const uint8_t index_html_start[] asm("_binary_index_html_start");
    extern const uint8_t index_html_end[]   asm("_binary_index_html_end");
    extern const uint8_t style_css_start[]  asm("_binary_style_css_start");
    extern const uint8_t style_css_end[]    asm("_binary_style_css_end");
    extern const uint8_t main_js_start[]    asm("_binary_main_js_start");
    extern const uint8_t main_js_end[]      asm("_binary_main_js_end");
}

// ================= Arduino 风格配置 =================
#define WIFI_SSID      "Hank"
#define WIFI_PASS      "336161046"
#define RGB_LED_PIN    38

static const char *TAG = "BACKEND";
static led_strip_handle_t led_strip;

// Arduino 风格函数声明
void setup();
void loop();
void delay(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

// ================= Web Server (后端 API) =================

/* 处理静态文件请求 */
esp_err_t server_file_handler(httpd_req_t *req) {
    const char* filename = (const char*)req->user_ctx;
    const uint8_t* start;
    const uint8_t* end;
    const char* type;

    if (strcmp(filename, "index") == 0) {
        start = index_html_start; end = index_html_end; type = "text/html";
    } else if (strcmp(filename, "css") == 0) {
        start = style_css_start; end = style_css_end; type = "text/css";
    } else if (strcmp(filename, "js") == 0) {
        start = main_js_start; end = main_js_end; type = "application/javascript";
    } else { return ESP_FAIL; }

    httpd_resp_set_type(req, type);
    httpd_resp_send(req, (const char*)start, end - start);
    return ESP_OK;
}

/* API: 控制 LED */
esp_err_t api_set_color_handler(httpd_req_t *req) {
    char buf[64];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char val[32];
        if (httpd_query_key_value(buf, "rgb", val, sizeof(val)) == ESP_OK) {
            int r, g, b;
            sscanf(val, "%02x%02x%02x", &r, &g, &b);
            ESP_LOGI(TAG, "Backend received Color -> R:%d G:%d B:%d", r, g, b);
            led_strip_set_pixel(led_strip, 0, r, g, b);
            led_strip_refresh(led_strip);
        }
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"status\":\"ok\"}", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

void start_webserver() {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t index_uri = { .uri = "/", .method = HTTP_GET, .handler = server_file_handler, .user_ctx = (void*)"index" };
        httpd_register_uri_handler(server, &index_uri);

        httpd_uri_t css_uri = { .uri = "/style.css", .method = HTTP_GET, .handler = server_file_handler, .user_ctx = (void*)"css" };
        httpd_register_uri_handler(server, &css_uri);

        httpd_uri_t js_uri = { .uri = "/main.js", .method = HTTP_GET, .handler = server_file_handler, .user_ctx = (void*)"js" };
        httpd_register_uri_handler(server, &js_uri);

        httpd_uri_t api_uri = { .uri = "/api/set_color", .method = HTTP_GET, .handler = api_set_color_handler, .user_ctx = NULL };
        httpd_register_uri_handler(server, &api_uri);
    }
}

// ================= Wi-Fi 事件管理 =================
static void wifi_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) esp_wifi_connect();
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) data;
        ESP_LOGI(TAG, "Backend is Live! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        start_webserver();
    }
}

// ================= Arduino 核心逻辑 =================

void setup() {
    Serial.begin(115200); // 模拟 Arduino
    
    // 初始化 NVS
    ESP_ERROR_CHECK(nvs_flash_init());

    // 初始化 LED
    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = RGB_LED_PIN;
    strip_config.max_leds = 1;
    strip_config.led_model = LED_MODEL_WS2812;
    strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;

    led_strip_rmt_config_t rmt_config = {};
    rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;
    rmt_config.resolution_hz = 10 * 1000 * 1000;
    
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    // 连接 Wi-Fi
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL));
    
    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.sta.password, WIFI_PASS);
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static int last_blink = 0;
void loop() {
    // 这里可以放你的循环逻辑，比如串口心跳
    if (esp_log_timestamp() - last_blink > 5000) {
        ESP_LOGD(TAG, "Backend System Heartbeat...");
        last_blink = esp_log_timestamp();
    }
    delay(10); // 必须保留一小段延迟以喂狗
}

// 入口函数：桥接 ESP-IDF 与 Arduino 风格
extern "C" void app_main(void) {
    setup();
    while(1) {
        loop();
    }
}
