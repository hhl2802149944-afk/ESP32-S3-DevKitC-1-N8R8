#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "led_strip.h"
#include "driver/gpio.h"

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

// 电机 1 引脚 
#define MOTOR1_AIN1     4
#define MOTOR1_AIN2     5
#define MOTOR1_PWMA     6
#define MOTOR1_BIN1     7
#define MOTOR1_BIN2     9
#define MOTOR1_PWMB     10
#define MOTOR1_STBY     21

// 电机 2 引脚 (完全避开 GPIO 12/MTDI, 15-18/SPI 以及 33-42/Octal-SPI)
#define MOTOR2_AIN1     1
#define MOTOR2_AIN2     2
#define MOTOR2_PWMA     3
#define MOTOR2_BIN1     8
#define MOTOR2_BIN2     11
#define MOTOR2_PWMB     13
#define MOTOR2_STBY     14

static const char *TAG = "BACKEND";
static led_strip_handle_t led_strip;

// 电机状态定义
enum MotorState { M_STOP, M_FORWARD, M_BACKWARD };
static MotorState motor1_state = M_STOP;
static MotorState motor2_state = M_STOP;
static int motor_step_delay = 50; // 稍微加快一点，但保持稳定

// 步进脉冲任务：采用更稳健的驱动逻辑
void motor_task(void *pvParameters) {
    int step1 = 0;
    int step2 = 0;
    while(1) {
        // ... (电机 1 逻辑保持不变) ...
        if (motor1_state != M_STOP) {
            gpio_set_level((gpio_num_t)MOTOR1_STBY, 1); 
            gpio_set_level((gpio_num_t)MOTOR1_PWMA, 1);
            gpio_set_level((gpio_num_t)MOTOR1_PWMB, 1);
            
            if (motor1_state == M_FORWARD) step1++; else step1--;
            int s = (step1 % 4 + 4) % 4; 

            if(s==0){gpio_set_level((gpio_num_t)MOTOR1_AIN1,1); gpio_set_level((gpio_num_t)MOTOR1_AIN2,0); gpio_set_level((gpio_num_t)MOTOR1_BIN1,1); gpio_set_level((gpio_num_t)MOTOR1_BIN2,0);}
            else if(s==1){gpio_set_level((gpio_num_t)MOTOR1_AIN1,0); gpio_set_level((gpio_num_t)MOTOR1_AIN2,1); gpio_set_level((gpio_num_t)MOTOR1_BIN1,1); gpio_set_level((gpio_num_t)MOTOR1_BIN2,0);}
            else if(s==2){gpio_set_level((gpio_num_t)MOTOR1_AIN1,0); gpio_set_level((gpio_num_t)MOTOR1_AIN2,1); gpio_set_level((gpio_num_t)MOTOR1_BIN1,0); gpio_set_level((gpio_num_t)MOTOR1_BIN2,1);}
            else if(s==3){gpio_set_level((gpio_num_t)MOTOR1_AIN1,1); gpio_set_level((gpio_num_t)MOTOR1_AIN2,0); gpio_set_level((gpio_num_t)MOTOR1_BIN1,0); gpio_set_level((gpio_num_t)MOTOR1_BIN2,1);}
        } else {
            gpio_set_level((gpio_num_t)MOTOR1_STBY, 0); 
            gpio_set_level((gpio_num_t)MOTOR1_AIN1, 0); gpio_set_level((gpio_num_t)MOTOR1_AIN2, 0);
            gpio_set_level((gpio_num_t)MOTOR1_BIN1, 0); gpio_set_level((gpio_num_t)MOTOR1_BIN2, 0);
            gpio_set_level((gpio_num_t)MOTOR1_PWMA, 0); gpio_set_level((gpio_num_t)MOTOR1_PWMB, 0);
        }

        // 电机 2: 双相通电 (Full Step)
        if (motor2_state != M_STOP) {
            gpio_set_level((gpio_num_t)MOTOR2_STBY, 1);
            gpio_set_level((gpio_num_t)MOTOR2_PWMA, 1);
            gpio_set_level((gpio_num_t)MOTOR2_PWMB, 1);
            
            if (motor2_state == M_FORWARD) step2++; else step2--;
            int s = (step2 % 4 + 4) % 4; 

            if(s==0){gpio_set_level((gpio_num_t)MOTOR2_AIN1,1); gpio_set_level((gpio_num_t)MOTOR2_AIN2,0); gpio_set_level((gpio_num_t)MOTOR2_BIN1,1); gpio_set_level((gpio_num_t)MOTOR2_BIN2,0);}
            else if(s==1){gpio_set_level((gpio_num_t)MOTOR2_AIN1,0); gpio_set_level((gpio_num_t)MOTOR2_AIN2,1); gpio_set_level((gpio_num_t)MOTOR2_BIN1,1); gpio_set_level((gpio_num_t)MOTOR2_BIN2,0);}
            else if(s==2){gpio_set_level((gpio_num_t)MOTOR2_AIN1,0); gpio_set_level((gpio_num_t)MOTOR2_AIN2,1); gpio_set_level((gpio_num_t)MOTOR2_BIN1,0); gpio_set_level((gpio_num_t)MOTOR2_BIN2,1);}
            else if(s==3){gpio_set_level((gpio_num_t)MOTOR2_AIN1,1); gpio_set_level((gpio_num_t)MOTOR2_AIN2,0); gpio_set_level((gpio_num_t)MOTOR2_BIN1,0); gpio_set_level((gpio_num_t)MOTOR2_BIN2,1);}
        } else {
            gpio_set_level((gpio_num_t)MOTOR2_STBY, 0); 
            gpio_set_level((gpio_num_t)MOTOR2_AIN1, 0); gpio_set_level((gpio_num_t)MOTOR2_AIN2, 0);
            gpio_set_level((gpio_num_t)MOTOR2_BIN1, 0); gpio_set_level((gpio_num_t)MOTOR2_BIN2, 0);
            gpio_set_level((gpio_num_t)MOTOR2_PWMA, 0); gpio_set_level((gpio_num_t)MOTOR2_PWMB, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(motor_step_delay)); 
    }
}

// Arduino 风格函数声明
void setup();
void loop();
void delay(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

// ================= Web Server (后端 API) =================

/* 处理静态文件请求 */
esp_err_t server_file_handler(httpd_req_t *req) {
    const char* filename = (const char*)req->user_ctx;
    const uint8_t* start = NULL;
    const uint8_t* end = NULL;
    const char* type = "text/plain";

    if (strcmp(filename, "index") == 0) {
        start = index_html_start; end = index_html_end; type = "text/html; charset=utf-8";
    } else if (strcmp(filename, "css") == 0) {
        start = style_css_start; end = style_css_end; type = "text/css; charset=utf-8";
    } else if (strcmp(filename, "js") == 0) {
        start = main_js_start; end = main_js_end; type = "application/javascript; charset=utf-8";
    }

    if (start == NULL || end == NULL || start >= end) {
        ESP_LOGE(TAG, "File logic error for: %s", filename);
        httpd_resp_send_404(req);
        return ESP_OK;
    }

    // 严谨计算长度：排除可能存在的内存脏数据
    size_t size = (size_t)(end - start);
    
    // 如果末尾是 0x00，通常是嵌入工具多加的，裁掉它以免 JS 报错
    if (size > 0 && start[size-1] == 0x00) {
        size--;
    }

    ESP_LOGI(TAG, "Serving: %s (%u bytes)", filename, (unsigned int)size);
    
    char len_str[32];
    snprintf(len_str, sizeof(len_str), "%u", (unsigned int)size);
    httpd_resp_set_hdr(req, "Content-Length", len_str);
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
    httpd_resp_set_type(req, type);
    
    return httpd_resp_send(req, (const char*)start, size);
}

/* API: 控制 LED */
esp_err_t api_set_color_handler(httpd_req_t *req) {
    char buf[64] = {0};
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char val[32] = {0};
        if (httpd_query_key_value(buf, "rgb", val, sizeof(val)) == ESP_OK) {
            int r, g, b;
            if (sscanf(val, "%02x%02x%02x", &r, &g, &b) == 3) {
                ESP_LOGI(TAG, "Color Set -> R:%d G:%d B:%d", r, g, b);
                led_strip_set_pixel(led_strip, 0, r, g, b);
                led_strip_refresh(led_strip);
            }
        }
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"status\":\"ok\"}", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* API: 控制电机 */
esp_err_t api_motor_handler(httpd_req_t *req) {
    char buf[128] = {0};
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        ESP_LOGI(TAG, "Raw Query: %s", buf); // 打印原始参数
        char dir[32] = {0};
        char item[16] = {0};
        httpd_query_key_value(buf, "dir", dir, sizeof(dir));
        httpd_query_key_value(buf, "id", item, sizeof(item));

        int motor_id = (strlen(item) > 0) ? atoi(item) : 1;
        MotorState next_state = M_STOP;
        if (strcmp(dir, "forward") == 0) next_state = M_FORWARD;
        else if (strcmp(dir, "backward") == 0) next_state = M_BACKWARD;

        if (motor_id == 2) {
            motor2_state = next_state;
            ESP_LOGW(TAG, "ID=2 parsed | Dir=%s", dir);
        } else {
            motor1_state = next_state;
            ESP_LOGW(TAG, "ID=1 parsed | Dir=%s", dir);
        }
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"status\":\"motor_ok\"}", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

void start_webserver() {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.stack_size = 10240; // 继续增加栈空间到 10KB
    config.max_uri_handlers = 10;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t index_uri = { .uri = "/", .method = HTTP_GET, .handler = server_file_handler, .user_ctx = (void*)"index" };
        httpd_register_uri_handler(server, &index_uri);

        httpd_uri_t css_uri = { .uri = "/style.css", .method = HTTP_GET, .handler = server_file_handler, .user_ctx = (void*)"css" };
        httpd_register_uri_handler(server, &css_uri);

        httpd_uri_t js_uri = { .uri = "/main.js", .method = HTTP_GET, .handler = server_file_handler, .user_ctx = (void*)"js" };
        httpd_register_uri_handler(server, &js_uri);

        httpd_uri_t api_uri = { .uri = "/api/set_color", .method = HTTP_GET, .handler = api_set_color_handler, .user_ctx = NULL };
        httpd_register_uri_handler(server, &api_uri);

        httpd_uri_t motor_uri = { .uri = "/api/motor", .method = HTTP_GET, .handler = api_motor_handler, .user_ctx = NULL };
        httpd_register_uri_handler(server, &motor_uri);
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
    
    // 初始化电机引脚
    gpio_config_t motor_conf = {};
    motor_conf.pin_bit_mask = (1ULL << MOTOR1_AIN1) | (1ULL << MOTOR1_AIN2) | (1ULL << MOTOR1_PWMA) |
                              (1ULL << MOTOR1_BIN1) | (1ULL << MOTOR1_BIN2) | (1ULL << MOTOR1_PWMB) |
                              (1ULL << MOTOR1_STBY) | (1ULL << MOTOR2_AIN1) | (1ULL << MOTOR2_AIN2) |
                              (1ULL << MOTOR2_PWMA) | (1ULL << MOTOR2_BIN1) | (1ULL << MOTOR2_BIN2) |
                              (1ULL << MOTOR2_PWMB) | (1ULL << MOTOR2_STBY);
    motor_conf.mode = GPIO_MODE_OUTPUT;
    motor_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    motor_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    motor_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&motor_conf);

    // 默认关闭使能，节省功耗 (只有点击按钮时才开启)
    gpio_set_level((gpio_num_t)MOTOR1_STBY, 0);
    gpio_set_level((gpio_num_t)MOTOR1_PWMA, 1);
    gpio_set_level((gpio_num_t)MOTOR1_PWMB, 1);
    gpio_set_level((gpio_num_t)MOTOR2_STBY, 0);
    gpio_set_level((gpio_num_t)MOTOR2_PWMA, 1);
    gpio_set_level((gpio_num_t)MOTOR2_PWMB, 1);

    // 启动电机控制任务 (双轴平滑控制)
    ESP_LOGI(TAG, "Creating Motor Task on Core 1...");
    xTaskCreatePinnedToCore(motor_task, "motor_task", 2048, NULL, 5, NULL, 1);

    // 初始化 NVS
    ESP_LOGI(TAG, "Initializing WiFi on Core 0 (default)...");
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
