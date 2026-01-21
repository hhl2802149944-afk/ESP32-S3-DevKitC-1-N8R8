#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "led_strip.h"
#include "driver/gpio.h"
#include "web_server.h"

// 模拟 Arduino 的 Serial 对象
class ArduinoSerial {
public:
    void begin(unsigned long baud) { /* ESP-IDF 默认初始化了 UART0 */ }
    void println(const char* s) { printf("%s\n", s); }
    void print(const char* s) { printf("%s", s); }
};
static ArduinoSerial Serial;

// ================= Arduino 风格配置 =================
#define WIFI_SSID      "Hank"
#define WIFI_PASS      "336161046"
#define RGB_LED_PIN    38

// 电机 1 引脚 (验证通过)
#define MOTOR1_AIN1     4
#define MOTOR1_AIN2     5
#define MOTOR1_PWMA     6
#define MOTOR1_BIN1     7
#define MOTOR1_BIN2     9
#define MOTOR1_PWMB     10
#define MOTOR1_STBY     21

// 电机 2 引脚 (使用 1, 2, 3, 8, 14, 45, 46 安全 IO)
#define MOTOR2_AIN1     1
#define MOTOR2_AIN2     2
#define MOTOR2_PWMA     3
#define MOTOR2_BIN1     8
#define MOTOR2_BIN2     11
#define MOTOR2_PWMB     13
#define MOTOR2_STBY     14

static const char *TAG = "BACKEND";
led_strip_handle_t led_strip;

// 电机状态定义
MotorState motor1_state = M_STOP;
MotorState motor2_state = M_STOP;
static int motor_step_delay = 60; 

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
    vTaskDelay(pdMS_TO_TICKS(2000)); 
    printf("\n\n[SYSTEM] --- ESP32-S3 BACKEND STARTING ---\n\n");
    
    Serial.begin(115200); 
    
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

    // 默认状态
    gpio_set_level((gpio_num_t)MOTOR1_STBY, 0);
    gpio_set_level((gpio_num_t)MOTOR1_PWMA, 1);
    gpio_set_level((gpio_num_t)MOTOR1_PWMB, 1);
    gpio_set_level((gpio_num_t)MOTOR2_STBY, 0);
    gpio_set_level((gpio_num_t)MOTOR2_PWMA, 1);
    gpio_set_level((gpio_num_t)MOTOR2_PWMB, 1);

    xTaskCreatePinnedToCore(motor_task, "motor_task", 2048, NULL, 5, NULL, 1);

    ESP_LOGI(TAG, "Initializing WiFi...");
    ESP_ERROR_CHECK(nvs_flash_init());

    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = RGB_LED_PIN;
    strip_config.max_leds = 1;
    strip_config.led_model = LED_MODEL_WS2812;
    strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;

    led_strip_rmt_config_t rmt_config = {};
    rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;
    rmt_config.resolution_hz = 10 * 1000 * 1000;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

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
    if (esp_log_timestamp() - last_blink > 5000) {
        ESP_LOGD(TAG, "Backend System Heartbeat...");
        last_blink = esp_log_timestamp();
    }
    delay(10);
}

extern "C" void app_main(void) {
    setup();
    while(1) {
        loop();
    }
}
