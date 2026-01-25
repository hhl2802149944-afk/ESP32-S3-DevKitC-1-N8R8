#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "led_strip.h"
#include "driver/gpio.h"
#include "ble_server.h"
#include "shared_state.h"
#include "manta_sniffer.h"

// 模拟 Arduino 的 Serial 对象
class ArduinoSerial {
public:
    void begin(unsigned long baud) { /* ESP-IDF 默认初始化了 UART0 */ }
    void println(const char* s) { printf("%s\n", s); }
    void print(const char* s) { printf("%s", s); }
};
static ArduinoSerial Serial;

// ================= Arduino 风格配置 =================
#define RGB_LED_PIN    38

// ================= 直流电机引脚配置 =================
#define MOTOR1_IN1     4
#define MOTOR1_IN2     5
#define MOTOR2_IN1     6
#define MOTOR2_IN2     7
#define MOTOR3_IN1     1  // 摄像头电机
#define MOTOR3_IN2     2
#define MOT_STBY      21  // 驱动使能引脚 (如果有)

// 串口摄像头引脚
#define CAM_TX        17
#define CAM_RX        18

static const char *TAG = "BACKEND";
led_strip_handle_t led_strip;

MotorState motor1_state = M_STOP;
MotorState motor2_state = M_STOP;
MotorState motor3_state = M_STOP;
int motor_step_delay = 20;

// 直流电机控制函数
void set_dc_motor(int pin1, int pin2, MotorState state) {
    if (state == M_FORWARD) {
        gpio_set_level((gpio_num_t)pin1, 1);
        gpio_set_level((gpio_num_t)pin2, 0);
    } else if (state == M_BACKWARD) {
        gpio_set_level((gpio_num_t)pin1, 0);
        gpio_set_level((gpio_num_t)pin2, 1);
    } else {
        gpio_set_level((gpio_num_t)pin1, 0);
        gpio_set_level((gpio_num_t)pin2, 0);
    }
}

void motor_task(void *pvParameters) {
    gpio_set_level((gpio_num_t)MOT_STBY, 1); 
    while(1) {
        set_dc_motor(MOTOR1_IN1, MOTOR1_IN2, motor1_state);
        set_dc_motor(MOTOR2_IN1, MOTOR2_IN2, motor2_state);
        set_dc_motor(MOTOR3_IN1, MOTOR3_IN2, motor3_state);
        vTaskDelay(pdMS_TO_TICKS(50)); // 直流电机刷新率不需要太快
    }
}

// Arduino 风格函数声明
void setup();
void loop();
void delay(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

// ================= Arduino 核心逻辑 =================

void setup() {
    vTaskDelay(pdMS_TO_TICKS(2000)); 
    printf("\n\n[SYSTEM] --- ESP32-S3 HACKING STARTING ---\n\n");
    
    Serial.begin(115200); 
    
    // 初始化直流电机引脚
    gpio_config_t motor_conf = {};
    motor_conf.pin_bit_mask = (1ULL << MOTOR1_IN1) | (1ULL << MOTOR1_IN2) | 
                              (1ULL << MOTOR2_IN1) | (1ULL << MOTOR2_IN2) |
                              (1ULL << MOTOR3_IN1) | (1ULL << MOTOR3_IN2) |
                              (1ULL << MOT_STBY);
    motor_conf.mode = GPIO_MODE_OUTPUT;
    motor_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    motor_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    motor_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&motor_conf);

    // 默认关闭电机
    gpio_set_level((gpio_num_t)MOTOR1_IN1, 0);
    gpio_set_level((gpio_num_t)MOTOR1_IN2, 0);
    gpio_set_level((gpio_num_t)MOTOR2_IN1, 0);
    gpio_set_level((gpio_num_t)MOTOR2_IN2, 0);
    gpio_set_level((gpio_num_t)MOTOR3_IN1, 0);
    gpio_set_level((gpio_num_t)MOTOR3_IN2, 0);
    gpio_set_level((gpio_num_t)MOT_STBY, 1);

    xTaskCreatePinnedToCore(motor_task, "motor_task", 4096, NULL, 5, NULL, 1);
    
    // 初始化摄像头串口 (针对 T, R 接口)
    // 这里可以在 loop 或单独任务中通过 UART1 发送指令
    ESP_LOGI(TAG, "Initializing Camera Serial on TX:%d RX:%d", CAM_TX, CAM_RX);

    ESP_LOGI(TAG, "Initializing BLE Control...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = RGB_LED_PIN;
    strip_config.max_leds = 1;
    strip_config.led_model = LED_MODEL_WS2812;
    strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;

    led_strip_rmt_config_t rmt_config = {};
    rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;
    rmt_config.resolution_hz = 10 * 1000 * 1000;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    start_ble_server();
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
