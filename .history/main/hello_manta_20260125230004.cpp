#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "led_strip.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "ble_server.h"
#include "shared_state.h"

// 模拟 Arduino 的 Serial 对象
class ArduinoSerial {
public:
    void begin(unsigned long baud) { /* ESP-IDF 默认初始化了 UART0 */ }
    void println(const char* s) { printf("%s\n", s); }
    void print(const char* s) { printf("%s", s); }
};
static ArduinoSerial Serial;

// ================= BTS7960 大功率驱动配置 =================
// 驱动器 1 (左)
#define MOTOR1_RPWM     4
#define MOTOR1_LPWM     5
// 驱动器 2 (右)
#define MOTOR2_RPWM     6
#define MOTOR2_LPWM     7
// R_EN 和 L_EN 建议直接在硬件上接驱动器的 5V VCC，节省 ESP32 引脚

// I2C (MPU6050)
#define I2C_MASTER_SDA_IO           15
#define I2C_MASTER_SCL_IO           16
#define I2C_MASTER_NUM              I2C_NUM_0

// UART (GPS)
#define GPS_UART_NUM                UART_NUM_1
#define GPS_TX_PIN                  17
#define GPS_RX_PIN                  18

static const char *TAG = "BACKEND";
led_strip_handle_t led_strip;

// 电机状态定义
MotorState motor1_state = M_STOP;
MotorState motor2_state = M_STOP;
int motor_step_delay = 20; // 默认中等速度 20ms

// 传感器数据全局变量
float current_lat = 0;
float current_lon = 0;
float pitch = 0, roll = 0, yaw = 0;

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

// 传感器读取任务：处理 MPU6050 和 GPS
void sensor_task(void *pvParameters) {
    uint8_t data[128];
    while(1) {
        // 1. 读取 GPS (UART)
        int len = uart_read_bytes(GPS_UART_NUM, data, sizeof(data) - 1, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0';
            // 简单的 NMEA 解析示例: 查找 $GNGGA 或 $GPRMC
            char *pos = strstr((char*)data, "$GNGGA");
            if (pos) {
                // 这里可以用 sscanf 解析经纬度，目前模拟逻辑
                ESP_LOGD(TAG, "GPS Raw: %s", pos);
            }
        }

        // 2. 读取 MPU6050 (I2C) - 仅做连通性简单示例
        // 实际开发建议引入 MPU6050 专门驱动库库 (如 i2cdev)
        uint8_t reg = 0x3B; // Accel X High
        uint8_t accel_bytes[6];
        i2c_master_write_read_device(I2C_MASTER_NUM, 0x68, &reg, 1, accel_bytes, 6, 100 / portTICK_PERIOD_MS);
        
        // 模拟计算
        pitch = (float)((int16_t)((accel_bytes[0] << 8) | accel_bytes[1])) / 16384.0f;
        roll  = (float)((int16_t)((accel_bytes[2] << 8) | accel_bytes[3])) / 16384.0f;

        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz 更新
    }
}

// Arduino 风格函数声明
void setup();
void loop();
void delay(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

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

    // --- 初始化 I2C (MPU6050) ---
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    // --- 初始化 UART (GPS) ---
    uart_config_t uart_conf = {};
    uart_conf.baud_rate = 9600; // GPS 默认为 9600
    uart_conf.data_bits = UART_DATA_8_BITS;
    uart_conf.parity = UART_PARITY_DISABLE;
    uart_conf.stop_bits = UART_STOP_BITS_1;
    uart_conf.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_param_config(GPS_UART_NUM, &uart_conf);
    uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(GPS_UART_NUM, 1024, 0, 0, NULL, 0);

    xTaskCreatePinnedToCore(motor_task, "motor_task", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(sensor_task, "sensor_task", 4096, NULL, 4, NULL, 1);

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
