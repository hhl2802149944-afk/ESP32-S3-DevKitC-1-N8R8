#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "led_strip.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "driver/uart.h"
// #include "ble_server.h"
#include "web_server.h"
#include "shared_state.h"

// ================= 硬件引脚配置 (适配 IBT_2 接线) =================
#define RGB_LED_PIN     38
#define M1_RPWM_PIN     4   // Motor 1 Right PWM
#define M1_LPWM_PIN     5   // Motor 1 Left PWM
#define M2_RPWM_PIN     6   // Motor 2 Right PWM
#define M2_LPWM_PIN     7   // Motor 2 Left PWM

#define I2C_SDA_PIN     15  // IMU SDA
#define I2C_SCL_PIN     16  // IMU SCL

#define GPS_TX_PIN      17  // GPS UART TX (Connected to GPS RX)
#define GPS_RX_PIN      18  // GPS UART RX (Connected to GPS TX)
#define UART_NUM_GPS    UART_NUM_2

static const char *TAG = "MANTA_CORE";
led_strip_handle_t led_strip;

// 全局状态初始化
MotorState motor1_state = M_STOP;
MotorState motor2_state = M_STOP;
int motor_step_delay = 50; 
SensorData g_sensor_data = {}; // 初始化传感器数据

// ================= PWM (LEDC) 初始化 =================
void init_motors_pwm() {
    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode       = LEDC_LOW_SPEED_MODE;
    ledc_timer.duty_resolution  = LEDC_TIMER_10_BIT; // 0-1023
    ledc_timer.timer_num        = LEDC_TIMER_0;
    ledc_timer.freq_hz          = 20000;             // 20kHz 频率，无电感啸叫
    ledc_timer.clk_cfg          = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    // 通道配置
    ledc_channel_config_t ch0 = {
        .gpio_num = M1_RPWM_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .flags = {0}
    };
    ledc_channel_config_t ch1 = {
        .gpio_num = M1_LPWM_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .flags = {0}
    };
    ledc_channel_config_t ch2 = {
        .gpio_num = M2_RPWM_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_2,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .flags = {0}
    };
    ledc_channel_config_t ch3 = {
        .gpio_num = M2_LPWM_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_3,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .flags = {0}
    };

    ledc_channel_config(&ch0);
    ledc_channel_config(&ch1);
    ledc_channel_config(&ch2);
    ledc_channel_config(&ch3);
}

// 核心电机控制函数
void set_motor(int motor_id, MotorState state, int speed_pct) {
    if (speed_pct < 0) speed_pct = 0;
    if (speed_pct > 100) speed_pct = 100;
    uint32_t duty = (speed_pct * 1023) / 100;

    if (motor_id == 1) {
        if (state == M_FORWARD) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
        } else if (state == M_BACKWARD) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
        } else {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
        }
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    } else {
        if (state == M_FORWARD) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, duty);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 0);
        } else if (state == M_BACKWARD) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, duty);
        } else {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 0);
        }
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
    }
}

// ================= 传感器初始化 =================
void init_sensors() {
    // I2C (IMU)
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA_PIN;
    conf.scl_io_num = I2C_SCL_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);

    // UART (GPS)
    uart_config_t uart_conf = {};
    uart_conf.baud_rate = 9600;
    uart_conf.data_bits = UART_DATA_8_BITS;
    uart_conf.parity    = UART_PARITY_DISABLE;
    uart_conf.stop_bits = UART_STOP_BITS_1;
    uart_conf.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_conf.source_clk = UART_SCLK_DEFAULT;
    uart_param_config(UART_NUM_2, &uart_conf);
    uart_set_pin(UART_NUM_2, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_2, 1024 * 2, 0, 0, NULL, 0);
}

// 电机控制循环，持续同步状态
void motor_task(void *pvParameters) {
    while(1) {
        // 将 motor_step_delay (10-100) 映射回速度百分比
        // 之前 10ms 是最快(100%)，100ms 是最慢(10%)
        int speed = 110 - motor_step_delay;
        if (speed < 10) speed = 10;
        if (speed > 100) speed = 100;

        set_motor(1, motor1_state, speed);
        set_motor(2, motor2_state, speed);
        
        vTaskDelay(pdMS_TO_TICKS(50)); 
    }
}

// 传感器采集任务 (模拟/读取逻辑)
void sensor_task(void *pvParameters) {
    while(1) {
        // --- 模拟 IMU 数据 (实际开发时这里调用 i2c_master_transmit/receive) ---
        g_sensor_data.acc_x = (rand() % 100) / 50.0f;
        g_sensor_data.acc_y = (rand() % 100) / 50.0f;
        g_sensor_data.acc_z = 9.8f + (rand() % 20) / 100.0f;
        
        // --- 模拟 GPS 数据 ---
        g_sensor_data.gps_lat = 31.23 + (rand() % 1000) / 1000000.0;
        g_sensor_data.gps_lon = 121.47 + (rand() % 1000) / 1000000.0;
        g_sensor_data.gps_sats = 8;

        vTaskDelay(pdMS_TO_TICKS(200)); 
    }
}

extern "C" void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    init_motors_pwm();
    init_sensors();
    
    // 启动 WiFi 和 Web Server
    init_wifi_ap();
    start_web_server();

    // 初始化 RGB
    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = RGB_LED_PIN;
    strip_config.max_leds = 1;
    strip_config.led_model = LED_MODEL_WS2812;
    strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;
    led_strip_rmt_config_t rmt_config = {};
    rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;
    rmt_config.resolution_hz = 10 * 1000 * 1000;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    // 启动任务
    xTaskCreatePinnedToCore(motor_task, "motor_task", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(sensor_task, "sensor_task", 4096, NULL, 4, NULL, 1);
    
    // 启动蓝牙
    // start_ble_server();
    
    ESP_LOGI(TAG, "Manta 24V PWM System Ready with Dashboard.");
}
