/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led_strip.h"

// 定义日志标签
static const char *TAG = "example";

// ESP32-S3-DevKitC-1 板载 RGB LED 连接在 GPIO 48
// 注意：在 N8R8 (Octal PSRAM) 型号上，GPIO 48 可能会与 PSRAM 冲突
#define RGB_LED_GPIO 48

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing RGB LED...");

    /* LED strip 初始化配置 */
    led_strip_config_t strip_config = {
        .strip_gpio_num = RGB_LED_GPIO,
        .max_leds = 1, // 只有一颗板载灯
        .led_model = LED_MODEL_WS2812, // WS2812 型号
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out = false,
    };
    
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    ESP_LOGI(TAG, "RGB LED initialized at GPIO %d", RGB_LED_GPIO);

    while (1) {
        // 红色
        ESP_LOGI(TAG, "Setting color: RED");
        led_strip_set_pixel(led_strip, 0, 20, 0, 0); // 降低亮度到 20，避免刺眼且省电
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // 绿色
        ESP_LOGI(TAG, "Setting color: GREEN");
        led_strip_set_pixel(led_strip, 0, 0, 20, 0);
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // 蓝色
        ESP_LOGI(TAG, "Setting color: BLUE");
        led_strip_set_pixel(led_strip, 0, 0, 0, 20);
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // 关闭
        ESP_LOGI(TAG, "Setting color: OFF");
        led_strip_clear(led_strip);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
