/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"

// ESP32-S3-DevKitC-1 板载 RGB LED 连接在 GPIO 48
#define RGB_LED_GPIO 48

void app_main(void)
{
    printf("Initializing RGB LED...\n");

    /* LED strip 初始化配置 */
    led_strip_config_t strip_config = {
        .strip_gpio_num = RGB_LED_GPIO,
        .max_leds = 1, // 只有一个板载 LED
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // WS2812 标准格式
        .led_model = LED_MODEL_WS2812,            // 明确指定型号
        .flags.invert_out = false,                // 不反转输出
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,    // 明确指定时钟源
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,           // 只有一颗灯，不需要 DMA
    };
    
    led_strip_handle_t led_strip;
    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
    if (err != ESP_OK) {
        printf("LED Strip init failed: %s\n", esp_err_to_name(err));
        return;
    }

    printf("RGB LED initialized at GPIO %d\n", RGB_LED_GPIO);

    while (1) {
        // 红色
        printf("Setting color: RED\n");
        led_strip_set_pixel(led_strip, 0, 255, 0, 0); 
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // 绿色
        printf("Setting color: GREEN\n");
        led_strip_set_pixel(led_strip, 0, 0, 255, 0);
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // 蓝色
        printf("Setting color: BLUE\n");
        led_strip_set_pixel(led_strip, 0, 0, 0, 255);
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // 关闭
        printf("Setting color: OFF\n");
        led_strip_clear(led_strip);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
