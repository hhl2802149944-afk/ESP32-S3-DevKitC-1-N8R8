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
    /* LED strip 初始化配置 */
    led_strip_config_t strip_config = {
        .strip_gpio_num = RGB_LED_GPIO,
        .max_leds = 1, // 只有一个板载 LED
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
    };
    
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    printf("Starting RGB Blink Example\n");

    while (1) {
        // 红色
        printf("RGB: RED\n");
        led_strip_set_pixel(led_strip, 0, 255, 0, 0); // R, G, B
        led_strip_refresh(led_strip);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        // 绿色
        printf("RGB: GREEN\n");
        led_strip_set_pixel(led_strip, 0, 0, 255, 0);
        led_strip_refresh(led_strip);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        // 蓝色
        printf("RGB: BLUE\n");
        led_strip_set_pixel(led_strip, 0, 0, 0, 255);
        led_strip_refresh(led_strip);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        
        // 关闭
        printf("RGB: OFF\n");
        led_strip_clear(led_strip);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
