/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// 定义 LED 引脚 (Arduino 中的 LED1 = 1)
#define LED_PIN 1

void app_main(void)
{
    // --- 相当于 Arduino 的 setup() ---
    
    // 重置并设置 GPIO 方向
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    printf("Starting Blink Example (Arduino Style in ESP-IDF)\n");

    // --- 相当于 Arduino 的 loop() ---
    while (1) {
        // digitalWrite(LED1, HIGH);
        gpio_set_level(LED_PIN, 1);
        printf("LED ON\n");
        
        // delay(1000);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        // digitalWrite(LED1, LOW);
        gpio_set_level(LED_PIN, 0);
        printf("LED OFF\n");

        // delay(1000);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
