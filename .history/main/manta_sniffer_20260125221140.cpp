#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "manta_sniffer.h"

static const char *TAG = "MANTA_SNIFFER";

#define SNIFFER_TX_PIN 17
#define SNIFFER_RX_PIN 18
#define BUF_SIZE (1024)

static void sniffer_task(void *pvParameters) {
    ESP_LOGI(TAG, "Starting Sniffer Task on RX:%d", SNIFFER_RX_PIN);
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE);
    
    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            // Log as string if possible
            ESP_LOGI(TAG, "Data Received (%d bytes):", len);
            
            // Helpful for human-readable ASCII commands
            ESP_LOGI(TAG, "ASCII: %.*s", len, data);
            
            // Crucial for binary protocols (like the one we saw in the iOS app)
            ESP_LOG_BUFFER_HEX(TAG, data, len);
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Prevent CPU hogging
    }
    free(data);
}

void init_manta_sniffer() {
    const uart_config_t uart_config = {
        .baud_rate = 115200, // Try 115200 first, common for ESP/Camera
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // We use UART_NUM_1 so UART_NUM_0 stays for PC Debugging
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, SNIFFER_TX_PIN, SNIFFER_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    xTaskCreate(sniffer_task, "sniffer_task", 4096, NULL, 10, NULL);
}
