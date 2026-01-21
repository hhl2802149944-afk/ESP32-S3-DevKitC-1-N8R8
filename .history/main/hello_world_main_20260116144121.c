/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "led_strip.h"

// =================配置建议=================
#define WIFI_SSID      "Hank"      // 修改为你的热点名称
#define WIFI_PASS      "336161046"  // 修改为你的热点密码

// 如果灯不亮，请依次尝试修改此值为: 48, 38, 18, 1
#define RGB_LED_GPIO   38                    
// ==========================================

static const char *TAG = "WEB_LED";
static led_strip_handle_t led_strip;

/* HTML 页面内容 */
const char* index_html = "<!DOCTYPE html><html><head><meta charset=\"UTF-8\">"
    "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
    "<link rel=\"icon\" href=\"data:,\">"
    "<title>ESP32-S3 RGB Control</title>"
    "<style>body { font-family: sans-serif; text-align: center; background: #fff; color: #333; }"
    ".container { padding: 40px 20px; max-width: 400px; margin: 40px auto; border: 1px solid #ddd; border-radius: 10px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); }"
    "input[type=color] { width: 100%; height: 100px; border: 2px solid #333; cursor: pointer; margin-bottom: 20px; }"
    ".btn { background: #007bff; color: white; border: none; padding: 15px 30px; font-size: 18px; border-radius: 5px; cursor: pointer; width: 100%; }"
    ".btn:active { background: #0056b3; }"
    "h1 { font-size: 24px; }</style></head><body>"
    "<div class=\"container\"><h1>RGB LED 控制面板</h1>"
    "<p>选择颜色后点击下方按钮生效</p>"
    "<input type=\"color\" id=\"picker\" value=\"#ff0000\">"
    "<br><br>"
    "<button class=\"btn\" onclick=\"updateColor()\">确认更改</button>"
    "<p id=\"status\" style=\"margin-top:20px; color:#666;\"></p>"
    "</div><script>"
    "function updateColor() {"
    "  var color = document.getElementById('picker').value.substring(1);"
    "  var status = document.getElementById('status');"
    "  status.innerText = '正在发送指令...';"
    "  fetch('/set_color?rgb=' + color).then(r => {"
    "    if(r.ok) status.innerText = '设置成功！状态：' + color.toUpperCase();"
    "    else status.innerText = '发送失败，请检查网络';"
    "  }).catch(e => { status.innerText = '请求超时';"
    "  });"
    "}</script></body></html>";

/* URI 句柄：返回首页 */
esp_err_t get_handler(httpd_req_t *req) {
    httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* URI 句柄：设置颜色 */
esp_err_t set_color_handler(httpd_req_t *req) {
    char buf[32];
    if (httpd_query_key_value(req->uri, "rgb", buf, sizeof(buf)) == ESP_OK) {
        int r, g, b;
        sscanf(buf, "%02x%02x%02x", &r, &g, &b);
        ESP_LOGI(TAG, "Setting RGB: R:%d G:%d B:%d", r, g, b);
        led_strip_set_pixel(led_strip, 0, r, g, b);
        led_strip_refresh(led_strip);
    }
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* 启动 HTTP 服务器 */
void start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t index_uri = { .uri = "/", .method = HTTP_GET, .handler = get_handler, .user_ctx = NULL };
        httpd_register_uri_handler(server, &index_uri);

        httpd_uri_t color_uri = { .uri = "/set_color", .method = HTTP_GET, .handler = set_color_handler, .user_ctx = NULL };
        httpd_register_uri_handler(server, &color_uri);
        ESP_LOGI(TAG, "Web server started!");
    }
}

/* Wi-Fi 事件回调 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Disconnected, retrying...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        start_webserver();
    }
}

void app_main(void) {
    // 1. 初始化 NVS (必须)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. 初始化 LED Strip
    led_strip_config_t strip_config = {
        .strip_gpio_num = RGB_LED_GPIO,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip); 

    // 3. 初始化网络堆栈并连接 Wi-Fi
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi setup finished. Waiting for connection...");
}
