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

// 用户确认 38 是正确的引脚
#define RGB_LED_GPIO   38                    
// ==========================================

static const char *TAG = "WEB_LED";
static led_strip_handle_t led_strip;

/* HTML 页面内容 */
const char* index_html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="icon" href="data:,">
    <title>ESP32-S3 Controller</title>
    <style>
        :root { --primary: #4361ee; --bg: #f8f9fa; --card: #ffffff; }
        body { font-family: 'Segoe UI', system-ui, sans-serif; background: var(--bg); margin: 0; display: flex; align-items: center; justify-content: center; min-height: 100vh; }
        .card { background: var(--card); padding: 2rem; border-radius: 1.5rem; box-shadow: 0 10px 25px rgba(0,0,0,0.05); width: 90%; max-width: 400px; text-align: center; }
        h1 { font-size: 1.5rem; margin-bottom: 0.5rem; color: #1a1a1a; }
        p { color: #666; margin-bottom: 2rem; font-size: 0.9rem; }
        .picker-container { position: relative; width: 100%; aspect-ratio: 1; margin-bottom: 2rem; }
        input[type=color] { appearance: none; -webkit-appearance: none; border: none; width: 100%; height: 100%; cursor: pointer; background: none; border-radius: 1rem; }
        input[type=color]::-webkit-color-swatch-wrapper { padding: 0; }
        input[type=color]::-webkit-color-swatch { border: 8px solid rgba(0,0,0,0.05); border-radius: 1rem; transition: transform 0.2s; }
        input[type=color]:hover::-webkit-color-swatch { transform: scale(1.02); }
        .btn { background: var(--primary); color: white; border: none; padding: 1rem 2rem; font-size: 1rem; font-weight: 600; border-radius: 0.75rem; cursor: pointer; width: 100%; transition: all 0.2s; box-shadow: 0 4px 15px rgba(67, 97, 238, 0.3); }
        .btn:active { transform: scale(0.98); box-shadow: 0 2px 8px rgba(67, 97, 238, 0.2); }
        #status { margin-top: 1.5rem; font-size: 0.85rem; padding: 0.75rem; border-radius: 0.5rem; display: none; }
        .success { display: block !important; background: #dcfce7; color: #166534; }
        .loading { display: block !important; background: #e0e7ff; color: #3730a3; }
    </style>
</head>
<body>
    <div class="card">
        <h1>RGB LED 控制</h1>
        <p>选择颜色并点击下方按钮同步至硬件</p>
        <div class="picker-container">
            <input type="color" id="picker" value="#4361ee">
        </div>
        <button class="btn" onclick="updateColor()">同步颜色</button>
        <div id="status"></div>
    </div>
    <script>
        function updateColor() {
            const color = document.getElementById('picker').value.substring(1);
            const status = document.getElementById('status');
            status.className = 'loading';
            status.innerText = '正在同步中...';
            
            fetch('/set_color?rgb=' + color)
                .then(r => {
                    if(r.ok) {
                        status.className = 'success';
                        status.innerText = '完成！当前颜色: #' + color.toUpperCase();
                        setTimeout(() => status.style.display = 'none', 3000);
                    } else { throw new Error(); }
                })
                .catch(e => {
                    status.className = '';
                    status.style.background = '#fee2e2';
                    status.style.color = '#991b1b';
                    status.style.display = 'block';
                    status.innerText = '同步失败，请重试';
                });
        }
    </script>
</body>
</html>
)rawliteral";

/* URI 句柄：返回首页 */
esp_err_t get_handler(httpd_req_t *req) {
    httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* URI 句柄：设置颜色 */
esp_err_t set_color_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "Received request: %s", req->uri);
    char buf[64];
    
    // 获取完整的查询字符串
    size_t query_len = httpd_req_get_url_query_len(req) + 1;
    if (query_len > 1) {
        char *query = (char *)malloc(query_len);
        if (httpd_req_get_url_query_str(req, query, query_len) == ESP_OK) {
            ESP_LOGI(TAG, "Full query received: %s", query);
            if (httpd_query_key_value(query, "rgb", buf, sizeof(buf)) == ESP_OK) {
                int r, g, b;
                if (sscanf(buf, "%02x%02x%02x", &r, &g, &b) == 3) {
                    ESP_LOGI(TAG, "Parsed RGB values -> Red: %d, Green: %d, Blue: %d", r, g, b);
                    esp_err_t res1 = led_strip_set_pixel(led_strip, 0, r, g, b);
                    esp_err_t res2 = led_strip_refresh(led_strip);
                    if (res1 == ESP_OK && res2 == ESP_OK) {
                        ESP_LOGI(TAG, "LED successfully updated!");
                    } else {
                        ESP_LOGE(TAG, "LED update FAILED! (set_pixel: %s, refresh: %s)", 
                                 esp_err_to_name(res1), esp_err_to_name(res2));
                    }
                } else {
                    ESP_LOGE(TAG, "Failed to parse RGB hex values from: %s", buf);
                }
            }
        }
        free(query);
    } else {
        ESP_LOGW(TAG, "No query string found in request");
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

extern "C" void app_main(void) {
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
