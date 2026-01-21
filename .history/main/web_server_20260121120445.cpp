#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_http_server.h"
#include "web_server.h"

static const char *TAG = "WEB_BACKEND";

// 引用嵌入的二进制文件
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");
extern const uint8_t style_css_start[]  asm("_binary_style_css_start");
extern const uint8_t style_css_end[]    asm("_binary_style_css_end");
extern const uint8_t main_js_start[]    asm("_binary_main_js_start");
extern const uint8_t main_js_end[]      asm("_binary_main_js_end");

/* 处理静态文件请求 */
static esp_err_t server_file_handler(httpd_req_t *req) {
    const char* filename = (const char*)req->user_ctx;
    const uint8_t* start = NULL;
    const uint8_t* end = NULL;
    const char* type = "text/plain";

    if (strcmp(filename, "index") == 0) {
        start = index_html_start; end = index_html_end; type = "text/html; charset=utf-8";
    } else if (strcmp(filename, "css") == 0) {
        start = style_css_start; end = style_css_end; type = "text/css; charset=utf-8";
    } else if (strcmp(filename, "js") == 0) {
        start = main_js_start; end = main_js_end; type = "application/javascript; charset=utf-8";
    }

    if (!start || !end || start >= end) {
        httpd_resp_send_404(req);
        return ESP_OK;
    }

    size_t size = (size_t)(end - start);
    if (size > 0 && start[size-1] == 0x00) size--;

    httpd_resp_set_type(req, type);
    return httpd_resp_send(req, (const char*)start, size);
}

/* API: 控制 LED */
static esp_err_t api_set_color_handler(httpd_req_t *req) {
    char buf[64] = {0};
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char val[32] = {0};
        if (httpd_query_key_value(buf, "rgb", val, sizeof(val)) == ESP_OK) {
            int r, g, b;
            if (sscanf(val, "%02x%02x%02x", &r, &g, &b) == 3) {
                led_strip_set_pixel(led_strip, 0, r, g, b);
                led_strip_refresh(led_strip);
            }
        }
    }
    httpd_resp_send(req, "{\"status\":\"ok\"}", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* API: 控制电机 */
static esp_err_t api_motor_handler(httpd_req_t *req) {
    char buf[128] = {0};
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char dir[32] = {0};
        char item[16] = {0};
        httpd_query_key_value(buf, "dir", dir, sizeof(dir));
        httpd_query_key_value(buf, "id", item, sizeof(item));

        int motor_id = (strlen(item) > 0) ? atoi(item) : 1;
        MotorState next_state = M_STOP;
        if (strcmp(dir, "forward") == 0) next_state = M_FORWARD;
        else if (strcmp(dir, "backward") == 0) next_state = M_BACKWARD;

        if (motor_id == 2) motor2_state = next_state;
        else motor1_state = next_state;
        
        ESP_LOGI(TAG, "Motor %d set to %s", motor_id, dir);
    }
    httpd_resp_send(req, "{\"status\":\"ok\"}", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

void start_webserver() {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.stack_size = 8192;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t index_uri = { .uri = "/", .method = HTTP_GET, .handler = server_file_handler, .user_ctx = (void*)"index" };
        httpd_register_uri_handler(server, &index_uri);
        httpd_uri_t css_uri = { .uri = "/style.css", .method = HTTP_GET, .handler = server_file_handler, .user_ctx = (void*)"css" };
        httpd_register_uri_handler(server, &css_uri);
        httpd_uri_t js_uri = { .uri = "/main.js", .method = HTTP_GET, .handler = server_file_handler, .user_ctx = (void*)"js" };
        httpd_register_uri_handler(server, &js_uri);
        httpd_uri_t color_uri = { .uri = "/api/set_color", .method = HTTP_GET, .handler = api_set_color_handler, .user_ctx = NULL };
        httpd_register_uri_handler(server, &color_uri);
        httpd_uri_t motor_uri = { .uri = "/api/motor", .method = HTTP_GET, .handler = api_motor_handler, .user_ctx = NULL };
        httpd_register_uri_handler(server, &motor_uri);
        ESP_LOGI(TAG, "Web Server Started");
    }
}
