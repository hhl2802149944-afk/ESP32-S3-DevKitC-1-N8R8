#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "ble_server.h"
#include "shared_state.h"

extern "C" {

static const char *TAG = "BLE_CONTROL";
static uint8_t ble_addr_type;
void ble_app_advertise(void);

// UUIDs updated to match iOS App
// Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
static const ble_uuid128_t gatt_svr_svc_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E);

// Characteristic: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E
static const ble_uuid128_t gatt_svr_chr_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E);

static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg);

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = (ble_uuid_t *)&gatt_svr_svc_uuid,
        .includes = NULL,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = (ble_uuid_t *)&gatt_svr_chr_uuid,
                .access_cb = gatt_svr_chr_access,
                .arg = NULL,
                .descriptors = NULL,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .min_key_size = 0,
                .val_handle = NULL,
                .cpfd = NULL,
            },
            {
                .uuid = NULL,
            }
        },
    },
    {
        .type = 0,
    }
};

static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        
        // --- 1. 处理 App 二进制协议 (9字节: 4B Lat, 4B Lon, 1B Checksum) ---
        if (len == 9) {
            uint8_t pkt[9];
            ble_hs_mbuf_to_flat(ctxt->om, pkt, 9, NULL);
            
            uint8_t sum = 0;
            for (int i = 0; i < 8; i++) sum += pkt[i];
            
            if (sum == pkt[8]) {
                int32_t lat_raw, lon_raw;
                memcpy(&lat_raw, pkt, 4);
                memcpy(&lon_raw, pkt + 4, 4);
                
                float lat = lat_raw / 1000000.0f;
                float lon = lon_raw / 1000000.0f;
                
                ESP_LOGI(TAG, "App Coordinate: Lat=%.6f, Lon=%.6f", lat, lon);
                // 这里可以后续对接导航逻辑
                return 0;
            }
        }

        // --- 2. 处理文本指令 (兼容 Web UI) ---
        char buf[32] = {0};
        if (len > sizeof(buf) - 1) len = sizeof(buf) - 1;
        ble_hs_mbuf_to_flat(ctxt->om, buf, len, NULL);
        ESP_LOGI(TAG, "Received BLE Command: %s", buf);

        if (strcmp(buf, "m1f") == 0) motor1_state = M_FORWARD;
        else if (strcmp(buf, "m1b") == 0) motor1_state = M_BACKWARD;
        else if (strcmp(buf, "m1s") == 0) motor1_state = M_STOP;
        else if (strcmp(buf, "m2f") == 0) motor2_state = M_FORWARD;
        else if (strcmp(buf, "m2b") == 0) motor2_state = M_BACKWARD;
        else if (strcmp(buf, "m2s") == 0) motor2_state = M_STOP;
        else if (strncmp(buf, "rgb", 3) == 0 && strlen(buf) >= 9) {
            int r, g, b;
            if (sscanf(buf + 3, "%02x%02x%02x", &r, &g, &b) == 3) {
                led_strip_set_pixel(led_strip, 0, r, g, b);
                led_strip_refresh(led_strip);
            }
        }
        else if (strncmp(buf, "spd", 3) == 0) {
            int val = atoi(buf + 3);
            if (val < 10) val = 10;
            if (val > 100) val = 100;
            motor_step_delay = val;
            
            float ratio = (float)(val - 10) / 90.0f;
            int r = (int)(255 * (1.0f - ratio));
            int g = (int)(255 * ratio);
            led_strip_set_pixel(led_strip, 0, r, g, 0);
            led_strip_refresh(led_strip);
        }
        else if (strcmp(buf, "off") == 0) {
            ESP_LOGI(TAG, "Entering Deep Sleep...");
            led_strip_clear(led_strip);
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(500));
            // 使用 ext0 唤醒：GPIO 0 (BOOT 按键), 低电平(0)唤醒
            esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);
            esp_deep_sleep_start();
        }
    }
    return 0;
}

void ble_app_on_sync(void) {
    ble_hs_id_infer_auto(0, &ble_addr_type);
    ble_app_advertise();
}

void ble_app_advertise(void) {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    const char *name;

    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    ble_gap_adv_set_fields(&fields);

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, NULL, NULL);
}

void device_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void start_ble_server(void) {
    nimble_port_init();
    ble_svc_gap_device_name_set("ESP32S3-Motor-Control");
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svr_svcs);
    ble_gatts_add_svcs(gatt_svr_svcs);
    
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    nimble_port_freertos_init(device_host_task);
}
}
