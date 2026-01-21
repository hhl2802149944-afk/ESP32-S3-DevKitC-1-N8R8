#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "ble_server.h"
#include "shared_state.h"

static const char *TAG = "BLE_CONTROL";
static uint8_t ble_addr_type;
void ble_app_advertise(void);

// UUIDs Generated for this device
// Service: 1212
// Characteristic: 3434
static const ble_uuid128_t gatt_svr_svc_uuid = 
    BLE_UUID128_INIT(0xef, 0xbe, 0xad, 0xde, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x12, 0x00, 0x00);

static const ble_uuid128_t gatt_svr_chr_uuid = 
    BLE_UUID128_INIT(0xef, 0xbe, 0xad, 0xde, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x34, 0x34, 0x00, 0x00);

static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg);

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &gatt_svr_chr_uuid.u,
                .access_cb = gatt_svr_chr_access,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {0}
        },
    },
    {0}
};

static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        char buf[32] = {0};
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len > sizeof(buf) - 1) len = sizeof(buf) - 1;
        
        ble_hs_mbuf_to_flat(ctxt->om, buf, len, NULL);
        ESP_LOGI(TAG, "Received BLE Command: %s", buf);

        // Command Parsing
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
