#include "BLEManager.h"
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"

/* BLE Includes */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "BLEManager";
static ble_nav_data_callback_t g_nav_callback = NULL;
static uint16_t g_val_handle;
static uint8_t ble_addr_type;

static int ble_gatt_svr_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

// Define UUIDs
static const ble_uuid16_t gatt_svr_svc_uuid = BLE_UUID16_INIT(BLE_SERVICE_UUID);
static const ble_uuid16_t gatt_svr_chr_uuid = BLE_UUID16_INIT(BLE_CHARACTERISTIC_UUID);

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = &gatt_svr_chr_uuid.u,
                .access_cb = ble_gatt_svr_cb,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .val_handle = &g_val_handle,
            },
            {
                0, // No more characteristics
            }},
    },
    {
        0, // No more services
    },
};

static uint8_t compute_checksum(const uint8_t *data, int len, int start_index)
{
    uint8_t xor_val = 0;
    for (int i = start_index; i < len; ++i)
    {
        xor_val ^= data[i];
    }
    return xor_val;
}

static void parse_and_callback_nav_data(const uint8_t *data, int len)
{
    // Minimum packet size: [0] AA, [1] Cmd, [2] Turn, [3] Lane, [4] Total Lanes, [5] Dist High, [6] Dist Low, [7] Checksum
    if (len < 8)
    {
        ESP_LOGW(TAG, "Received packet too short: %d bytes", len);
        return;
    }

    if (data[0] != 0xAA)
    {
        ESP_LOGW(TAG, "Header byte incorrect: 0x%02X", data[0]);
        return;
    }

    if (data[1] != 0x01)
    {
        ESP_LOGW(TAG, "Unknown command type: 0x%02X", data[1]);
        return;
    }

    uint8_t rx_checksum = data[7];
    uint8_t calc_checksum = compute_checksum(data, 7, 1);
    if (rx_checksum != calc_checksum)
    {
        ESP_LOGE(TAG, "Checksum mismatch! Rx: 0x%02X, Calc: 0x%02X", rx_checksum, calc_checksum);
        return;
    }

    nav_data_t nav_data;
    nav_data.turn_direction = data[2];
    nav_data.lane_index = data[3];
    nav_data.total_lanes = data[4];
    nav_data.distance_to_turn = ((uint16_t)data[5] << 8) | data[6];

    ESP_LOGI(TAG, "Parsed Nav Data: Turn=%d, Lane=%d/%d, Distance=%d m",
             nav_data.turn_direction, nav_data.lane_index, nav_data.total_lanes, nav_data.distance_to_turn);

    if (g_nav_callback != NULL)
    {
        g_nav_callback(&nav_data);
    }
}

static int ble_gatt_svr_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR)
    {
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        uint8_t buf[32];
        if (len > sizeof(buf))
        {
            ESP_LOGW(TAG, "Received message too long: %d bytes", len);
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }

        int rc = ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &len);
        if (rc != 0)
        {
            return BLE_ATT_ERR_UNLIKELY;
        }

        parse_and_callback_nav_data(buf, len);
        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

static void ble_app_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    const char *name = "ESP32_NAV";
    int rc;

    memset(&fields, 0, sizeof(fields));

    // Discoverability flags
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    // Transmit Power
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    // Device Name
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    // Advertised Services (16-bit UUID)
    fields.uuids16 = (ble_uuid16_t[]){
        BLE_UUID16_INIT(BLE_SERVICE_UUID)};
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Error setting advertisement data; rc=%d", rc);
        return;
    }

    // Set advertisement parameters
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, NULL, NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Error starting advertisement; rc=%d", rc);
    }
    else
    {
        ESP_LOGI(TAG, "BLE advertising started successfully");
    }
}

static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "BLE Connection %s; status=%d",
                 event->connect.status == 0 ? "established" : "failed",
                 event->connect.status);
        if (event->connect.status != 0)
        {
            ble_app_advertise();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE Disconnected; reason=%d", event->disconnect.reason);
        ble_app_advertise();
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "BLE Advertising complete; reason=%d", event->adv_complete.reason);
        ble_app_advertise();
        break;
    }
    return 0;
}

static void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void ble_on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &ble_addr_type);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Error determining address type; rc=%d", rc);
        return;
    }

    ble_app_advertise();
}

void BLE_Manager_Init(ble_nav_data_callback_t callback)
{
    g_nav_callback = callback;

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    int rc = nimble_port_init();
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to initialize NimBLE; rc=%d", rc);
        return;
    }

    // Configure NimBLE
    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_hs_cfg.gatts_register_cb = NULL;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    // Register GATT services
    ble_svc_gap_init();
    ble_svc_gatt_init();
    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Error counting GATT services; rc=%d", rc);
        return;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Error adding GATT services; rc=%d", rc);
        return;
    }

    // Set Device Name
    rc = ble_svc_gap_device_name_set("ESP32_NAV");
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Error setting GAP device name; rc=%d", rc);
        return;
    }

    nimble_port_freertos_init(ble_host_task);
}

void BLE_Manager_Deinit(void)
{
    nimble_port_stop();
    g_nav_callback = NULL;
    ESP_LOGI(TAG, "BLE Manager Deinitialized");
}
