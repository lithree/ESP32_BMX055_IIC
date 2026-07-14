#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

// Service and Characteristic UUIDs matching the Android App
// SERVICE_UUID: 0000ffe0-0000-1000-8000-00805f9b34fb
// CHARACTERISTIC_UUID: 0000ffe1-0000-1000-8000-00805f9b34fb

#define BLE_SERVICE_UUID 0xFFE0
#define BLE_CHARACTERISTIC_UUID 0xFFE1

typedef struct
{
    uint8_t turn_direction;
    uint8_t lane_index;
    uint8_t total_lanes;
    uint16_t distance_to_turn;
} nav_data_t;

/**
 * @brief Callback function type for received navigation data
 */
typedef void (*ble_nav_data_callback_t)(const nav_data_t *nav_data);

/**
 * @brief Initialize BLE stack and start advertising as "ESP32_NAV"
 * @param callback Callback to execute when valid navigation data is received
 */
void BLE_Manager_Init(ble_nav_data_callback_t callback);

/**
 * @brief Stop BLE advertising and release resources if necessary
 */
void BLE_Manager_Deinit(void);

#endif // BLE_MANAGER_H
