#pragma once

#include "device.h"

#if defined(PLATFORM_ESP32)
#define BLE_SERVICE_ELRS_RC "E735"        // Bluetooth LE service ID for tank level
#define BLE_SERVICE_ELRS_REMOTE "E736"        // Bluetooth LE service ID for tank level
#define BLE_CHARACTERISTIC_RC_BAT "00002a19-0000-1000-8000-00805f9b34fb" // Bluetooth LE characteristic ID for RC battery
extern device_t BLE_telemetry;

#define BLE_UPDATE_INTERVAL    1000U
#define HAS_BLE_TELEMETRY
#endif