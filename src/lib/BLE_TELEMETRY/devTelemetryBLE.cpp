#include "telemetryBLE.h"

#if defined(PLATFORM_ESP32)

#include "CRSF.h"
#include "NimBLEAdvertising.h"
#include "NimBLEDevice.h"
#include "POWERMGNT.h"
#include "common.h"
#include "hwTimer.h"

NimBLEServer *pServer = NULL;
NimBLECharacteristic *pRCVBat;

const CrossfireSensor crossfireSensors[] = {
    {LINK_ID, 0, STR_SENSOR_RX_RSSI1, UNIT_DB, 0},
    {LINK_ID, 1, STR_SENSOR_RX_RSSI2, UNIT_DB, 0},
    {LINK_ID, 2, STR_SENSOR_RX_QUALITY, UNIT_PERCENT, 0},
    {LINK_ID, 3, STR_SENSOR_RX_SNR, UNIT_DB, 0},
    {LINK_ID, 4, STR_SENSOR_ANTENNA, UNIT_RAW, 0},
    {LINK_ID, 5, STR_SENSOR_RF_MODE, UNIT_RAW, 0},
    {LINK_ID, 6, STR_SENSOR_TX_POWER, UNIT_MILLIWATTS, 0},
    {LINK_ID, 7, STR_SENSOR_TX_RSSI, UNIT_DB, 0},
    {LINK_ID, 8, STR_SENSOR_TX_QUALITY, UNIT_PERCENT, 0},
    {LINK_ID, 9, STR_SENSOR_TX_SNR, UNIT_DB, 0},
    {LINK_RX_ID, 0, STR_SENSOR_RX_RSSI_PERC, UNIT_PERCENT, 0},
    {LINK_RX_ID, 1, STR_SENSOR_RX_RF_POWER, UNIT_DBM, 0},
    {LINK_TX_ID, 0, STR_SENSOR_TX_RSSI_PERC, UNIT_PERCENT, 0},
    {LINK_TX_ID, 1, STR_SENSOR_TX_RF_POWER, UNIT_DBM, 0},
    {LINK_TX_ID, 2, STR_SENSOR_TX_FPS, UNIT_HERTZ, 0},
    {BATTERY_ID, 0, STR_SENSOR_BATT, UNIT_VOLTS, 1},
    {BATTERY_ID, 1, STR_SENSOR_CURR, UNIT_AMPS, 1},
    {BATTERY_ID, 2, STR_SENSOR_CAPACITY, UNIT_MAH, 0},
    {BATTERY_ID, 3, STR_BATT_PERCENT, UNIT_PERCENT, 0},
    {GPS_ID, 0, STR_SENSOR_GPS, UNIT_GPS_LATITUDE, 0},
    {GPS_ID, 0, STR_SENSOR_GPS, UNIT_GPS_LONGITUDE, 0},
    {GPS_ID, 2, STR_SENSOR_GSPD, UNIT_KMH, 1},
    {GPS_ID, 3, STR_SENSOR_HDG, UNIT_DEGREE, 3},
    {GPS_ID, 4, STR_SENSOR_ALT, UNIT_METERS, 0},
    {GPS_ID, 5, STR_SENSOR_SATELLITES, UNIT_RAW, 0},
    {ATTITUDE_ID, 0, STR_SENSOR_PITCH, UNIT_RADIANS, 3},
    {ATTITUDE_ID, 1, STR_SENSOR_ROLL, UNIT_RADIANS, 3},
    {ATTITUDE_ID, 2, STR_SENSOR_YAW, UNIT_RADIANS, 3},
    {FLIGHT_MODE_ID, 0, STR_SENSOR_FLIGHT_MODE, UNIT_TEXT, 0},
    {CF_VARIO_ID, 0, STR_SENSOR_VSPD, UNIT_METERS_PER_SECOND, 2},
    {BARO_ALT_ID, 0, STR_SENSOR_ALT, UNIT_METERS, 2},
    {0, 0, "UNKNOWN", UNIT_RAW, 0},
};

const CrossfireSensor &getCrossfireSensor(uint8_t id, uint8_t subId)
{
    if (id == LINK_ID)
        return crossfireSensors[RX_RSSI1_INDEX + subId];
    else if (id == LINK_RX_ID)
        return crossfireSensors[RX_RSSI_PERC_INDEX + subId];
    else if (id == LINK_TX_ID)
        return crossfireSensors[TX_RSSI_PERC_INDEX + subId];
    else if (id == BATTERY_ID)
        return crossfireSensors[BATT_VOLTAGE_INDEX + subId];
    else if (id == GPS_ID)
        return crossfireSensors[GPS_LATITUDE_INDEX + subId];
    else if (id == CF_VARIO_ID)
        return crossfireSensors[VERTICAL_SPEED_INDEX];
    else if (id == ATTITUDE_ID)
        return crossfireSensors[ATTITUDE_PITCH_INDEX + subId];
    else if (id == FLIGHT_MODE_ID)
        return crossfireSensors[FLIGHT_MODE_INDEX];
    else if (id == BARO_ALT_ID)
        return crossfireSensors[BARO_ALTITUDE_INDEX];
    else
        return crossfireSensors[UNKNOWN_INDEX];
}

void processCrossfireTelemetryValue(uint8_t index, int32_t value)
{
    if (!TELEMETRY_STREAMING())
        return;

    const CrossfireSensor &sensor = crossfireSensors[index];
    setTelemetryValue(PROTOCOL_TELEMETRY_CROSSFIRE, sensor.id, 0, sensor.subId,
                      value, sensor.unit, sensor.precision);
}
void BluetoothTelemetryUpdateValues()
{
    if (CRSF::CRSFstate)
    {
    }
}

void stopBleServer()
{
    NimBLEDevice::deinit();
}

void createBleServer(String hostname)
{
    DBGLN(F("[BLE] Initializing the Bluetooth low energy (BLE) stack"));
    NimBLEDevice::init(hostname.c_str());
    NimBLEDevice::setPower(ESP_PWR_LVL_N12, ESP_BLE_PWR_TYPE_ADV);
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
    NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_PUBLIC);
    pServer = NimBLEDevice::createServer();

    NimBLEService *pEnvService = pServer->createService(BLE_SERVICE_ELRS_RC);
    pRCVBat = pEnvService->createCharacteristic(
        BLE_CHARACTERISTIC_RC_BAT,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::BROADCAST | NIMBLE_PROPERTY::NOTIFY);
    NimBLE2904 *p2904 = (NimBLE2904 *)pRCVBat->createDescriptor("2904");
    p2904->setFormat(NimBLE2904::FORMAT_UINT8);
    p2904->setUnit(NimBLE2904::FORMAT_UINT8);
    pRCVBat->setValue(0);

    pEnvService->start();

    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    DBGLN(F("[BLE] Begin Advertising of "));
    DBGLN(pEnvService->getUUID().toString().c_str());
    pAdvertising->addServiceUUID(pEnvService->getUUID());
    NimBLEDevice::startAdvertising();
    DBGLN(F("[BLE] Advertising Started"));
}

template <int N>
bool getCrossfireTelemetryValue(uint8_t index, int32_t &value, uint8_t module)
{
    uint8_t *rxBuffer = getTelemetryRxBuffer(module);
    bool result = false;
    uint8_t *byte = &rxBuffer[index];
    value = (*byte & 0x80) ? -1 : 0;
    for (uint8_t i = 0; i < N; i++)
    {
        value <<= 8;
        if (*byte != 0xff)
        {
            result = true;
        }
        value += *byte++;
    }
    return result;
}

void processCrossfireTelemetryFrame(uint8_t *rxBuffer)
{

    uint8_t id = rxBuffer[2];
    int32_t value;
    switch (id)
    {
    case CF_VARIO_ID:
        if (getCrossfireTelemetryValue<2>(3, value, module))
            processCrossfireTelemetryValue(VERTICAL_SPEED_INDEX, value);
        break;

    case GPS_ID:
        if (getCrossfireTelemetryValue<4>(3, value, module))
            processCrossfireTelemetryValue(GPS_LATITUDE_INDEX, value / 10);
        if (getCrossfireTelemetryValue<4>(7, value, module))
            processCrossfireTelemetryValue(GPS_LONGITUDE_INDEX, value / 10);
        if (getCrossfireTelemetryValue<2>(11, value, module))
            processCrossfireTelemetryValue(GPS_GROUND_SPEED_INDEX, value);
        if (getCrossfireTelemetryValue<2>(13, value, module))
            processCrossfireTelemetryValue(GPS_HEADING_INDEX, value);
        if (getCrossfireTelemetryValue<2>(15, value, module))
            processCrossfireTelemetryValue(GPS_ALTITUDE_INDEX, value - 1000);
        if (getCrossfireTelemetryValue<1>(17, value, module))
            processCrossfireTelemetryValue(GPS_SATELLITES_INDEX, value);
        break;

    case BARO_ALT_ID:
        if (getCrossfireTelemetryValue<2>(3, value, module))
        {
            if (value & 0x8000)
            {
                // Altitude in meters
                value &= ~(0x8000);
                value *= 100; // cm
            }
            else
            {
                // Altitude in decimeters + 10000dm
                value -= 10000;
                value *= 10;
            }
            processCrossfireTelemetryValue(BARO_ALTITUDE_INDEX, value);
        }
        break;

    case LINK_ID:
        for (unsigned int i = 0; i <= TX_SNR_INDEX; i++)
        {
            if (getCrossfireTelemetryValue<1>(3 + i, value, module))
            {
                if (i == TX_POWER_INDEX)
                {
                    static const int32_t power_values[] = {0, 10, 25, 100, 500,
                                                           1000, 2000, 250, 50};
                    value =
                        ((unsigned)value < DIM(power_values) ? power_values[value] : 0);
                }
                processCrossfireTelemetryValue(i, value);
                if (i == RX_QUALITY_INDEX)
                {
                    if (value)
                    {
                        telemetryData.rssi.set(value);
                        telemetryStreaming = TELEMETRY_TIMEOUT10ms;
                        telemetryData.telemetryValid |= 1 << module;
                    }
                    else
                    {
                        if (telemetryData.telemetryValid & (1 << module))
                        {
                            telemetryData.rssi.reset();
                            telemetryStreaming = 0;
                        }
                        telemetryData.telemetryValid &= ~(1 << module);
                    }
                }
            }
        }
        break;

    case LINK_RX_ID:
        if (getCrossfireTelemetryValue<1>(4, value, module))
            processCrossfireTelemetryValue(RX_RSSI_PERC_INDEX, value);
        if (getCrossfireTelemetryValue<1>(7, value, module))
            processCrossfireTelemetryValue(TX_RF_POWER_INDEX, value);
        break;

    case LINK_TX_ID:
        if (getCrossfireTelemetryValue<1>(4, value, module))
            processCrossfireTelemetryValue(TX_RSSI_PERC_INDEX, value);
        if (getCrossfireTelemetryValue<1>(7, value, module))
            processCrossfireTelemetryValue(RX_RF_POWER_INDEX, value);
        if (getCrossfireTelemetryValue<1>(8, value, module))
            processCrossfireTelemetryValue(TX_FPS_INDEX, value * 10);
        break;

    case BATTERY_ID:
        if (getCrossfireTelemetryValue<2>(3, value, module))
            processCrossfireTelemetryValue(BATT_VOLTAGE_INDEX, value);
        if (getCrossfireTelemetryValue<2>(5, value, module))
            processCrossfireTelemetryValue(BATT_CURRENT_INDEX, value);
        if (getCrossfireTelemetryValue<3>(7, value, module))
            processCrossfireTelemetryValue(BATT_CAPACITY_INDEX, value);
        if (getCrossfireTelemetryValue<1>(10, value, module))
            processCrossfireTelemetryValue(BATT_REMAINING_INDEX, value);
        break;

    case ATTITUDE_ID:
        if (getCrossfireTelemetryValue<2>(3, value, module))
            processCrossfireTelemetryValue(ATTITUDE_PITCH_INDEX, value / 10);
        if (getCrossfireTelemetryValue<2>(5, value, module))
            processCrossfireTelemetryValue(ATTITUDE_ROLL_INDEX, value / 10);
        if (getCrossfireTelemetryValue<2>(7, value, module))
            processCrossfireTelemetryValue(ATTITUDE_YAW_INDEX, value / 10);
        break;

    case FLIGHT_MODE_ID: {
        const CrossfireSensor &sensor = crossfireSensors[FLIGHT_MODE_INDEX];
        auto textLength = min<int>(16, rxBuffer[1]);
        rxBuffer[textLength] = '\0';
        setTelemetryText(PROTOCOL_TELEMETRY_CROSSFIRE, sensor.id, 0, sensor.subId,
                         (const char *)rxBuffer + 3);
        break;
    }

    case RADIO_ID:
        if (rxBuffer[3] == 0xEA    // radio address
            && rxBuffer[5] == 0x10 // timing correction frame
        )
        {
            uint32_t update_interval;
            int32_t offset;
            if (getCrossfireTelemetryValue<4>(6, (int32_t &)update_interval,
                                              module) &&
                getCrossfireTelemetryValue<4>(10, offset, module))
            {
                // values are in 10th of micro-seconds
                update_interval /= 10;
                offset /= 10;

                TRACE("[XF] Rate: %d, Lag: %d", update_interval, offset);
                getModuleSyncStatus(module).update(update_interval, offset);
            }
        }
        break;
    }
}

void updateTelemetryCharacteristic(int val)
{
    if (pServer->getConnectedCount())
    {

        pRCVBat->setValue(val);
        pRCVBat->notify(true);
    }
}

static int start()
{
    createBleServer("ELRS Telemetry");
    return DURATION_IMMEDIATELY;
}

static int timeout()
{
    updateTelemetryCharacteristic(12);
    return BLE_UPDATE_INTERVAL;
}

device_t BLE_telemetry = {
    .initialize = NULL,
    .start = start,
    .event = NULL,
    .timeout = timeout};

#endif