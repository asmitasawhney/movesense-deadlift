#pragma once

#include <stdint.h>

#define DEFAULT_FAST_ADVERTISING_TIME 60        // 60 seconds
#define DEFAULT_SLOW_ADVERTISING_TIME 0         // 0 = Infinite
#define DEFAULT_FAST_ADVERTISING_INTERVAL 40    // * 0.625ms
#define DEFAULT_SLOW_ADVERTISING_INTERVAL 160   // * 0.625ms


class BleController
{
friend class BleGapService;
friend class BleGattService;

public:
    static BleController* spInstance;
    BleController();

    // Builder for Advertising and Scan Response packets
    static uint32_t AddChunkToBuffer(uint8_t * buffer, uint8_t code,
        const uint8_t * data, uint8_t size);
    const char * GetDeviceIdentifier();
    uint32_t GetCPUIdentifier();

protected:
    virtual void onBleInit() {};
    virtual uint32_t PrepareAdvertisingData(uint8_t * buffer, uint32_t max_len);
    virtual uint32_t PrepareScanResponseData(uint8_t * buffer, uint32_t max_len);
    virtual void OnHrsNotificationEnabled() {}
    virtual void OnHrsNotificationDisabled() {}
    virtual void OnUartData(uint8_t * p_data, uint16_t length) {}
};

void send_ble_hr_measurement(uint16_t heart_rate);
void send_ble_nus_data(uint8_t * p_data, uint16_t length);
