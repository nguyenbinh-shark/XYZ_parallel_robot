#include "vl53l0x.h"
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

static Adafruit_VL53L0X _sensor;
static bool _ready = false;

bool vl53l0x_init(uint8_t sda, uint8_t scl, uint32_t freq)
{
    Wire.begin((int)sda, (int)scl, freq);

    if (!_sensor.begin(0x29, false, &Wire)) {
        Serial.println("[VL53L0X] ERR: Khong tim thay cam bien tren I2C");
        return false;
    }

    /* Timing budget 33 ms – can bang giua toc do va do chinh xac */
    _sensor.setMeasurementTimingBudgetMicroSeconds(33000);
    _ready = true;
    Serial.printf("[VL53L0X] OK  SDA=%d SCL=%d\n", sda, scl);
    return true;
}

int16_t vl53l0x_read_mm()
{
    if (!_ready) return -1;

    VL53L0X_RangingMeasurementData_t data;
    _sensor.rangingTest(&data, false);

    /* RangeStatus == 4: out of range / khong co vat the */
    if (data.RangeStatus == 4) return -1;

    return (int16_t)data.RangeMilliMeter;
}

int16_t vl53l0x_read_avg(uint8_t n)
{
    int32_t sum = 0;
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < n; i++) {
        int16_t d = vl53l0x_read_mm();
        if (d > 0) { sum += d; cnt++; }
        if (i < n - 1) delay(35);
    }
    return (cnt > 0) ? (int16_t)(sum / cnt) : -1;
}
