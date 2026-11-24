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

/* =====================================================================
   BO LOC – chong nhieu / nhay gia tri cua VL53L0X
   ===================================================================== */

#define VL_MEDIAN_MAX   9       /* So mau toi da cho 1 lan median burst   */
#define VL_EMA_ALPHA    0.40f   /* He so EMA: cao=nhanh nhay, thap=muot hon*/

/* Sap xep chen (insertion sort) tai cho roi tra ve phan tu trung vi. */
static int16_t median_of(int16_t *a, uint8_t n)
{
    for (uint8_t i = 1; i < n; i++) {
        int16_t key = a[i];
        int8_t  j   = (int8_t)i - 1;
        while (j >= 0 && a[j] > key) { a[j + 1] = a[j]; j--; }
        a[j + 1] = key;
    }
    return a[n / 2];
}

int16_t vl53l0x_read_median(uint8_t n)
{
    if (!_ready) return -1;
    if (n < 1) n = 1;
    if (n > VL_MEDIAN_MAX) n = VL_MEDIAN_MAX;

    int16_t buf[VL_MEDIAN_MAX];
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < n; i++) {
        int16_t d = vl53l0x_read_mm();   /* single-shot ~33ms, da blocking */
        if (d >= 0) buf[cnt++] = d;
    }
    if (cnt == 0) return -1;
    return median_of(buf, cnt);
}

/* --- Trang thai cho vl53l0x_read_filtered() --- */
static int16_t _med3[3] = { -1, -1, -1 };  /* cua so 3 mau (vong tron)     */
static uint8_t _med3_n  = 0;               /* so mau hop le trong cua so   */
static uint8_t _med3_i  = 0;               /* vi tri ghi tiep theo          */
static float   _ema     = -1.0f;           /* gia tri EMA (-1 = chua co)    */

void vl53l0x_filter_reset()
{
    _med3[0] = _med3[1] = _med3[2] = -1;
    _med3_n = 0;
    _med3_i = 0;
    _ema    = -1.0f;
}

int16_t vl53l0x_read_filtered()
{
    int16_t d = vl53l0x_read_mm();
    if (d < 0) {
        /* Mat tin hieu tam thoi: giu gia tri da loc, khong cap nhat */
        return (_ema >= 0.0f) ? (int16_t)(_ema + 0.5f) : -1;
    }

    /* (1) Median 3 mau gan nhat -> loai gai nhieu xung don le */
    _med3[_med3_i] = d;
    _med3_i = (_med3_i + 1) % 3;
    if (_med3_n < 3) _med3_n++;

    int16_t tmp[3];
    for (uint8_t k = 0; k < _med3_n; k++) tmp[k] = _med3[k];
    int16_t med = median_of(tmp, _med3_n);

    /* (2) EMA -> lam muot dao dong con lai */
    if (_ema < 0.0f) _ema = (float)med;
    else             _ema = VL_EMA_ALPHA * (float)med + (1.0f - VL_EMA_ALPHA) * _ema;

    return (int16_t)(_ema + 0.5f);
}
