#pragma once
#include <Arduino.h>

/*
 * Module doc cam bien khoang cach VL53L0X (ToF I2C).
 * I2C dung cho project: SDA=GPIO40, SCL=GPIO41 (dung chung voi OLED).
 */

#define VL53L0X_SDA_PIN     40
#define VL53L0X_SCL_PIN     41
#define VL53L0X_I2C_FREQ    400000UL   /* 400 kHz Fast Mode */

/*
 * Khoi tao cam bien. Goi mot lan trong setup().
 * Tra ve true neu phat hien cam bien tren bus I2C.
 */
bool vl53l0x_init(uint8_t sda = VL53L0X_SDA_PIN,
                  uint8_t scl = VL53L0X_SCL_PIN,
                  uint32_t freq = VL53L0X_I2C_FREQ);

/*
 * Doc khoang cach (mm) single-shot.
 * Tra ve gia tri >= 0 neu hop le, -1 neu ngoai tam do hoac loi.
 */
int16_t vl53l0x_read_mm();

/*
 * Lay trung binh n lan do (delay 35ms giua moi lan).
 * Dung cho homing / teach can ket qua on dinh.
 * BLOCKING: n * 35ms tong cong.
 */
int16_t vl53l0x_read_avg(uint8_t n = 3);
