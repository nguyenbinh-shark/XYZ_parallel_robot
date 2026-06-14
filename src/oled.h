#pragma once
#include <Arduino.h>

/* SSD1306 0.96" 128x64 – dung chung I2C voi VL53L0X (SDA=40, SCL=41) */
#define OLED_I2C_ADDR  0x3C
#define OLED_WIDTH     128
#define OLED_HEIGHT    64

/* Khoi tao man hinh. Goi sau vl53l0x_init() de dung chung Wire. */
void oled_init();

/*
 * Cap nhat toan bo man hinh.
 * Goi trong loop() moi ~200ms.
 *   x, y      : vi tri dau cuoi 5-bar (mm)
 *   z         : vi tri truc Z (mm)
 *   sucking   : trang thai giac hut
 *   pp_state  : ten state PP (chuoi ngan, vi du "IDLE", "PICK_XY")
 *   dist_mm   : gia tri VL53L0X (<0 = loi / ngoai tam do)
 */
void oled_update(float x, float y, float z,
                 bool sucking, const char *pp_state, int16_t dist_mm);
