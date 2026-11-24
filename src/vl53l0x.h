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
 * Luu y: trung binh NHAY voi gai nhieu (spike) – uu tien dung read_median().
 */
int16_t vl53l0x_read_avg(uint8_t n = 3);

/*
 * MEDIAN cua n lan do nhanh (mac dinh 5) – LOAI BO gai nhieu xung (spike).
 * Tot nhat cho do tai vi tri CO DINH (homing, teach, do tim vat khi nhat,
 * kiem tra giac hut) vi trung vi bo qua cac gia tri nhay bat thuong.
 * BLOCKING ~ n * 33ms. Tra ve -1 neu khong co lan do nao hop le.
 */
int16_t vl53l0x_read_median(uint8_t n = 5);

/*
 * Doc co LOC TRANG THAI (goi LAP LAI, vi du OLED / giam sat lien tuc):
 *   median 3 mau gan nhat (loai spike) -> trung binh truot EMA (lam muot).
 * Moi lan goi chi 1 phep do (~33ms). Tra ve mm da loc, hoac -1 khi chua co.
 * Mat tin hieu (-1) tam thoi -> giu gia tri da loc truoc do.
 */
int16_t vl53l0x_read_filtered();

/*
 * Xoa trang thai bo loc (median window + EMA). Goi khi chuyen ngu canh de
 * tranh dung gia tri cu (vi du sau khi ket thuc mot chu trinh van chuyen).
 */
void    vl53l0x_filter_reset();
