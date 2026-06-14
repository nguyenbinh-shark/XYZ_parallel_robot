#pragma once
#include <Arduino.h>

/* =====================================================================
   Module Z Homing & Teach – su dung VL53L0X xac dinh be mat
   =====================================================================
   Yeu cau phần cứng: VL53L0X gap trên dầu công tác, nhìn XUỐNG.

   Quan he hinh hoc:
     Z_HOME_SENSOR_OFFSET_MM = khoang cach vat ly tu VL53L0X xuong dau giac hut
     Khi dau giac hut o do cao h (mm) so voi be mat:
         VL53L0X doc ~ h + Z_HOME_SENSOR_OFFSET_MM
     => Z_vat_ly = VL53L0X_reading - Z_HOME_SENSOR_OFFSET_MM
   ===================================================================== */

/* --- Hinh hoc --- */
#define Z_HOME_SENSOR_OFFSET_MM   8    /* Khoang cach VL53L0X -> dau giac hut (mm) */
#define Z_HOME_NEAR_MM   (Z_HOME_SENSOR_OFFSET_MM + 3)  /* Nguong "cham be mat"    */

/* --- Tham so homing cham --- */
#define Z_HOME_STEP_MM    1.0f    /* Moi buoc ha xuong (mm)                        */
#define Z_HOME_LIFT_MM   20.0f   /* Nang len sau khi homing xong (mm)             */

/* --- Phat hien vat sau khi hut ---
   Sau khi nang len safe_z, neu VL53L0X < nguong nay → co vat               */
#define Z_GRIP_DETECT_MM  (Z_HOME_SENSOR_OFFSET_MM + 15)

/* =====================================================================
   A. Instant homing (BLOCKING ~105ms)
      Doc VL53L0X, tinh Z hien tai, set encoder truc tiep.
      Goi mot lan khi khoi dong (khong can di chuyen co hoc).
      Yeu cau: robot trong tam nhin cua VL53L0X (be mat phia duoi ro rang).
   ===================================================================== */
bool z_home_instant();

/* =====================================================================
   B. Slow homing (NON-BLOCKING)
      Ha Z theo tung buoc Z_HOME_STEP_MM, kiem tra VL53L0X sau moi buoc.
      Khi phat hien be mat: set zero, nang len Z_HOME_LIFT_MM.
      Yeu cau: dat robot gan vi tri CAO NHAT (top of travel) truoc khi goi.
   ===================================================================== */
void z_home_slow_start();
void z_home_update();    /* Goi lien tuc trong loop()              */
bool z_home_busy();      /* true khi dang homing/teach             */

/* =====================================================================
   B'. Homing ve GIOI HAN TREN bang cong tac hanh trinh (GPIO36)
       Chay len cham (raw PWM) den khi cham cong tac, set Z = Z_MAX.
       NON-BLOCKING (dung chung z_home_update / z_home_busy).
       Tin cay hon homing VL53L0X cho moc tham chieu dinh hanh trinh.
   ===================================================================== */
#define Z_HOME_TOP_PWM         110     /* PWM nhe khi chay len tim cong tac */
#define Z_HOME_TOP_TIMEOUT_MS  6000    /* Qua thoi gian nay -> bao loi      */
void z_home_top_start();

/* Huy moi tac vu homing/teach dang chay (dung dong co ngay). */
void z_home_abort();

/* =====================================================================
   C. Teach mode (NON-BLOCKING)
      Ha Z tung buoc cho den khi VL53L0X phat hien be mat (hoac vat the).
      Luu ket qua vao *out_z_mm.
      Dung de day "pick_z" hoac "place_z" chinh xac.
   ===================================================================== */
void z_teach_start(float *out_z_mm);
/* Dung chung z_home_update() va z_home_busy() */

/* =====================================================================
   D. Phat hien vat sau khi hut
      Doc VL53L0X ngay tai thoi diem goi.
      true  = co vat (distance < Z_GRIP_DETECT_MM)
      false = khong co vat hoac cam bien loi
   ===================================================================== */
bool z_grip_has_object();
