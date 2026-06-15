#pragma once
#include <Arduino.h>

/* =====================================================================
   Module truc Z – GA25-370 DC Geared Motor + Hall Encoder AB + Vit me
   ===================================================================== */

/* --- Chan phan cung (H-bridge: L298N / TB6612FNG) ---
   IN1 / IN2 : dieu khien chieu quay
   PWM (ENA) : dieu khien toc do qua LEDC                              */
#define Z_MOTOR_IN1   4
#define Z_MOTOR_IN2   5
#define Z_MOTOR_PWM   6

/* Encoder Hall sensor 2 kenh AB (ngat CHANGE ca hai kenh – 4x mode)
   Day motor: C1 -> GPIO11 (kenh A),  C2 -> GPIO10 (kenh B).
   Neu Z dem NGUOC chieu (PID chay loan) -> hoan doi 2 chan nay
   HOAC dao IN1/IN2, mot trong hai la du.                              */
#define Z_ENC_A       10   /* C1 */
#define Z_ENC_B       11   /* C2 */

/* --- Cong tac hanh trinh GIOI HAN TREN (end-stop) ---
   Noi GND + dung INPUT_PULLUP -> active-LOW (nhan = muc LOW).
   Cong dung: (1) dung khan khi cham dinh hanh trinh khi dang di LEN,
              (2) homing ve dinh -> chuan hoa Z = Z_MAX.
   (GPIO8 la chan thuong, ho tro INPUT_PULLUP + ngat. Neu doi sang
    GPIO35/36/37 luu y: bi chiem khi module dung Octal PSRAM.)           */
#define Z_LIMIT_TOP            8
#define Z_LIMIT_TOP_ACTIVE_LOW true

/* --- Thong so co khi ---
   PPR    : 11 xung / 1 kenh / 1 vong motor (truoc hop so)
   Gear   : 21.3:1
   Pitch  : buoc vit me (mm / vong truc ra)  – chinh theo vit thuc te  */
#define Z_ENC_PPR       11
#define Z_GEAR_RATIO    21.3f
#define Z_SCREW_PITCH   2.0f    /* vit me lead 8mm/vong */

/* Xung/mm trong che do 4x quadrature:
   PPR * 4 * Gear / Pitch  =  11 * 4 * 21.3 / 8.0  ~= 117.15 xung/mm  */
#define Z_PULSE_PER_MM  ((Z_ENC_PPR * 4.0f * Z_GEAR_RATIO) / Z_SCREW_PITCH)

/* --- Gioi han hanh trinh (mm) – chinh theo chieu dai vit me thuc te --- */
#define Z_MIN_MM    -100.0f
#define Z_MAX_MM   100.0f

/* --- Bo dieu khien PD --- */
#define Z_KP              2.5f   /* He so ty le                              */
#define Z_KD              0.08f  /* He so dao ham – giam dao dong khi den dich*/
#define Z_DEADBAND        4      /* Vung chet +/- xung -> dung dong co        */
#define Z_PWM_MIN         55     /* PWM toi thieu de thang ma sat tinh        */
#define Z_PWM_MAX         220    /* PWM toi da (bao ve dong, ~80% full duty)  */
#define Z_PID_INTERVAL_MS 20     /* Chu ky cap nhat PID (ms)                  */

/* PWM motor: LEDC channel rieng, 20 kHz (khong nghe duoc, giam tieng on) */
#define Z_LEDC_CH    4
#define Z_LEDC_FREQ  20000
#define Z_LEDC_RES   8          /* 8-bit: duty 0..255                        */

/* ========================= API ======================================= */

/* Khoi tao chan, encoder, LEDC. Goi trong setup(). */
void  z_axis_init();

/* Di chuyen den vi tri tuyet doi z_mm (mm tinh tu zero).
   Tra ve false neu z_mm ngoai [Z_MIN_MM, Z_MAX_MM]. */
bool  z_axis_move_to(float z_mm);

/* Di chuyen tuong doi +/- delta_mm tu vi tri hien tai. */
bool  z_axis_move_by(float delta_mm);

/* Dung dong co ngay lap tuc (co phanh). */
void  z_axis_stop();

/* Lay vi tri hien tai (mm). */
float z_axis_get_pos_mm();

/* true neu dang trong qua trinh di chuyen den dich. */
bool  z_axis_is_moving();

/* Dat vi tri hien tai lam goc zero (=0 mm). */
void  z_axis_set_zero();

/* Cap nhat PID – phai duoc goi lien tuc trong loop(). */
void  z_axis_update();

/* Force-set vi tri (mm) ma khong di chuyen co hoc.
   Chi dung cho homing – set encoder pos truc tiep.           */
void  z_axis_set_pos_mm(float z_mm);

/* Dieu khien PWM truc tiep, bo qua PID. pwm: -Z_PWM_MAX..+Z_PWM_MAX.
   0 = phanh. Goi z_axis_stop() de quay lai che do binh thuong. */
void    z_axis_raw_pwm(int16_t pwm);

/* Lay gia tri encoder thuan (xung). Dung cho diagnostics / do toc do. */
int32_t z_axis_get_enc_cnt();

/* Doc cong tac gioi han tren. true = dang bi nhan (cham dinh hanh trinh). */
bool    z_limit_top_triggered();
