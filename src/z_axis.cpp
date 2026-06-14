#include "z_axis.h"
#include <math.h>

/* ---------------------------------------------------------------
   Bien encoder – volatile, chia se giua ISR va loop
   --------------------------------------------------------------- */
static volatile int32_t enc_pos  = 0;
static volatile int8_t  last_ab  = 0;

/* Lookup table giai ma quadrature 4x.
   Index = (truoc << 2) | hien_tai, gia tri = +1 / -1 / 0
   Nguon goc: Gray-code state machine 2-bit                       */
static const int8_t QEM[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};


static void IRAM_ATTR enc_isr()
{
    int8_t ab = (int8_t)((digitalRead(Z_ENC_A) << 1) | digitalRead(Z_ENC_B));
    enc_pos += QEM[(last_ab << 2) | ab];
    last_ab = ab;
}

/* ---------------------------------------------------------------
   Bien dieu khien
   --------------------------------------------------------------- */
static int32_t  target_cnt  = 0;
static bool     _moving     = false;
static int32_t  prev_error  = 0;
static uint32_t last_pid_ms = 0;

/* ---------------------------------------------------------------
   Dieu khien H-bridge
   pwm_val: -Z_PWM_MAX .. +Z_PWM_MAX
   0 = phanh (brake: IN1=IN2=HIGH, PWM=0)
   --------------------------------------------------------------- */
static void motor_set(int16_t pwm_val)
{
    if (pwm_val > 0) {
        digitalWrite(Z_MOTOR_IN1, HIGH);
        digitalWrite(Z_MOTOR_IN2, LOW);
        ledcWrite(Z_LEDC_CH, (uint32_t)min(pwm_val, (int16_t)Z_PWM_MAX));
    } else if (pwm_val < 0) {
        digitalWrite(Z_MOTOR_IN1, LOW);
        digitalWrite(Z_MOTOR_IN2, HIGH);
        ledcWrite(Z_LEDC_CH, (uint32_t)min((int16_t)-pwm_val, (int16_t)Z_PWM_MAX));
    } else {
        /* Phanh tich cuc: IN1=IN2=HIGH, duty=0 */
        digitalWrite(Z_MOTOR_IN1, HIGH);
        digitalWrite(Z_MOTOR_IN2, HIGH);
        ledcWrite(Z_LEDC_CH, 0);
    }
}

/* =============================================================== */

void z_axis_init()
{
    pinMode(Z_MOTOR_IN1, OUTPUT);
    pinMode(Z_MOTOR_IN2, OUTPUT);
    pinMode(Z_MOTOR_PWM, OUTPUT);
    pinMode(Z_ENC_A, INPUT_PULLUP);
    pinMode(Z_ENC_B, INPUT_PULLUP);
    pinMode(Z_LIMIT_TOP, INPUT_PULLUP);   /* cong tac gioi han tren (active-LOW) */

    /* LEDC cho PWM dong co */
    ledcSetup(Z_LEDC_CH, Z_LEDC_FREQ, Z_LEDC_RES);
    ledcAttachPin(Z_MOTOR_PWM, Z_LEDC_CH);

    motor_set(0);

    /* Doc trang thai ban dau encoder tranh lech dem luc khoi dong */
    last_ab = (int8_t)((digitalRead(Z_ENC_A) << 1) | digitalRead(Z_ENC_B));
    enc_pos = 0;

    attachInterrupt(digitalPinToInterrupt(Z_ENC_A), enc_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Z_ENC_B), enc_isr, CHANGE);

    Serial.printf("[Z]  Init OK  ENC_A=GPIO%d  ENC_B=GPIO%d\n",
                  Z_ENC_A, Z_ENC_B);
    Serial.printf("[Z]  PPR=%d  gear=%.1f  pitch=%.1fmm  -> %.1f xung/mm\n",
                  Z_ENC_PPR, Z_GEAR_RATIO, Z_SCREW_PITCH, Z_PULSE_PER_MM);
    Serial.printf("[Z]  Hanh trinh: %.0f – %.0f mm\n", Z_MIN_MM, Z_MAX_MM);
}

bool z_axis_move_to(float z_mm)
{
    if (z_mm < Z_MIN_MM || z_mm > Z_MAX_MM) {
        Serial.printf("[Z]  ERR: %.2f mm ngoai hanh trinh [%.0f, %.0f]\n",
                      z_mm, Z_MIN_MM, Z_MAX_MM);
        return false;
    }

    target_cnt = (int32_t)(z_mm * Z_PULSE_PER_MM);
    prev_error = 0;
    _moving    = true;

    Serial.printf("[Z]  -> %.2f mm  (%ld xung)\n", z_mm, (long)target_cnt);
    return true;
}

bool z_axis_move_by(float delta_mm)
{
    return z_axis_move_to(z_axis_get_pos_mm() + delta_mm);
}

void z_axis_stop()
{
    motor_set(0);
    _moving = false;
    Serial.printf("[Z]  Dung tai %.2f mm\n", z_axis_get_pos_mm());
}

float z_axis_get_pos_mm()
{
    return (float)enc_pos / Z_PULSE_PER_MM;
}

bool z_axis_is_moving()
{
    return _moving;
}

void z_axis_set_zero()
{
    enc_pos    = 0;
    target_cnt = 0;
    _moving    = false;
    Serial.println("[Z]  Zero da duoc dat tai vi tri hien tai");
}

void z_axis_raw_pwm(int16_t pwm)
{
    _moving = false;   /* tat PID */
    motor_set(pwm);
}

int32_t z_axis_get_enc_cnt()
{
    return enc_pos;
}

bool z_limit_top_triggered()
{
    bool raw = digitalRead(Z_LIMIT_TOP);   /* HIGH khi tha (pull-up) */
    return Z_LIMIT_TOP_ACTIVE_LOW ? (raw == LOW) : (raw == HIGH);
}

void z_axis_set_pos_mm(float z_mm)
{
    int32_t cnt = (int32_t)(z_mm * Z_PULSE_PER_MM);
    noInterrupts();
    enc_pos = cnt;
    interrupts();
    target_cnt = cnt;   /* tranh PID phat sinh chuyen dong ngay lap tuc */
    _moving    = false;
    Serial.printf("[Z]  Force-set vi tri: %.1f mm (%ld xung)\n",
                  z_mm, (long)cnt);
}

/* ---------------------------------------------------------------
   PD Controller – goi trong loop()
   --------------------------------------------------------------- */
void z_axis_update()
{
    if (!_moving) return;

    uint32_t now = millis();
    if (now - last_pid_ms < Z_PID_INTERVAL_MS) return;
    last_pid_ms = now;

    int32_t pos   = enc_pos;              /* snapshot atomic tren 32-bit */
    int32_t error = target_cnt - pos;
    int32_t deriv = error - prev_error;
    prev_error    = error;

    /* Cong tac gioi han TREN (phan cung) – uu tien cao nhat.
       error > 0 => target > pos => dang di LEN. Cham dinh -> dung va
       chuan hoa Z = Z_MAX (cong tac la moc tham chieu dinh hanh trinh). */
    if (error > 0 && z_limit_top_triggered()) {
        motor_set(0);
        z_axis_set_pos_mm((float)Z_MAX_MM);   /* set _moving=false + re-zero dinh */
        Serial.println("[Z]  Cong tac gioi han TREN -> dung, Z=Z_MAX");
        return;
    }

    /* Kiem tra gioi han phan mem (bao ve cuoi hanh trinh) */
    float pos_mm = (float)pos / Z_PULSE_PER_MM;
    if ((pos_mm <= Z_MIN_MM && error < 0) ||
        (pos_mm >= Z_MAX_MM && error > 0)) {
        z_axis_stop();
        Serial.println("[Z]  CANH BAO: cham gioi han hanh trinh!");
        return;
    }

    /* Den dich: dung lai */
    if (abs(error) <= Z_DEADBAND) {
        motor_set(0);
        _moving = false;
        Serial.printf("[Z]  Den dich %.2f mm\n", pos_mm);
        return;
    }

    /* Tinh output PD */
    float output = Z_KP * (float)error + Z_KD * (float)deriv;

    /* Giu PWM toi thieu de thang ma sat, gioi han toi da */
    int16_t pwm = (int16_t)constrain(fabsf(output), Z_PWM_MIN, Z_PWM_MAX);
    motor_set((output >= 0.0f) ? pwm : (int16_t)-pwm);
}
