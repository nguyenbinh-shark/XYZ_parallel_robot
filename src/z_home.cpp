#include "z_home.h"
#include "z_axis.h"
#include "vl53l0x.h"

/* ---------------------------------------------------------------
   State machine dung chung cho slow homing va teach mode
   --------------------------------------------------------------- */
enum class ZHState : uint8_t {
    IDLE,
    STEPPING,    /* dang ha tung buoc, doi PID xong de doc sensor  */
    FOUND,       /* be mat tim thay – xu ly                         */
    LIFTING,     /* nang len Z_HOME_LIFT_MM (chi cho homing)         */
    DONE
};

static ZHState  _state    = ZHState::IDLE;
static bool     _is_teach = false;      /* false = homing, true = teach */
static float   *_teach_out = nullptr;   /* con tro luu ket qua teach    */

/* Homing ve gioi han tren (cong tac) – co che rieng (raw PWM + cong tac) */
static bool     _top_homing = false;
static uint32_t _top_t0     = 0;

/* ======================== A. Instant homing ======================== */

bool z_home_instant()
{
    int16_t d = vl53l0x_read_avg(3);   /* ~169ms (3x33ms do + 2x35ms delay) */
    if (d < 0) {
        Serial.println("[HOME] ERR: VL53L0X khong co gia tri hop le");
        return false;
    }

    float z_mm = (float)d - Z_HOME_SENSOR_OFFSET_MM;
    if (z_mm < 0.0f) z_mm = 0.0f;

    z_axis_set_pos_mm(z_mm);
    Serial.printf("[HOME] Tuc thoi: sensor=%dmm -> Z=%.1fmm\n", d, z_mm);
    return true;
}

/* ======================== B/C. Slow homing / Teach ================ */

static void enter(ZHState s) { _state = s; }

static void start_common()
{
    if (_state != ZHState::IDLE) {
        Serial.println("[HOME] Dang chay – bo qua lenh moi");
        return;
    }
    /* CANH BAO: ham nay force-set encoder = Z_MAX_MM roi ha dan xuong.
       Chi goi khi robot THUC SU dang o vi tri cao nhat (cuoi hanh trinh
       phia tren). Neu goi sai luc robot dang giua hanh trinh, motor se
       chay xuong tu vi tri "ao" va co the gay hu hong co khi.          */
    z_axis_set_pos_mm((float)Z_MAX_MM);
    z_axis_move_to(Z_MAX_MM - Z_HOME_STEP_MM);
    enter(ZHState::STEPPING);
    Serial.println("[HOME] Bat dau ha cham – dam bao robot o vi tri CAO NHAT");
}

void z_home_slow_start()
{
    _is_teach  = false;
    _teach_out = nullptr;
    start_common();
}

void z_teach_start(float *out_z_mm)
{
    _is_teach  = true;
    _teach_out = out_z_mm;
    start_common();
    Serial.println("[TEACH] Ha Z de xac dinh be mat...");
}

/* ===================== B'. Homing ve gioi han tren ================= */

void z_home_top_start()
{
    if (z_home_busy()) {
        Serial.println("[HOME] Dang chay – bo qua lenh moi");
        return;
    }
    if (z_limit_top_triggered()) {
        /* Da cham cong tac san -> chi can chuan hoa, khong chay motor */
        z_axis_set_pos_mm((float)Z_MAX_MM);
        Serial.println("[HOME] Da o gioi han tren -> Z=Z_MAX");
        return;
    }
    _top_homing = true;
    _top_t0     = millis();
    z_axis_raw_pwm((int16_t)Z_HOME_TOP_PWM);   /* chay LEN cham (bo qua PID) */
    Serial.println("[HOME] Chay len tim cong tac gioi han tren...");
}

void z_home_abort()
{
    _top_homing = false;
    _state      = ZHState::IDLE;
    z_axis_stop();
}

void z_home_update()
{
    /* --- Homing ve gioi han tren (cong tac GPIO Z_LIMIT_TOP) --- */
    if (_top_homing) {
        if (z_limit_top_triggered()) {
            z_axis_stop();
            z_axis_set_pos_mm((float)Z_MAX_MM);
            _top_homing = false;
            Serial.printf("[HOME] Cham gioi han tren -> Z=%.0f mm\n", (float)Z_MAX_MM);
        } else if (millis() - _top_t0 > (uint32_t)Z_HOME_TOP_TIMEOUT_MS) {
            z_axis_stop();
            _top_homing = false;
            Serial.println("[HOME] FAIL: timeout – khong cham cong tac");
            Serial.println("[HOME] Kiem tra chieu motor (raw PWM phai di LEN) va day cong tac");
        }
        return;
    }

    if (_state == ZHState::IDLE || _state == ZHState::DONE) return;

    switch (_state) {

    case ZHState::STEPPING:
        if (z_axis_is_moving()) return;   /* Doi PID on dinh */

        {
            int16_t d = vl53l0x_read_mm();
            float   pos = z_axis_get_pos_mm();

            if (d > 0 && d < (int16_t)Z_HOME_NEAR_MM) {
                /* Tim thay be mat */
                Serial.printf("[HOME] Be mat: sensor=%dmm  Z=%.1fmm\n", d, pos);
                enter(ZHState::FOUND);
            } else if (pos <= 0.1f) {
                /* Het hanh trinh ma khong tim thay */
                Serial.println("[HOME] FAIL: het hanh trinh, khong phat hien be mat");
                Serial.println("[HOME] Kiem tra VL53L0X va Z_HOME_SENSOR_OFFSET_MM");
                enter(ZHState::DONE);
            } else {
                /* Tiep tuc ha them 1 buoc */
                float next = pos - Z_HOME_STEP_MM;
                if (next < 0.0f) next = 0.0f;
                z_axis_move_to(next);
            }
        }
        break;

    case ZHState::FOUND:
        if (_is_teach) {
            /* Teach: ghi lai vi tri, khong set zero, khong nang len */
            float result = z_axis_get_pos_mm();
            if (_teach_out) *_teach_out = result;
            Serial.printf("[TEACH] Vi tri be mat: Z=%.2f mm\n", result);
            enter(ZHState::DONE);
        } else {
            /* Homing: set zero tai day, nang len */
            z_axis_set_zero();
            z_axis_move_to(Z_HOME_LIFT_MM);
            Serial.printf("[HOME] Zero da dat. Nang len %.0f mm...\n",
                          (float)Z_HOME_LIFT_MM);
            enter(ZHState::LIFTING);
        }
        break;

    case ZHState::LIFTING:
        if (!z_axis_is_moving()) {
            Serial.println("[HOME] Homing hoan thanh.");
            enter(ZHState::DONE);
        }
        break;

    case ZHState::DONE:
        _state = ZHState::IDLE;
        break;

    default:
        _state = ZHState::IDLE;
    }
}

bool z_home_busy()
{
    return _top_homing || (_state != ZHState::IDLE);
}

/* ======================== D. Phat hien vat ======================== */

bool z_grip_has_object()
{
    int16_t d = vl53l0x_read_mm();
    if (d < 0) return true;   /* Cam bien loi: gia dinh co vat (an toan hon) */
    return (d < (int16_t)Z_GRIP_DETECT_MM);
}
