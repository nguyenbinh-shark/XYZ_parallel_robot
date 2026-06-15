#include <Arduino.h>
#include "z_axis.h"

/*
 * Test firmware: kiem tra rieng truc Z + doc encoder xem chuan khong.
 * Chon env PlatformIO: test_z_axis
 *   pio run -e test_z_axis -t upload
 *   pio device monitor -b 115200
 *
 * Lenh Serial 115200:
 *   help          In bang lenh
 *   z <mm>        Di den vi tri tuyet doi, vi du: z 10
 *   z+<mm>        Di len tuong doi, vi du: z+5
 *   z-<mm>        Di xuong tuong doi, vi du: z-3
 *   z0            Dat vi tri hien tai = 0 (zero)
 *   pos           In vi tri (mm) + so xung encoder tho
 *   stop          Dung dong co ngay
 *   raw <pwm>     Chay PWM truc tiep (-255..255), BO QUA PID
 *                 -> dung de KIEM TRA CHIEU ENCODER (xem ben duoi)
 *   mon on|off    Bat/tat in lien tuc vi tri + xung (200ms/lan)
 *   cal <mm>      Hieu chuan: sau khi do thuc te, nhap quang duong THAT
 *                 da di cho lan 'z+<mm>' gan nhat -> in he so xung/mm dung
 *
 * === QUY TRINH KIEM TRA ENCODER CHUAN ===
 * 1) Go 'mon on' de xem xung encoder thay doi realtime.
 * 2) Go 'z0' (zero), roi 'raw 80' (chay cham chieu duong).
 *    - Neu so xung TANG dan  -> encoder dung chieu (OK).
 *    - Neu so xung GIAM dan   -> encoder NGUOC chieu -> hoan doi
 *      Z_ENC_A <-> Z_ENC_B (hoac Z_MOTOR_IN1 <-> Z_MOTOR_IN2) trong z_axis.h.
 *    Go 'stop' de dung.
 * 3) Kiem tra quang duong: 'z0' -> 'z+10' -> doi dung -> do bang thuoc.
 *    Neu sai, go 'cal <so_do_thuc_te>' de tinh lai Z_SCREW_PITCH.
 */

#define MON_INTERVAL_MS 200

static String   rx_buf;
static bool     mon_on   = false;
static uint32_t mon_ms   = 0;
static float    last_cmd_mm = 0.0f;   /* delta da lenh cho lan di gan nhat */

static void print_help()
{
    Serial.println();
    Serial.println("=== TEST TRUC Z + ENCODER ===");
    Serial.printf ("He so hien tai: %.2f xung/mm  |  Hanh trinh %.0f..%.0f mm\n",
                   Z_PULSE_PER_MM, Z_MIN_MM, Z_MAX_MM);
    Serial.println("  help          In bang lenh");
    Serial.println("  z <mm>        Di den vi tri tuyet doi, vi du: z 10");
    Serial.println("  z+<mm>        Di len tuong doi, vi du: z+5");
    Serial.println("  z-<mm>        Di xuong tuong doi, vi du: z-3");
    Serial.println("  z0            Zero vi tri hien tai");
    Serial.println("  pos           In vi tri (mm) + xung encoder");
    Serial.println("  stop          Dung dong co");
    Serial.println("  raw <pwm>     Chay PWM truc tiep -255..255 (kiem tra chieu)");
    Serial.println("  mon on|off    In lien tuc vi tri + xung");
    Serial.println("  cal <mm>      Hieu chuan: nhap quang duong THUC TE vua do");
    Serial.println();
}

static void print_pos()
{
    int raw = digitalRead(Z_LIMIT_TOP);
    Serial.printf("[POS] %.3f mm  |  %ld xung  |  %s  |  lim_top=%s (GPIO%d raw=%s)\n",
                  z_axis_get_pos_mm(),
                  (long)z_axis_get_enc_cnt(),
                  z_axis_is_moving() ? "dang di" : "dung",
                  z_limit_top_triggered() ? "NHAN" : "tha",
                  Z_LIMIT_TOP, raw ? "HIGH" : "LOW");
}

static void process_command(String cmd)
{
    cmd.trim();
    if (cmd.length() == 0) return;

    if (cmd == "help" || cmd == "?") { print_help(); return; }
    if (cmd == "pos")  { print_pos(); return; }
    if (cmd == "stop") { z_axis_stop(); return; }
    if (cmd == "z0")   { z_axis_set_zero(); print_pos(); return; }

    if (cmd == "mon on")  { mon_on = true;  Serial.println("[MON] on");  return; }
    if (cmd == "mon off") { mon_on = false; Serial.println("[MON] off"); return; }

    if (cmd.startsWith("raw ")) {
        int16_t pwm = (int16_t)cmd.substring(4).toInt();
        z_axis_raw_pwm(pwm);
        Serial.printf("[RAW] PWM=%d (go 'stop' de dung). Theo doi xung tang/giam.\n", pwm);
        return;
    }

    if (cmd.startsWith("cal ")) {
        float real_mm = cmd.substring(4).toFloat();
        if (last_cmd_mm == 0.0f || real_mm == 0.0f) {
            Serial.println("[CAL] ERR: hay chay 'z+<mm>' truoc, roi do thuc te va 'cal <mm>'");
            return;
        }
        float pitch_moi = Z_SCREW_PITCH * (real_mm / last_cmd_mm);
        float ppmm_moi  = (Z_ENC_PPR * 4.0f * Z_GEAR_RATIO) / pitch_moi;
        Serial.printf("[CAL] Lenh %.2fmm, thuc te %.2fmm\n", last_cmd_mm, real_mm);
        Serial.printf("[CAL] -> Z_SCREW_PITCH moi = %.3f  (xung/mm = %.2f)\n",
                      pitch_moi, ppmm_moi);
        Serial.println("[CAL] Sua Z_SCREW_PITCH trong z_axis.h roi nap lai.");
        return;
    }

    if (cmd.startsWith("z")) {
        String val = cmd.substring(1);   /* bo 'z' */
        val.trim();
        if (val.startsWith("+")) {
            last_cmd_mm = val.substring(1).toFloat();
            z_axis_move_by(last_cmd_mm);
        } else if (val.startsWith("-")) {
            last_cmd_mm = -val.substring(1).toFloat();
            z_axis_move_by(last_cmd_mm);
        } else if (val.length() > 0) {
            last_cmd_mm = 0.0f;          /* tuyet doi -> khong dung cho 'cal' */
            z_axis_move_to(val.toFloat());
        }
        return;
    }

    Serial.printf("[ERR] Khong hieu lenh: %s  (go 'help')\n", cmd.c_str());
}

void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println("\n=== TEST TRUC Z (encoder + PID) ===");
    z_axis_init();
    print_help();
}

void loop()
{
    z_axis_update();   /* PID truc Z – phai goi lien tuc */

    if (mon_on && (millis() - mon_ms >= MON_INTERVAL_MS)) {
        mon_ms = millis();
        print_pos();
    }

    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            process_command(rx_buf);
            rx_buf = "";
        } else {
            rx_buf += c;
        }
    }
}
