#include <Arduino.h>
#include <ESP32Servo.h>
#include "ik_5bar.h"
#include "vl53l0x.h"
#include "gripper.h"
#include "z_axis.h"
#include "z_home.h"
#include "oled.h"

/* =====================================================================
   5-Bar Parallel Robot – ESP32-S3
   Dieu khien qua Serial 115200 baud
   Lenh XY : x,y | home | demo | pos | dist
   Lenh Z  : z<mm> | z+<mm> | z-<mm> | z0 | zstop | zpos
   Gripper : suck | drop | pump on | pump off
   Pick&Place: pick x,y,z | place x,y,z | safez z | run | abort | ppinfo
   Chung   : help
   ===================================================================== */

/* ---------- Cau hinh phan cung ---------- */
#define PIN_SERVO1      48      /* Motor 1 – goc A (trai)              */
#define PIN_SERVO2      47      /* Motor 2 – goc C (phai)              */

#define SERVO_HZ        50
#define SERVO_US_MIN    500
#define SERVO_US_MAX    2500

/* Toc do goc toi da khi noi suy servo XY (do/giay).
   Gioi han van toc -> chuyen dong muot, giam giat & dong dien dinh.
   2 servo luon ket thuc CUNG LUC (chung tham so u) -> chong noi luc 5-bar. */
#define SERVO_SPEED_DPS     150.0f
/* Chu ky cap nhat noi suy servo (ms) – trung khop nhip servo 50Hz (20ms) */
#define SERVO_XY_INTERVAL_MS  20

/* Hieu chinh offset lap dat (do): PHAI khop voi cach lap coi servo thuc te
   VA voi SV1_OFF/SV2_OFF trong hmi/index.html. Hien dang dung -45/+45.      */
#define SERVO1_OFFSET  -45.0f    /* Servo GPIO48 */
#define SERVO2_OFFSET  +45.0f    /* Servo GPIO47 */

/* Motor 2 cung chieu Motor 1 (SERVO2_INVERT = false)                   */
#define SERVO2_INVERT   false

/* ---------- Vi tri Home ---------- */
/* Trung diem 2 motor (X=27.5), Y du cao de NAM TRONG vung lam viec:
   vung chet ban kinh |L2-L1|=60mm quanh moi dong co -> tai tam X=27.5
   can Y >= ~53mm. Chon 80mm + margin cho demo (HOME +/-10).            */
#define HOME_X  (IK_L0 * 0.5f)   /* 27.5 mm */
#define HOME_Y  80.0f

/* =================================================================== */

static Servo    s1, s2;
static float    cur_x = HOME_X;
static float    cur_y = HOME_Y;
static String   rx_buf;
static uint32_t oled_ms   = 0;
static int16_t  oled_dist = -1;

/* ---------- Trang thai noi suy servo XY (non-blocking) ---------- */
static int      cur_a1 = 90, cur_a2 = 90;   /* goc servo hien tai (do)     */
static int      a1_start, a2_start;          /* goc dau khi bat dau noi suy */
static int      a1_goal,  a2_goal;           /* goc dich                    */
static uint32_t xy_t_start  = 0;             /* moc thoi gian bat dau (ms)  */
static uint32_t xy_T_total  = 0;             /* tong thoi gian di chuyen(ms)*/
static bool     xy_moving   = false;
static uint32_t xy_tick_ms  = 0;             /* nhip cap nhat noi suy       */

/* Phien ban firmware (tra ve qua giao thuc lenh VER) */
#define FW_VERSION  "1.0"

/* Forward declaration – dinh nghia day du o phia duoi */
static bool move_to(float x, float y);
static bool xy_is_moving() { return xy_moving; }
static void demo_stop();
static void proto_event(const char *ev, const char *arg = nullptr);

/* =====================================================================
   Pick & Place – State Machine (non-blocking)
   Phoi hop: XY servo (5-bar) + Truc Z (DC encoder) + Giac hut
   ===================================================================== */

struct PPTask {
    float pick_x,  pick_y,  pick_z;   /* Vi tri lay vat (mm)              */
    float place_x, place_y, place_z;  /* Vi tri dat vat (mm)              */
    float safe_z;                      /* Do cao an toan khi di chuyen XY  */
    uint16_t xy_settle_ms;             /* Thoi gian cho servo XY on dinh   */
    uint16_t grip_ms;                  /* Thoi gian giac hut bam chac      */
    uint16_t drop_ms;                  /* Thoi gian sau nha truoc khi nang  */
};

static PPTask pp_task = {
    /* pick  */ HOME_X,        HOME_Y,   5.0f,
    /* place */ HOME_X + 15.0f, HOME_Y,  5.0f,
    /* safe  */ 20.0f,
    /* times */ 500, 400, 300
};

enum class PPState : uint8_t {
    IDLE,
    PICK_XY,        /* Di XY den vi tri lay, cho Z an toan & servo on dinh */
    PICK_LOWER,     /* Ha Z xuong pick_z                                    */
    PICK_GRIP,      /* Kich hoat giac hut, cho bam chac                    */
    PICK_VERIFY,    /* Doc VL53L0X kiem tra co vat hay khong               */
    PICK_RAISE,     /* Nang Z len safe_z                                    */
    PLACE_XY,       /* Di XY den vi tri dat, monitor VL53L0X               */
    PLACE_LOWER,    /* Ha Z xuong place_z                                   */
    PLACE_DROP,     /* Mo van (non-blocking), cho nha xong                  */
    PLACE_RAISE,    /* Nang Z len safe_z                                    */
    DROP_DETECTED,  /* Vat roi trong luc van chuyen – xu ly loi             */
    DONE
};

static PPState  pp_state        = PPState::IDLE;
static uint32_t pp_ms           = 0;
static uint32_t pp_drop_chk_ms  = 0;  /* timer kiem tra VL53L0X dinh ky khi van chuyen */

static const char* pp_state_name()
{
    switch (pp_state) {
        case PPState::PICK_XY:     return "PICK_XY";
        case PPState::PICK_LOWER:  return "PICK_LOWER";
        case PPState::PICK_GRIP:   return "PICK_GRIP";
        case PPState::PICK_VERIFY: return "PICK_VERIFY";
        case PPState::PICK_RAISE:  return "PICK_RAISE";
        case PPState::PLACE_XY:    return "PLACE_XY";
        case PPState::PLACE_LOWER: return "PLACE_LOWER";
        case PPState::PLACE_DROP:     return "PLACE_DROP";
        case PPState::PLACE_RAISE:    return "PLACE_RAISE";
        case PPState::DROP_DETECTED:  return "DROP_DETECTED";
        case PPState::DONE:           return "DONE";
        default:                   return "IDLE";
    }
}

static void pp_enter(PPState next)
{
    pp_state = next;
    pp_ms    = millis();
    Serial.printf("[PP] -> %s\n", pp_state_name());
    proto_event("ST", pp_state_name());   /* bao trang thai cho host (neu bat) */
}

static bool pp_is_busy() { return pp_state != PPState::IDLE; }

static void pp_abort()
{
    z_axis_stop();
    gripper_release();
    pp_state = PPState::IDLE;
    Serial.println("[PP] ABORT");
}

static void pp_start()
{
    if (pp_is_busy()) {
        Serial.println("[PP] ERR: Dang chay – go 'abort' truoc");
        return;
    }
    demo_stop();   /* tranh demo va PP cung lenh move_to */
    Serial.println("[PP] === Bat dau Pick & Place ===");
    Serial.printf("[PP]  Lay : X=%.1f Y=%.1f Z=%.1f\n",
                  pp_task.pick_x,  pp_task.pick_y,  pp_task.pick_z);
    Serial.printf("[PP]  Dat : X=%.1f Y=%.1f Z=%.1f\n",
                  pp_task.place_x, pp_task.place_y, pp_task.place_z);
    Serial.printf("[PP]  SafeZ=%.1f  settle=%dms  grip=%dms  drop=%dms\n",
                  pp_task.safe_z, pp_task.xy_settle_ms,
                  pp_task.grip_ms, pp_task.drop_ms);

    /* Nang Z an toan va di XY den vi tri lay dong thoi */
    z_axis_move_to(pp_task.safe_z);
    move_to(pp_task.pick_x, pp_task.pick_y);
    pp_enter(PPState::PICK_XY);
}

/* Goi lien tuc trong loop() */
static void pp_update()
{
    if (pp_state == PPState::IDLE) return;
    uint32_t elapsed = millis() - pp_ms;

    switch (pp_state) {
    case PPState::PICK_XY:
        /* Doi ca Z dat safe_z, servo XY noi suy xong (tin hieu THAT), va
           them dwell xy_settle_ms cho servo hobby on dinh co hoc          */
        if (!z_axis_is_moving() && !xy_is_moving() &&
            elapsed >= pp_task.xy_settle_ms) {
            z_axis_move_to(pp_task.pick_z);
            pp_enter(PPState::PICK_LOWER);
        }
        break;
    case PPState::PICK_LOWER:
        if (!z_axis_is_moving()) {
            gripper_suck();
            pp_enter(PPState::PICK_GRIP);
        }
        break;
    case PPState::PICK_GRIP:
        if (elapsed >= pp_task.grip_ms) {
            pp_enter(PPState::PICK_VERIFY);
        }
        break;
    case PPState::PICK_VERIFY:
        /* Doc VL53L0X kiem tra co vat hay khong, cho 80ms on dinh */
        if (elapsed >= 80) {
            if (z_grip_has_object()) {
                Serial.println("[PP] Vat duoc hut thanh cong");
            } else {
                Serial.println("[PP] CANH BAO: Khong phat hien vat – kiem tra giac hut");
                proto_event("EVT", "NOOBJ");
                /* Tiep tuc (khong abort) – nguoi van co the dieu chinh */
            }
            z_axis_move_to(pp_task.safe_z);
            pp_enter(PPState::PICK_RAISE);
        }
        break;
    case PPState::PICK_RAISE:
        if (!z_axis_is_moving()) {
            move_to(pp_task.place_x, pp_task.place_y);
            pp_drop_chk_ms = millis();
            pp_enter(PPState::PLACE_XY);
        }
        break;
    case PPState::PLACE_XY:
        /* Kiem tra VL53L0X moi 300ms – phat hien vat roi trong luc van chuyen */
        if ((millis() - pp_drop_chk_ms) >= 300) {
            pp_drop_chk_ms = millis();
            if (!z_grip_has_object()) {
                Serial.println("[PP] !!! VAT ROI – dung Z, tat bom, nang len safe_z");
                z_axis_stop();
                gripper_pump_off();
                z_axis_move_to(pp_task.safe_z);
                pp_enter(PPState::DROP_DETECTED);
                break;
            }
        }
        if (!xy_is_moving() && elapsed >= pp_task.xy_settle_ms) {
            z_axis_move_to(pp_task.place_z);
            pp_enter(PPState::PLACE_LOWER);
        }
        break;
    case PPState::PLACE_LOWER:
        if (!z_axis_is_moving()) {
            gripper_valve_open();          /* Non-blocking: mo van, bom van chay */
            pp_enter(PPState::PLACE_DROP);
        }
        break;
    case PPState::PLACE_DROP:
        /* t >= GRIPPER_RELEASE_PUMP_DELAY_MS: tat bom (idempotent) */
        if (elapsed >= GRIPPER_RELEASE_PUMP_DELAY_MS) {
            gripper_pump_off();
        }
        /* t >= drop_ms: nang Z, chuyen state */
        if (elapsed >= pp_task.drop_ms) {
            z_axis_move_to(pp_task.safe_z);
            pp_enter(PPState::PLACE_RAISE);
        }
        break;
    case PPState::PLACE_RAISE:
        if (!z_axis_is_moving()) {
            pp_enter(PPState::DONE);
        }
        break;

    case PPState::DROP_DETECTED:
        /* Cho Z nang len safe_z (da lenh truoc khi vao state nay) */
        if (!z_axis_is_moving()) {
            Serial.println("[PP] === LOI: Vat roi – quay ve IDLE ===");
            Serial.println("[PP]  Kiem tra giac hut va chay lai 'run'");
            pp_state = PPState::IDLE;
        }
        break;
    case PPState::DONE:
        Serial.printf("[PP] === Hoan thanh ===  X=%.1f Y=%.1f Z=%.1f\n",
                      cur_x, cur_y, z_axis_get_pos_mm());
        pp_state = PPState::IDLE;
        break;
    default:
        pp_state = PPState::IDLE;
    }
}
/* ------------------------------------------------------------------
   reverse_dir: anh xa nguoc gia tri goc 0..180 -> 180..0.
   IK tinh theo quy uoc NGUOC chieu kim dong ho (CCW), nhung dong co
   thuc te quay THUAN chieu kim dong ho (CW) -> phai dao chieu.
   ------------------------------------------------------------------ */
static inline float reverse_dir(float angle)
{
    return 180.0f - angle;
}

/* Chuan hoa goc ve [0, 360). IK (atan/atan2) co the tra goc lech +/-360
   -> phai wrap truoc khi so [0,180], neu khong se bi "qua gioi han ao".  */
static inline float wrap360(float a)
{
    a = fmodf(a, 360.0f);
    if (a < 0.0f) a += 360.0f;
    return a;
}

/* ------------------------------------------------------------------
   map_servo1/2: doi goc IK (do, CCW) sang goc vat ly servo (CW).
   Wrap ve [0,360) roi chap nhan neu <= 180; nguoc lai (180..360) la
   that su ngoai hanh trinh servo -> tra -1.
   ------------------------------------------------------------------ */
static float map_servo1(float theta)
{
    float a = wrap360(reverse_dir(theta + SERVO1_OFFSET));
    return (a <= 180.0f) ? a : -1.0f;
}

static float map_servo2(float theta)
{
    float a = reverse_dir(theta + SERVO2_OFFSET);
    if (SERVO2_INVERT) a = 180.0f - a;   /* truong hop servo 2 lap doi xung */
    a = wrap360(a);
    return (a <= 180.0f) ? a : -1.0f;
}

/* ------------------------------------------------------------------
   move_to: tinh IK va KHOI DONG noi suy 2 servo den (x, y).
   KHONG ghi servo tuc thi nua – servo_xy_update() noi suy dan trong loop().
   Tra ve false neu diem ngoai workspace hoac goc vuot giai han servo
   (validate DICH; cac diem trung gian nam giua 2 goc hop le nen luon
    trong [0,180] – tham so u chung dam bao 2 truc ket thuc cung luc).
   ------------------------------------------------------------------ */
static bool move_to(float x, float y)
{
    float t1, t2;
    if (!ik_5bar(x, y, &t1, &t2)) {
        Serial.printf("[ERR] IK vo nghiem: (%.2f, %.2f)\n", x, y);
        return false;
    }

    float a1 = map_servo1(t1);
    float a2 = map_servo2(t2);

    if (a1 < 0.0f || a2 < 0.0f) {
        Serial.printf("[ERR] Goc ngoai giai han servo: theta1=%.1f  theta2=%.1f\n",
                      t1, t2);
        return false;
    }

    /* Khoi dong noi suy tu vi tri HIEN TAI (cur_a1/cur_a2) den dich.
       T_total ti le voi delta goc lon nhat -> van toc goc <= SERVO_SPEED_DPS,
       buoc lon di lau hon, buoc nho nhanh hon (van toc gioi han, muot).      */
    a1_goal = (int)roundf(a1);
    a2_goal = (int)roundf(a2);
    a1_start = cur_a1;
    a2_start = cur_a2;

    int max_delta = max(abs(a1_goal - a1_start), abs(a2_goal - a2_start));
    xy_T_total = (uint32_t)((float)max_delta * 1000.0f / SERVO_SPEED_DPS);
    if (xy_T_total < (uint32_t)SERVO_XY_INTERVAL_MS)
        xy_T_total = SERVO_XY_INTERVAL_MS;   /* it nhat 1 tick (tranh chia 0) */

    xy_t_start = millis();
    xy_tick_ms = 0;                          /* ep cap nhat ngay tick ke tiep */
    xy_moving  = true;
    cur_x = x;                               /* vi tri logic = dich da lenh   */
    cur_y = y;

    Serial.printf("[OK] -> (%.1f, %.1f) mm  |  t1=%.1f° t2=%.1f°  |  s1=%d° s2=%d°  T=%lums\n",
                  x, y, t1, t2, a1_goal, a2_goal, (unsigned long)xy_T_total);
    return true;
}

/* ------------------------------------------------------------------
   servo_xy_update: noi suy tuyen tinh goc theo thoi gian (goi trong loop()).
   2 servo DUNG CHUNG tham so u in [0,1] -> dong bo, ket thuc cung luc.
   ------------------------------------------------------------------ */
static void servo_xy_update()
{
    if (!xy_moving) return;
    if (millis() - xy_tick_ms < SERVO_XY_INTERVAL_MS) return;
    xy_tick_ms = millis();

    float u = (float)(millis() - xy_t_start) / (float)xy_T_total;
    if (u >= 1.0f) u = 1.0f;

    cur_a1 = a1_start + (int)roundf((a1_goal - a1_start) * u);
    cur_a2 = a2_start + (int)roundf((a2_goal - a2_start) * u);
    s1.write(cur_a1);
    s2.write(cur_a2);

    if (u >= 1.0f) xy_moving = false;        /* da toi dich (tin hieu THAT)  */
}
/* ---- Demo: hinh vuong 20x20 mm quanh Home (NON-BLOCKING) ----
   State machine: di tung diem, doi servo toi noi (xy_is_moving) + dwell nho.
   KHONG dung delay() -> khong chan Serial/abort, khong bo doi PID truc Z.    */
static const float demo_pts[][2] = {
    { HOME_X - 10, HOME_Y - 10 },
    { HOME_X + 10, HOME_Y - 10 },
    { HOME_X + 10, HOME_Y + 10 },
    { HOME_X - 10, HOME_Y + 10 },
    { HOME_X,      HOME_Y      },
};
#define DEMO_NPTS    (sizeof(demo_pts) / sizeof(demo_pts[0]))
#define DEMO_DWELL_MS 300            /* dung tai moi diem (ms) */

static int      demo_idx = -1;       /* -1 = khong chay */
static uint32_t demo_ms  = 0;

static void cmd_demo()
{
    Serial.println("[DEMO] Hinh vuong 20x20 mm (non-blocking)...");
    demo_idx = 0;
    move_to(demo_pts[0][0], demo_pts[0][1]);
    demo_ms = millis();
}

static void demo_update()
{
    if (demo_idx < 0) return;
    if (xy_is_moving()) return;                       /* doi servo toi diem   */
    if (millis() - demo_ms < DEMO_DWELL_MS) return;   /* dwell tai diem        */

    demo_idx++;
    if ((size_t)demo_idx >= DEMO_NPTS) {
        demo_idx = -1;
        Serial.println("[DEMO] Xong.");
        return;
    }
    move_to(demo_pts[demo_idx][0], demo_pts[demo_idx][1]);
    demo_ms = millis();
}

static void demo_stop() { demo_idx = -1; }
static void cmd_help()
{
    Serial.println("--- Lenh XY ---");
    Serial.println("  x,y      Di chuyen den (x, y) mm, vi du: 19,50");
    Serial.println("  home     Ve vi tri home XY");
    Serial.println("  demo     Chay demo hinh vuong 20x20");
    Serial.println("  pos      Hien vi tri XYZ + giac hut");
    Serial.println("  dist     Doc khoang cach VL53L0X");
    Serial.println("--- Lenh Z ---");
    Serial.println("  z<mm>    Di chuyen den vi tri Z tuyet doi, vi du: z25.5");
    Serial.println("  z+<mm>   Di chuyen len (tuong doi), vi du: z+5");
    Serial.println("  z-<mm>   Di chuyen xuong (tuong doi), vi du: z-3");
    Serial.println("  z0       Dat vi tri Z hien tai = 0 (zero)");
    Serial.println("  zstop    Dung dong co Z ngay lap tuc");
    Serial.println("  zpos     Hien vi tri Z hien tai");
    Serial.println("--- Giac hut ---");
    Serial.println("  suck     Bat bom + kich hoat giac hut");
    Serial.println("  drop     Nha vat + tat bom");
    Serial.println("  pump on  Bat bom (khong doi van)");
    Serial.println("  pump off Tat bom (khong doi van)");
    Serial.println("--- Homing & Teach (VL53L0X) ---");
    Serial.println("  homez        Instant homing: doc VL53L0X, set Z ngay (~105ms)");
    Serial.println("  homez slow   Slow homing: ha Z tung buoc den khi gap be mat");
    Serial.println("  hometop      Homing ve gioi han TREN (cong tac end-stop)");
    Serial.println("  lim?         Trang thai cong tac gioi han tren");
    Serial.println("  teachpick    Ha Z tim be mat, ghi lai lam pick_z");
    Serial.println("  teachplace   Ha Z tim be mat, ghi lai lam place_z");
    Serial.println("  grip?        Kiem tra co vat hay khong (doc VL53L0X)");
    Serial.println("--- Pick & Place ---");
    Serial.println("  pick x,y,z   Dat vi tri lay vat, vi du: pick 19,45,5");
    Serial.println("  place x,y,z  Dat vi tri dat vat, vi du: place 34,45,5");
    Serial.println("  safez z      Dat do cao an toan, vi du: safez 20");
    Serial.println("  settle ms    Thoi gian cho servo XY (ms), vi du: settle 500");
    Serial.println("  grip ms      Thoi gian hut bam (ms), vi du: grip 400");
    Serial.println("  drop ms      Thoi gian nha (ms), vi du: drop 300");
    Serial.println("  run          Thuc hien sequence pick-and-place");
    Serial.println("  abort        Dung khan cap, nha vat");
    Serial.println("  ppinfo       Hien cau hinh pick-and-place hien tai");
    Serial.println("  help         Hien bang lenh nay");
    Serial.println("--- Giao thuc may (cho host: GUI/Python/MCU) ---");
    Serial.println("  $<seq>,<CMD>,..*<CC>  Khung lenh may (vd: $1,MOVE,19,45*7A)");
    Serial.println("                        Tra ve #..OK/ERR, @telemetry, !su kien");
    Serial.println("                        Chi tiet day du: xem PROTOCOL.md");
}
/* Helper: tach "a,b,c" thanh 3 so thuc */
static bool parse3f(const String &s, float &a, float &b, float &c)
{
    int i1 = s.indexOf(',');
    int i2 = (i1 >= 0) ? s.indexOf(',', i1 + 1) : -1;
    if (i1 < 0 || i2 < 0) return false;
    a = s.substring(0, i1).toFloat();
    b = s.substring(i1 + 1, i2).toFloat();
    c = s.substring(i2 + 1).toFloat();
    return true;
}
/* =====================================================================
   GIAO THUC SERIAL CO KHUNG (machine protocol) – chay SONG SONG lenh text
   ---------------------------------------------------------------------
   Host -> ESP :  $<seq>,<CMD>,<arg1>,<arg2>,...*<CC>\n
   ESP  -> Host:  #<seq>,OK,<CMD>[,<data>...]*<CC>     (chap nhan / xong)
                  #<seq>,ERR,<code>[,<msg>]*<CC>        (tu choi / loi)
                  @<STATE>,<x>,<y>,<z>,<grip>,<dist>,<xymv>,<zmv>*<CC>  (telemetry)
                  !<EV>[,<arg>]*<CC>                    (su kien bat dong bo)
   <CC> = XOR (2 hex) cua TAT CA byte giua marker dau va '*'.
          Chieu Host->ESP: co '*<CC>' -> kiem tra (sai -> ERR,CRC);
                           khong co '*' -> van chap nhan (tien go tay).
   Dong bat dau bang '$' -> giao thuc; nguoc lai -> lenh text cu (process).
   Chi tiet lenh: xem PROTOCOL.md
   ===================================================================== */

static bool     proto_events_on = false;   /* bat phat su kien '!'        */
static uint32_t tlm_period_ms   = 0;        /* chu ky stream '@' (0 = tat) */
static uint32_t tlm_last_ms     = 0;

static uint8_t proto_xor(const char *s, size_t n)
{
    uint8_t cc = 0;
    for (size_t i = 0; i < n; i++) cc ^= (uint8_t)s[i];
    return cc;
}

/* Gui 1 frame: marker + body + '*' + checksum(hex 2 ky tu) + '\n'.
   body KHONG gom marker va KHONG gom '*CC'. */
static void proto_send(char marker, const String &body)
{
    uint8_t cc = proto_xor(body.c_str(), body.length());
    Serial.printf("%c%s*%02X\n", marker, body.c_str(), cc);
}

static void proto_ok(const String &seq, const char *cmd, const String &data = String())
{
    String body = seq + ",OK," + cmd;
    if (data.length()) { body += ','; body += data; }
    proto_send('#', body);
}

static void proto_err(const String &seq, const char *code, const char *msg = nullptr)
{
    String body = seq + ",ERR," + code;
    if (msg) { body += ','; body += msg; }
    proto_send('#', body);
}

/* Chuoi truong telemetry (khong kem marker/seq):
   STATE,X,Y,Z,GRIP,DIST,XYMV,ZMV.  Dung DIST cache (oled_dist) de KHONG
   doc VL53L0X (33ms) – tranh lam tre vong PID truc Z.                   */
static String tlm_body()
{
    char buf[104];
    /* STATE,X,Y,Z,GRIP,DIST,XYMV,ZMV,LIM,S1,S2  (S1/S2 = goc servo hien tai) */
    snprintf(buf, sizeof(buf), "%s,%.1f,%.1f,%.2f,%d,%d,%d,%d,%d,%d,%d",
             pp_state_name(), cur_x, cur_y, z_axis_get_pos_mm(),
             gripper_is_sucking() ? 1 : 0, (int)oled_dist,
             xy_is_moving() ? 1 : 0, z_axis_is_moving() ? 1 : 0,
             z_limit_top_triggered() ? 1 : 0, cur_a1, cur_a2);
    return String(buf);
}

static void proto_event(const char *ev, const char *arg)
{
    if (!proto_events_on) return;
    String body = String(ev);
    if (arg) { body += ','; body += arg; }
    proto_send('!', body);
}

/* Tach chuoi "a,b,c" thanh mang float. Tra ve so phan tu doc duoc. */
static int split_floats(const String &s, float *out, int maxn)
{
    int n = 0, start = 0;
    int len = s.length();
    while (n < maxn && start <= len) {
        int c = s.indexOf(',', start);
        String tok = (c >= 0) ? s.substring(start, c) : s.substring(start);
        tok.trim();
        if (tok.length() == 0 && c < 0) break;
        out[n++] = tok.toFloat();
        if (c < 0) break;
        start = c + 1;
    }
    return n;
}

static void proto_handle(const String &line)
{
    proto_events_on = true;   /* host da noi giao thuc -> bat su kien '!' */

    /* --- Tach checksum (neu co) va validate --- */
    int    star = line.indexOf('*');
    String payload;
    if (star >= 0) {
        payload = line.substring(1, star);
        String ccs = line.substring(star + 1);
        ccs.trim();
        uint8_t got  = proto_xor(payload.c_str(), payload.length());
        uint8_t want = (uint8_t)strtol(ccs.c_str(), nullptr, 16);
        if (ccs.length() == 0 || got != want) {
            proto_err("0", "CRC", "checksum sai");
            return;
        }
    } else {
        payload = line.substring(1);   /* bo '$' */
    }

    /* --- Tach seq, CMD, args --- */
    int    p    = payload.indexOf(',');
    String seq  = (p >= 0) ? payload.substring(0, p) : payload;
    String rest = (p >= 0) ? payload.substring(p + 1) : String();
    seq.trim();

    int    q    = rest.indexOf(',');
    String cmd  = (q >= 0) ? rest.substring(0, q) : rest;
    String args = (q >= 0) ? rest.substring(q + 1) : String();
    cmd.trim();  cmd.toUpperCase();

    float v[3];

    /* ===== Nhom 1: luon cho phep (ke ca khi PP/Home dang chay) ===== */
    if (cmd == "PING")   { proto_ok(seq, "PING", "PONG");     return; }
    if (cmd == "VER")    { proto_ok(seq, "VER", FW_VERSION);  return; }
    if (cmd == "STATUS" || cmd == "POS") { proto_ok(seq, "STATUS", tlm_body()); return; }
    if (cmd == "ABORT")  {
        z_home_abort(); pp_abort(); demo_stop(); xy_moving = false;
        proto_ok(seq, "ABORT"); return;
    }
    if (cmd == "TLM") {
        int n = split_floats(args, v, 1);
        tlm_period_ms = (n >= 1 && v[0] > 0) ? (uint32_t)v[0] : 0;
        tlm_last_ms   = millis();
        proto_ok(seq, "TLM", String((unsigned long)tlm_period_ms));
        return;
    }
    if (cmd == "EVENTS") {
        int n = split_floats(args, v, 1);
        proto_events_on = (n >= 1) ? (v[0] != 0.0f) : true;
        proto_ok(seq, "EVENTS", proto_events_on ? "1" : "0");
        return;
    }
    if (cmd == "DIST") {
        int16_t d = pp_is_busy() ? oled_dist : vl53l0x_read_mm();
        proto_ok(seq, "DIST", String((int)d));
        return;
    }
    if (cmd == "GRIPQ") {
        bool has = pp_is_busy()
                 ? (oled_dist >= 0 && oled_dist < (int16_t)Z_GRIP_DETECT_MM)
                 : z_grip_has_object();
        proto_ok(seq, "GRIPQ", has ? "1" : "0");
        return;
    }
    if (cmd == "LIMQ") { proto_ok(seq, "LIMQ", z_limit_top_triggered() ? "1" : "0"); return; }
    if (cmd == "PPINFO") {
        char b[96];
        snprintf(b, sizeof(b), "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%u,%u,%u",
                 pp_task.pick_x, pp_task.pick_y, pp_task.pick_z,
                 pp_task.place_x, pp_task.place_y, pp_task.place_z,
                 pp_task.safe_z, pp_task.xy_settle_ms,
                 pp_task.grip_ms, pp_task.drop_ms);
        proto_ok(seq, "PPINFO", b);
        return;
    }

    /* ===== Nhom 2: cau hinh PP (cho phep ngay ca khi dang chay) ===== */
    if (cmd == "PICK") {
        if (split_floats(args, v, 3) == 3) {
            pp_task.pick_x = v[0]; pp_task.pick_y = v[1]; pp_task.pick_z = v[2];
            proto_ok(seq, "PICK");
        } else proto_err(seq, "ARGS", "PICK,x,y,z");
        return;
    }
    if (cmd == "PLACE") {
        if (split_floats(args, v, 3) == 3) {
            pp_task.place_x = v[0]; pp_task.place_y = v[1]; pp_task.place_z = v[2];
            proto_ok(seq, "PLACE");
        } else proto_err(seq, "ARGS", "PLACE,x,y,z");
        return;
    }
    if (cmd == "SAFEZ") {
        if (split_floats(args, v, 1) >= 1) { pp_task.safe_z = v[0]; proto_ok(seq, "SAFEZ"); }
        else proto_err(seq, "ARGS", "SAFEZ,z");
        return;
    }
    if (cmd == "SETTLE") {
        if (split_floats(args, v, 1) >= 1) { pp_task.xy_settle_ms = (uint16_t)v[0]; proto_ok(seq, "SETTLE"); }
        else proto_err(seq, "ARGS", "SETTLE,ms");
        return;
    }
    if (cmd == "GRIPMS") {
        if (split_floats(args, v, 1) >= 1) { pp_task.grip_ms = (uint16_t)v[0]; proto_ok(seq, "GRIPMS"); }
        else proto_err(seq, "ARGS", "GRIPMS,ms");
        return;
    }
    if (cmd == "DROPMS") {
        if (split_floats(args, v, 1) >= 1) { pp_task.drop_ms = (uint16_t)v[0]; proto_ok(seq, "DROPMS"); }
        else proto_err(seq, "ARGS", "DROPMS,ms");
        return;
    }
    if (cmd == "RUN") {
        if (pp_is_busy()) proto_err(seq, "BUSY", pp_state_name());
        else { pp_start(); proto_ok(seq, "RUN"); }
        return;
    }

    /* ===== Nhom 3: homing / teach (chan khi dang ban) ===== */
    if (cmd == "HOMEZ") {
        if (z_home_busy() || pp_is_busy()) { proto_err(seq, "BUSY"); return; }
        bool ok = z_home_instant();
        if (!ok) z_home_slow_start();
        proto_ok(seq, "HOMEZ", ok ? "instant" : "slow");
        return;
    }
    if (cmd == "HOMEZS") {
        if (z_home_busy() || pp_is_busy()) { proto_err(seq, "BUSY"); return; }
        z_home_slow_start(); proto_ok(seq, "HOMEZS"); return;
    }
    if (cmd == "TEACHPICK") {
        if (z_home_busy() || pp_is_busy()) { proto_err(seq, "BUSY"); return; }
        z_teach_start(&pp_task.pick_z); proto_ok(seq, "TEACHPICK"); return;
    }
    if (cmd == "TEACHPLACE") {
        if (z_home_busy() || pp_is_busy()) { proto_err(seq, "BUSY"); return; }
        z_teach_start(&pp_task.place_z); proto_ok(seq, "TEACHPLACE"); return;
    }
    if (cmd == "HOMETOP") {
        if (z_home_busy() || pp_is_busy()) { proto_err(seq, "BUSY"); return; }
        z_home_top_start(); proto_ok(seq, "HOMETOP"); return;
    }

    /* ===== Nhom 4: dieu khien thu cong (chan khi PP dang chay) ===== */
    if (pp_is_busy()) { proto_err(seq, "BUSY", pp_state_name()); return; }

    if (cmd == "MOVE") {
        if (split_floats(args, v, 2) == 2) {
            if (move_to(v[0], v[1])) proto_ok(seq, "MOVE");
            else                     proto_err(seq, "WORKSPACE", "ngoai vung lam viec");
        } else proto_err(seq, "ARGS", "MOVE,x,y");
        return;
    }
    if (cmd == "HOME") { move_to(HOME_X, HOME_Y); proto_ok(seq, "HOME"); return; }
    if (cmd == "DEMO") { cmd_demo();              proto_ok(seq, "DEMO"); return; }

    if (cmd == "Z") {
        if (split_floats(args, v, 1) >= 1) {
            if (z_axis_move_to(v[0])) proto_ok(seq, "Z");
            else                      proto_err(seq, "RANGE", "ngoai hanh trinh");
        } else proto_err(seq, "ARGS", "Z,mm");
        return;
    }
    if (cmd == "ZR") {
        if (split_floats(args, v, 1) >= 1) {
            if (z_axis_move_by(v[0])) proto_ok(seq, "ZR");
            else                      proto_err(seq, "RANGE", "ngoai hanh trinh");
        } else proto_err(seq, "ARGS", "ZR,mm");
        return;
    }
    if (cmd == "ZSTOP") { z_axis_stop();     proto_ok(seq, "ZSTOP"); return; }
    if (cmd == "ZZERO") { z_axis_set_zero(); proto_ok(seq, "ZZERO"); return; }

    if (cmd == "SUCK")  { gripper_suck();    proto_ok(seq, "SUCK"); return; }
    if (cmd == "DROP")  { gripper_release(); proto_ok(seq, "DROP"); return; }
    if (cmd == "PUMP") {
        if (split_floats(args, v, 1) >= 1) {
            if (v[0] != 0.0f) gripper_pump_on(); else gripper_pump_off();
            proto_ok(seq, "PUMP", v[0] != 0.0f ? "1" : "0");
        } else proto_err(seq, "ARGS", "PUMP,0|1");
        return;
    }

    proto_err(seq, "CMD", "khong ro lenh");
}

static void process(const String &cmd)
{
    /* Luon cho phep cac lenh doc-only va abort khi PP/Home dang chay */
    if (cmd == "abort") {
        pp_abort();
        demo_stop();          /* dung demo neu dang chay        */
        xy_moving = false;    /* dung noi suy servo XY ngay      */
        /* Dung home/teach neu dang chay */
        if (z_home_busy()) { z_home_abort(); Serial.println("[HOME] Dung do abort"); }
        return;
    }

    /* --- Homing & Teach --- */
    if (cmd == "homez") {
        if (z_home_busy() || pp_is_busy()) {
            Serial.println("[HOME] ERR: Dang co tac vu khac chay");
        } else {
            Serial.println("[HOME] Instant homing...");
            if (!z_home_instant()) {
                Serial.println("[HOME] Fallback sang slow homing. Dam bao robot o vi tri cao nhat!");
                z_home_slow_start();
            }
        }
        return;
    }
    if (cmd == "homez slow") {
        if (z_home_busy() || pp_is_busy()) {
            Serial.println("[HOME] ERR: Dang co tac vu khac chay");
        } else {
            Serial.println("[HOME] Slow homing – dam bao robot o vi tri CAO NHAT truoc!");
            z_home_slow_start();
        }
        return;
    }
    if (cmd == "hometop") {
        if (z_home_busy() || pp_is_busy()) {
            Serial.println("[HOME] ERR: Dang co tac vu khac chay");
        } else {
            Serial.println("[HOME] Homing ve gioi han TREN (cong tac end-stop)...");
            z_home_top_start();
        }
        return;
    }
    if (cmd == "lim?") {
        int raw = digitalRead(Z_LIMIT_TOP);
        Serial.printf("[LIM] Gioi han tren: %s  (GPIO%d raw=%s, active=%s)\n",
                      z_limit_top_triggered() ? "NHAN" : "tha",
                      Z_LIMIT_TOP, raw ? "HIGH" : "LOW",
                      Z_LIMIT_TOP_ACTIVE_LOW ? "LOW" : "HIGH");
        return;
    }
    if (cmd == "teachpick") {
        if (z_home_busy() || pp_is_busy()) {
            Serial.println("[TEACH] ERR: Dang co tac vu khac chay");
        } else {
            Serial.println("[TEACH] Tim be mat cho pick – dam bao XY dang o vi tri lay hang!");
            z_teach_start(&pp_task.pick_z);
        }
        return;
    }
    if (cmd == "teachplace") {
        if (z_home_busy() || pp_is_busy()) {
            Serial.println("[TEACH] ERR: Dang co tac vu khac chay");
        } else {
            Serial.println("[TEACH] Tim be mat cho place – dam bao XY dang o vi tri dat hang!");
            z_teach_start(&pp_task.place_z);
        }
        return;
    }
    if (cmd == "grip?") {
        bool has = z_grip_has_object();
        int16_t d = vl53l0x_read_mm();
        Serial.printf("[GRIP?] %s  (VL53L0X=%dmm  nguong<%dmm)\n",
                      has ? "CO VAT" : "KHONG CO VAT",
                      d, (int)Z_GRIP_DETECT_MM);
        return;
    }
    if (cmd == "help" || cmd == "?") { cmd_help(); return; }
    if (cmd == "ppinfo") {
        Serial.printf("[PP] Lay : X=%.1f Y=%.1f Z=%.1f\n",
                      pp_task.pick_x,  pp_task.pick_y,  pp_task.pick_z);
        Serial.printf("[PP] Dat : X=%.1f Y=%.1f Z=%.1f\n",
                      pp_task.place_x, pp_task.place_y, pp_task.place_z);
        Serial.printf("[PP] SafeZ=%.1f  settle=%d  grip=%d  drop=%d ms  |  trang thai: %s\n",
                      pp_task.safe_z, pp_task.xy_settle_ms,
                      pp_task.grip_ms, pp_task.drop_ms, pp_state_name());
        return;
    }
    if (cmd == "run") { pp_start(); return; }

    /* Lenh cau hinh PP (co the dat khi dang dung) */
    if (cmd.startsWith("pick ")) {
        float a, b, c;
        if (parse3f(cmd.substring(5), a, b, c)) {
            pp_task.pick_x = a; pp_task.pick_y = b; pp_task.pick_z = c;
            Serial.printf("[PP] Lay: X=%.1f Y=%.1f Z=%.1f\n", a, b, c);
        } else Serial.println("[PP] ERR: dung 'pick x,y,z'");
        return;
    }
    if (cmd.startsWith("place ")) {
        float a, b, c;
        if (parse3f(cmd.substring(6), a, b, c)) {
            pp_task.place_x = a; pp_task.place_y = b; pp_task.place_z = c;
            Serial.printf("[PP] Dat: X=%.1f Y=%.1f Z=%.1f\n", a, b, c);
        } else Serial.println("[PP] ERR: dung 'place x,y,z'");
        return;
    }
    if (cmd.startsWith("safez ")) {
        pp_task.safe_z = cmd.substring(6).toFloat();
        Serial.printf("[PP] SafeZ = %.1f mm\n", pp_task.safe_z);
        return;
    }
    if (cmd.startsWith("settle ")) {
        pp_task.xy_settle_ms = (uint16_t)cmd.substring(7).toInt();
        Serial.printf("[PP] settle = %d ms\n", pp_task.xy_settle_ms);
        return;
    }
    if (cmd.startsWith("grip ")) {
        pp_task.grip_ms = (uint16_t)cmd.substring(5).toInt();
        Serial.printf("[PP] grip = %d ms\n", pp_task.grip_ms);
        return;
    }
    if (cmd.startsWith("drop ")) {
        pp_task.drop_ms = (uint16_t)cmd.substring(5).toInt();
        Serial.printf("[PP] drop = %d ms\n", pp_task.drop_ms);
        return;
    }

    /* Bloc cac lenh dieu khien thu cong khi PP dang chay */
    if (pp_is_busy()) {
        Serial.printf("[PP] Dang chay (%s) – go 'abort' de dung\n", pp_state_name());
        return;
    }

    if (cmd == "home") { move_to(HOME_X, HOME_Y); return; }
    if (cmd == "demo") { cmd_demo();               return; }

    if (cmd == "pos") {
        Serial.printf("[POS] X=%.2f  Y=%.2f  Z=%.2f mm  |  giac hut: %s\n",
                      cur_x, cur_y, z_axis_get_pos_mm(),
                      gripper_is_sucking() ? "HUT" : "NHA");
        return;
    }

    /* --- Lenh Z --- */
    if (cmd == "z0")    { z_axis_set_zero(); return; }
    if (cmd == "zstop") { z_axis_stop();     return; }
    if (cmd == "zpos") {
        Serial.printf("[Z] %.2f mm  |  %ld xung  |  %s  |  lim_top=%s\n",
                      z_axis_get_pos_mm(), (long)z_axis_get_enc_cnt(),
                      z_axis_is_moving() ? "dang di" : "dung",
                      z_limit_top_triggered() ? "NHAN" : "tha");
        return;
    }
    if (cmd == "zenc") {   /* doc encoder tho – debug chuan/chieu */
        Serial.printf("[Z] enc=%ld xung  pos=%.3f mm\n",
                      (long)z_axis_get_enc_cnt(), z_axis_get_pos_mm());
        return;
    }
    if (cmd.startsWith("zraw ")) {   /* chay PWM truc tiep, bo qua PID/gioi han */
        int16_t pwm = (int16_t)cmd.substring(5).toInt();
        z_axis_raw_pwm(pwm);
        Serial.printf("[Z] RAW PWM=%d (go 'zstop' de dung). enc=%ld\n",
                      pwm, (long)z_axis_get_enc_cnt());
        return;
    }
    if (cmd.startsWith("z")) {
        String val = cmd.substring(1);   /* bo chu 'z' */
        if (val.startsWith("+")) {
            z_axis_move_by(val.substring(1).toFloat());
        } else if (val.startsWith("-")) {
            z_axis_move_by(-val.substring(1).toFloat());
        } else if (val.length() > 0) {
            z_axis_move_to(val.toFloat());
        }
        return;
    }

    if (cmd == "suck")     { gripper_suck();     return; }
    if (cmd == "drop")     { gripper_release();  return; }
    if (cmd == "pump on")  { gripper_pump_on();  return; }
    if (cmd == "pump off") { gripper_pump_off(); return; }

    if (cmd == "dist") {
        int16_t d = vl53l0x_read_mm();
        if (d >= 0) Serial.printf("[DIST] %d mm\n", d);
        else        Serial.println("[DIST] Ngoai tam do hoac cam bien loi");
        return;
    }

    /* "x,y" */
    int comma = cmd.indexOf(',');
    if (comma > 0) {
        float x = cmd.substring(0, comma).toFloat();
        float y = cmd.substring(comma + 1).toFloat();
        move_to(x, y);
        return;
    }

    Serial.printf("[ERR] Khong hieu lenh: '%s'  (go 'help')\n", cmd.c_str());
}
/* =================================================================== */
void setup()  /* oled_ms / oled_dist khai bao o dau file */
{
    Serial.begin(115200);

    /* Tat relay bom NGAY khi khoi dong.
       GPIO45 la chan strapping (VDD_SPI) -> mac dinh keo LOW luc boot.
       Relay active-LOW nen bom co the keu ~vai tram ms luc cap nguon
       truoc khi firmware chay. Goi gripper_init() truoc tien de rut ngan
       cua so nay (phan con lai phai xu ly bang phan cung – xem ghi chu).  */
    gripper_init();

    delay(500);

    s1.setPeriodHertz(SERVO_HZ);
    s1.attach(PIN_SERVO1, SERVO_US_MIN, SERVO_US_MAX);

    s2.setPeriodHertz(SERVO_HZ);
    s2.attach(PIN_SERVO2, SERVO_US_MIN, SERVO_US_MAX);

    /* Dat goc khoi dau khop voi cur_a1/cur_a2 = 90 (diem xuat phat noi suy) */
    s1.write(cur_a1);
    s2.write(cur_a2);

    Serial.println("\n=== 5-Bar Parallel Robot (ESP32-S3) ===");
    Serial.printf("L0=%.0f  L1=%.0f L2=%.0f L3=%.0f L4=%.0f mm\n",
                  (float)IK_L0, (float)IK_L1, (float)IK_L2,
                  (float)IK_L3, (float)IK_L4);
    Serial.printf("Home: (%.1f, %.1f) mm\n", (float)HOME_X, (float)HOME_Y);

    vl53l0x_init();   /* VL53L0X tuy chon – chi in canh bao neu khong co */
    z_axis_init();
    oled_init();      /* OLED SSD1306 – dung chung I2C voi VL53L0X       */

    cmd_help();
    move_to(HOME_X, HOME_Y);
}
void loop()
{
    z_axis_update();    /* PID truc Z – phai goi lien tuc   */
    servo_xy_update();  /* Noi suy servo XY dong bo (20ms)  */
    z_home_update();    /* State machine Homing / Teach     */
    demo_update();      /* Demo hinh vuong (non-blocking)   */
    pp_update();        /* State machine Pick & Place        */

    /* Cap nhat OLED moi 300ms.
       Chi doc VL53L0X khi PP dang IDLE – tranh double-read (33ms x2)
       gay block PID khi PP dang van chuyen vat.                       */
    if (millis() - oled_ms >= 300) {
        oled_ms = millis();
        if (!pp_is_busy()) {
            oled_dist = vl53l0x_read_mm();
        }
        oled_update(cur_x, cur_y, z_axis_get_pos_mm(),
                    gripper_is_sucking(), pp_state_name(), oled_dist);
    }

    /* Telemetry streaming '@' (giao thuc) – bat bang lenh "$<seq>,TLM,<ms>" */
    if (tlm_period_ms && (millis() - tlm_last_ms >= tlm_period_ms)) {
        tlm_last_ms = millis();
        proto_send('@', tlm_body());
    }

    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            rx_buf.trim();
            if (rx_buf.length()) {
                if (rx_buf[0] == '$') proto_handle(rx_buf);   /* khung may  */
                else                  process(rx_buf);         /* lenh go tay */
            }
            rx_buf = "";
        } else {
            rx_buf += c;
        }
    }
}
