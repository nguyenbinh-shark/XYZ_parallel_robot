#include <Arduino.h>
#include <ESP32Servo.h>
#include "ik_5bar.h"

/*
 * Test firmware: dieu khien rieng XY cua co cau 5-bar bang 2 servo.
 * Chon env PlatformIO: test_xy_servo
 *
 * Lenh Serial 115200:
 *   help              In bang lenh
 *   home              Ve HOME_X, HOME_Y
 *   xy x,y            Di den toa do XY, vi du: xy 27.5,180
 *   xy x y            Tuong tu, vi du: xy 27.5 180
 *   x,y               Di nhanh den toa do XY, vi du: 27.5,180
 *   move x,y          Alias cua xy
 *   raw a1,a2         Ghi truc tiep goc servo vat ly 0..180 do
 *   offset o1,o2      Doi offset servo cho cac lenh XY tiep theo
 *   invert 0|1        Dao chieu map servo 2
 *   clamp on/off      Ep goc ngoai gioi han ve 0..180 khi debug
 *   speed dps         Doi toc do noi suy, vi du: speed 120
 *   demo              Chay hinh vuong quanh HOME
 *   stop              Dung noi suy/demo tai goc hien tai
 *   pos               In trang thai hien tai
 *   on/off            Attach/detach servo
 */

#define PIN_SERVO1      47
#define PIN_SERVO2      48

#define SERVO_HZ        50
#define SERVO_US_MIN    500
#define SERVO_US_MAX    2500
#define SERVO_XY_INTERVAL_MS 20

#define HOME_X  (IK_L0 * 0.5f)
#define HOME_Y  45.0f

static Servo servo1;
static Servo servo2;

static float servo1_offset = 45.0f;
static float servo2_offset = -45.0f;
static bool  servo2_invert = false;
static bool  servo_clamp = false;
static float servo_speed_dps = 150.0f;

static bool servos_attached = false;
static String rx_buf;

static float cur_x = HOME_X;
static float cur_y = HOME_Y;
static int cur_a1 = 90;
static int cur_a2 = 90;

static int a1_start = 90;
static int a2_start = 90;
static int a1_goal = 90;
static int a2_goal = 90;
static uint32_t xy_t_start = 0;
static uint32_t xy_T_total = 0;
static uint32_t xy_tick_ms = 0;
static bool xy_moving = false;

static const float demo_pts[][2] = {
    { HOME_X - 10.0f, HOME_Y - 10.0f },
    { HOME_X + 10.0f, HOME_Y - 10.0f },
    { HOME_X + 10.0f, HOME_Y + 10.0f },
    { HOME_X - 10.0f, HOME_Y + 10.0f },
    { HOME_X,         HOME_Y         },
};

static int demo_idx = -1;
static uint32_t demo_ms = 0;
static const uint32_t DEMO_DWELL_MS = 350;

static bool parse2f(const String &s, float &a, float &b)
{
    int comma = s.indexOf(',');
    if (comma < 0) return false;

    String left = s.substring(0, comma);
    String right = s.substring(comma + 1);
    left.trim();
    right.trim();
    if (left.length() == 0 || right.length() == 0) return false;

    a = left.toFloat();
    b = right.toFloat();
    return true;
}

static bool parse2nums(String s, float &a, float &b)
{
    s.trim();
    if (parse2f(s, a, b)) return true;

    int sep = -1;
    for (int i = 0; i < s.length(); i++) {
        char c = s[i];
        if (c == ' ' || c == '\t') {
            sep = i;
            break;
        }
    }
    if (sep < 0) return false;

    int next = sep + 1;
    while (next < s.length() && (s[next] == ' ' || s[next] == '\t')) {
        next++;
    }
    if (next >= s.length()) return false;

    String left = s.substring(0, sep);
    String right = s.substring(next);
    left.trim();
    right.trim();
    if (left.length() == 0 || right.length() == 0) return false;

    a = left.toFloat();
    b = right.toFloat();
    return true;
}

static float map_servo1(float theta)
{
    return theta + servo1_offset;
}

static float map_servo2(float theta)
{
    return servo2_invert ? (180.0f - theta + servo2_offset)
                         : (theta + servo2_offset);
}

static float clamp_servo_angle(float a)
{
    if (a < 0.0f) return 0.0f;
    if (a > 180.0f) return 180.0f;
    return a;
}

static void attach_servos()
{
    if (servos_attached) return;

    servo1.setPeriodHertz(SERVO_HZ);
    servo1.attach(PIN_SERVO1, SERVO_US_MIN, SERVO_US_MAX);
    servo2.setPeriodHertz(SERVO_HZ);
    servo2.attach(PIN_SERVO2, SERVO_US_MIN, SERVO_US_MAX);

    servo1.write(cur_a1);
    servo2.write(cur_a2);
    servos_attached = true;
    Serial.printf("[SERVO] Attached GPIO%d=%d deg, GPIO%d=%d deg\n",
                  PIN_SERVO1, cur_a1, PIN_SERVO2, cur_a2);
}

static void detach_servos()
{
    xy_moving = false;
    demo_idx = -1;
    if (!servos_attached) return;

    servo1.detach();
    servo2.detach();
    servos_attached = false;
    Serial.println("[SERVO] Detached");
}

static void write_servo_angles(int a1, int a2)
{
    cur_a1 = constrain(a1, 0, 180);
    cur_a2 = constrain(a2, 0, 180);
    if (!servos_attached) attach_servos();
    servo1.write(cur_a1);
    servo2.write(cur_a2);
}

static void print_status()
{
    Serial.printf("[POS] X=%.2f Y=%.2f | servo1=%d servo2=%d | moving=%d | offset=%.1f,%.1f invert2=%d clamp=%d speed=%.1f\n",
                  cur_x, cur_y, cur_a1, cur_a2, xy_moving ? 1 : 0,
                  servo1_offset, servo2_offset, servo2_invert ? 1 : 0,
                  servo_clamp ? 1 : 0, servo_speed_dps);
}

static void print_ik_debug(float x, float y)
{
    float left_r = sqrtf(x * x + y * y);
    float right_dx = x - IK_L0;
    float right_r = sqrtf(right_dx * right_dx + y * y);
    float left_min = fabsf(IK_L2 - IK_L1);
    float left_max = IK_L1 + IK_L2;
    float right_min = fabsf(IK_L3 - IK_L4);
    float right_max = IK_L3 + IK_L4;

    Serial.printf("[IK] Left r=%.2f mm  hop le %.2f..%.2f mm\n",
                  left_r, left_min, left_max);
    Serial.printf("[IK] Right r=%.2f mm hop le %.2f..%.2f mm\n",
                  right_r, right_min, right_max);
    Serial.printf("[IK] Link: L0=%.1f L1=%.1f L2=%.1f L3=%.1f L4=%.1f\n",
                  IK_L0, IK_L1, IK_L2, IK_L3, IK_L4);
}

static bool move_to(float x, float y)
{
    float t1 = 0.0f;
    float t2 = 0.0f;
    if (!ik_5bar(x, y, &t1, &t2)) {
        Serial.printf("[ERR] IK vo nghiem: X=%.2f Y=%.2f\n", x, y);
        print_ik_debug(x, y);
        return false;
    }

    float s1 = map_servo1(t1);
    float s2 = map_servo2(t2);
    if (s1 < 0.0f || s1 > 180.0f || s2 < 0.0f || s2 > 180.0f) {
        Serial.printf("[ERR] Goc servo vuot gioi han: theta1=%.1f theta2=%.1f -> servo1_calc=%.1f servo2_calc=%.1f (limit 0..180)\n",
                      t1, t2, s1, s2);
        if (!servo_clamp) {
            Serial.println("[HINT] Dung 'clamp on' neu muon ep goc ve bien de debug.");
            return false;
        }
        s1 = clamp_servo_angle(s1);
        s2 = clamp_servo_angle(s2);
        Serial.printf("[CLAMP] servo1=%.1f servo2=%.1f\n", s1, s2);
    }

    if (!servos_attached) attach_servos();

    a1_goal = (int)roundf(s1);
    a2_goal = (int)roundf(s2);
    a1_start = cur_a1;
    a2_start = cur_a2;

    int max_delta = max(abs(a1_goal - a1_start), abs(a2_goal - a2_start));
    xy_T_total = (uint32_t)((float)max_delta * 1000.0f / servo_speed_dps);
    if (xy_T_total < (uint32_t)SERVO_XY_INTERVAL_MS) {
        xy_T_total = SERVO_XY_INTERVAL_MS;
    }

    cur_x = x;
    cur_y = y;
    xy_t_start = millis();
    xy_tick_ms = 0;
    xy_moving = true;

    Serial.printf("[MOVE] X=%.1f Y=%.1f | theta1=%.1f theta2=%.1f | servo1=%d servo2=%d | T=%lums\n",
                  x, y, t1, t2, a1_goal, a2_goal,
                  (unsigned long)xy_T_total);
    return true;
}

static void servo_xy_update()
{
    if (!xy_moving) return;
    if (millis() - xy_tick_ms < SERVO_XY_INTERVAL_MS) return;
    xy_tick_ms = millis();

    float u = (float)(millis() - xy_t_start) / (float)xy_T_total;
    if (u >= 1.0f) u = 1.0f;

    int next_a1 = a1_start + (int)roundf((float)(a1_goal - a1_start) * u);
    int next_a2 = a2_start + (int)roundf((float)(a2_goal - a2_start) * u);
    write_servo_angles(next_a1, next_a2);

    if (u >= 1.0f) {
        xy_moving = false;
        Serial.printf("[DONE] servo1=%d servo2=%d\n", cur_a1, cur_a2);
    }
}

static void demo_start()
{
    demo_idx = 0;
    demo_ms = millis();
    Serial.println("[DEMO] Square 20x20 mm quanh HOME");
    move_to(demo_pts[demo_idx][0], demo_pts[demo_idx][1]);
}

static void demo_update()
{
    if (demo_idx < 0) return;
    if (xy_moving) return;
    if (millis() - demo_ms < DEMO_DWELL_MS) return;

    demo_idx++;
    if ((size_t)demo_idx >= (sizeof(demo_pts) / sizeof(demo_pts[0]))) {
        demo_idx = -1;
        Serial.println("[DEMO] Xong");
        return;
    }

    demo_ms = millis();
    move_to(demo_pts[demo_idx][0], demo_pts[demo_idx][1]);
}

static void print_help()
{
    Serial.println();
    Serial.println("=== TEST XY SERVO 5-BAR ===");
    Serial.println("  help          In bang lenh");
    Serial.println("  home          Ve toa do HOME");
    Serial.println("  xy x,y        Di den XY, vi du: xy 27.5,180");
    Serial.println("  xy x y        Di den XY, vi du: xy 27.5 180");
    Serial.println("  x,y           Di nhanh den XY, vi du: 27.5,180");
    Serial.println("  move x,y      Alias cua xy");
    Serial.println("  raw a1,a2     Ghi truc tiep goc servo vat ly");
    Serial.println("  offset o1,o2  Doi offset servo cho IK");
    Serial.println("  invert 0|1    Dao chieu map servo 2");
    Serial.println("  clamp on/off  Ep goc ngoai 0..180 ve bien khi debug");
    Serial.println("  speed dps     Toc do noi suy servo");
    Serial.println("  demo          Chay hinh vuong debug");
    Serial.println("  stop          Dung demo/noi suy");
    Serial.println("  pos           In trang thai");
    Serial.println("  on/off        Attach/detach servo");
    Serial.println();
}

static void process_command(String cmd)
{
    cmd.trim();
    if (cmd.length() == 0) return;
    cmd.toLowerCase();

    if (cmd == "help" || cmd == "?") {
        print_help();
        return;
    }
    if (cmd == "home") {
        demo_idx = -1;
        move_to(HOME_X, HOME_Y);
        return;
    }
    if (cmd == "pos") {
        print_status();
        return;
    }
    if (cmd == "on") {
        attach_servos();
        return;
    }
    if (cmd == "off") {
        detach_servos();
        return;
    }
    if (cmd == "stop") {
        xy_moving = false;
        demo_idx = -1;
        Serial.println("[STOP] Dung noi suy/demo");
        return;
    }
    if (cmd == "demo") {
        demo_start();
        return;
    }
    if (cmd.startsWith("speed ")) {
        float v = cmd.substring(6).toFloat();
        if (v < 1.0f) {
            Serial.println("[ERR] speed phai >= 1 dps");
            return;
        }
        servo_speed_dps = v;
        Serial.printf("[CFG] speed=%.1f dps\n", servo_speed_dps);
        return;
    }
    if (cmd.startsWith("invert ")) {
        servo2_invert = (cmd.substring(7).toInt() != 0);
        Serial.printf("[CFG] invert2=%d\n", servo2_invert ? 1 : 0);
        return;
    }
    if (cmd == "clamp on") {
        servo_clamp = true;
        Serial.println("[CFG] clamp=1");
        return;
    }
    if (cmd == "clamp off") {
        servo_clamp = false;
        Serial.println("[CFG] clamp=0");
        return;
    }

    float a = 0.0f;
    float b = 0.0f;
    if (cmd.startsWith("offset ")) {
        if (!parse2f(cmd.substring(7), a, b)) {
            Serial.println("[ERR] Dung: offset o1,o2");
            return;
        }
        servo1_offset = a;
        servo2_offset = b;
        Serial.printf("[CFG] offset1=%.1f offset2=%.1f\n", servo1_offset, servo2_offset);
        return;
    }
    if (cmd.startsWith("raw ")) {
        if (!parse2f(cmd.substring(4), a, b)) {
            Serial.println("[ERR] Dung: raw a1,a2");
            return;
        }
        xy_moving = false;
        demo_idx = -1;
        write_servo_angles((int)roundf(a), (int)roundf(b));
        Serial.printf("[RAW] servo1=%d servo2=%d\n", cur_a1, cur_a2);
        return;
    }
    if (cmd.startsWith("xy ")) {
        if (!parse2nums(cmd.substring(3), a, b)) {
            Serial.println("[ERR] Dung: xy x,y  hoac  xy x y");
            return;
        }
        demo_idx = -1;
        move_to(a, b);
        return;
    }
    if (cmd.startsWith("move ")) {
        if (!parse2nums(cmd.substring(5), a, b)) {
            Serial.println("[ERR] Dung: move x,y  hoac  move x y");
            return;
        }
        demo_idx = -1;
        move_to(a, b);
        return;
    }
    if (parse2f(cmd, a, b)) {
        demo_idx = -1;
        move_to(a, b);
        return;
    }

    Serial.printf("[ERR] Khong hieu lenh: %s\n", cmd.c_str());
}

void setup()
{
    Serial.begin(115200);
    delay(500);

    attach_servos();
    print_help();
    move_to(HOME_X, HOME_Y);
}

void loop()
{
    servo_xy_update();
    demo_update();

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
