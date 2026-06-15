#include <Arduino.h>
#include <Wire.h>
#include "vl53l0x.h"

/*
 * Test firmware: doc rieng cam bien khoang cach VL53L0X.
 * Chon env PlatformIO: test_vl53
 *
 * Lenh Serial 115200:
 *   help             In bang lenh
 *   dist/read        Doc 1 lan
 *   avg n            Doc trung binh n mau, vi du: avg 5
 *   stream on/off    Bat/tat stream
 *   rate ms          Doi chu ky stream, vi du: rate 200
 *   scan             Quet thiet bi I2C
 *   init             Khoi tao lai voi pin mac dinh
 *   init sda,scl     Khoi tao lai voi pin tuy chon
 */

static String rx_buf;
static bool sensor_ready = false;
static bool stream_on = true;
static uint32_t stream_ms = 300;
static uint32_t last_stream_ms = 0;

static bool parse2i(const String &s, int &a, int &b)
{
    int comma = s.indexOf(',');
    if (comma < 0) return false;

    String left = s.substring(0, comma);
    String right = s.substring(comma + 1);
    left.trim();
    right.trim();
    if (left.length() == 0 || right.length() == 0) return false;

    a = left.toInt();
    b = right.toInt();
    return true;
}

static void i2c_scan()
{
    uint8_t count = 0;

    Serial.println("[I2C] Scan start");
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        uint8_t err = Wire.endTransmission();
        if (err == 0) {
            Serial.printf("[I2C] Found 0x%02X", addr);
            if (addr == 0x29) Serial.print(" (VL53L0X)");
            if (addr == 0x3C || addr == 0x3D) Serial.print(" (OLED?)");
            Serial.println();
            count++;
        }
    }

    if (count == 0) {
        Serial.println("[I2C] Khong thay thiet bi nao");
    } else {
        Serial.printf("[I2C] Total: %u device(s)\n", count);
    }
}

static void read_once()
{
    int16_t d = vl53l0x_read_mm();
    if (d >= 0) {
        Serial.printf("[DIST] %d mm\n", d);
    } else if (!sensor_ready) {
        Serial.println("[DIST] Loi: cam bien chua init OK");
    } else {
        Serial.println("[DIST] -1 (ngoai tam do hoac loi doc)");
    }
}

static void read_average(uint8_t n)
{
    if (n < 1) n = 1;
    if (n > 20) n = 20;

    uint32_t t0 = millis();
    int16_t d = vl53l0x_read_avg(n);
    uint32_t dt = millis() - t0;

    if (d >= 0) {
        Serial.printf("[AVG] n=%u -> %d mm (%lums)\n", n, d, (unsigned long)dt);
    } else if (!sensor_ready) {
        Serial.println("[AVG] Loi: cam bien chua init OK");
    } else {
        Serial.printf("[AVG] n=%u -> -1 (khong co mau hop le, %lums)\n",
                      n, (unsigned long)dt);
    }
}

static void print_help()
{
    Serial.println();
    Serial.println("=== TEST VL53L0X DISTANCE SENSOR ===");
    Serial.printf("  Pins mac dinh: SDA=GPIO%d SCL=GPIO%d I2C=%lu Hz\n",
                  VL53L0X_SDA_PIN, VL53L0X_SCL_PIN,
                  (unsigned long)VL53L0X_I2C_FREQ);
    Serial.println("  help           In bang lenh");
    Serial.println("  dist/read      Doc 1 lan");
    Serial.println("  avg n          Doc trung binh n mau");
    Serial.println("  stream on/off  Bat/tat in lien tuc");
    Serial.println("  rate ms        Doi chu ky stream");
    Serial.println("  scan           Quet I2C");
    Serial.println("  init           Init lai pin mac dinh");
    Serial.println("  init sda,scl   Init lai pin tuy chon");
    Serial.println();
}

static void init_sensor(uint8_t sda = VL53L0X_SDA_PIN,
                        uint8_t scl = VL53L0X_SCL_PIN)
{
    sensor_ready = vl53l0x_init(sda, scl, VL53L0X_I2C_FREQ);
    Serial.printf("[INIT] VL53L0X %s\n", sensor_ready ? "OK" : "FAIL");
    if (!sensor_ready) {
        i2c_scan();
    }
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
    if (cmd == "dist" || cmd == "read") {
        read_once();
        return;
    }
    if (cmd == "scan") {
        i2c_scan();
        return;
    }
    if (cmd.startsWith("avg ")) {
        int n = cmd.substring(4).toInt();
        read_average((uint8_t)n);
        return;
    }
    if (cmd == "stream on") {
        stream_on = true;
        Serial.println("[STREAM] ON");
        return;
    }
    if (cmd == "stream off") {
        stream_on = false;
        Serial.println("[STREAM] OFF");
        return;
    }
    if (cmd.startsWith("rate ")) {
        uint32_t v = (uint32_t)cmd.substring(5).toInt();
        if (v < 50) v = 50;
        stream_ms = v;
        Serial.printf("[STREAM] rate=%lums\n", (unsigned long)stream_ms);
        return;
    }
    if (cmd == "init") {
        init_sensor();
        return;
    }
    if (cmd.startsWith("init ")) {
        int sda = 0;
        int scl = 0;
        if (!parse2i(cmd.substring(5), sda, scl)) {
            Serial.println("[ERR] Dung: init sda,scl");
            return;
        }
        init_sensor((uint8_t)sda, (uint8_t)scl);
        return;
    }

    Serial.printf("[ERR] Khong hieu lenh: %s\n", cmd.c_str());
}

void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("=== VL53L0X DEBUG ===");
    init_sensor();
    print_help();
}

void loop()
{
    if (stream_on && (millis() - last_stream_ms >= stream_ms)) {
        last_stream_ms = millis();
        read_once();
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
