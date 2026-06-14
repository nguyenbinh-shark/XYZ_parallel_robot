#include "oled.h"
#include "vl53l0x.h"          /* lay dinh nghia SDA/SCL pin */
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

static Adafruit_SSD1306 _disp(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
static bool _ready = false;

void oled_init()
{
    /* Wire da duoc khoi tao boi vl53l0x_init() – goi them lan nua an toan */
    Wire.begin((int)VL53L0X_SDA_PIN, (int)VL53L0X_SCL_PIN,
               (uint32_t)VL53L0X_I2C_FREQ);

    if (!_disp.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
        Serial.printf("[OLED] ERR: Khong tim thay man hinh (addr=0x%02X)\n",
                      OLED_I2C_ADDR);
        return;
    }

    _disp.clearDisplay();
    _disp.setTextColor(SSD1306_WHITE);
    _disp.setTextSize(1);
    _disp.setCursor(14, 20);
    _disp.print("5-Bar Robot v1.0");
    _disp.setCursor(22, 34);
    _disp.print("ESP32-S3  115200");
    _disp.display();
    _ready = true;

    Serial.printf("[OLED] OK  addr=0x%02X  %dx%d\n",
                  OLED_I2C_ADDR, OLED_WIDTH, OLED_HEIGHT);
}

/* ----------------------------------------------------------------
   Layout 128x64 – font 6x8 (size=1)
   ----------------------------------------------------------------
   y= 0  "= 5-Bar Robot      ="   title
   y=12  "XY:+19.0  +45.0 mm "   vi tri XY
   y=22  "Z : +12.34 mm [DUNG]"   vi tri Z + trang thai
   y=34  "Grip: HUT  VL: 142mm"   giac hut + cam bien
   y=46  "PP : PICK_LOWER      "  state PP
   y=57  divider line
   ---------------------------------------------------------------- */
void oled_update(float x, float y, float z,
                 bool sucking, const char *pp_state, int16_t dist_mm)
{
    if (!_ready) return;

    char buf[22];   /* max 21 ky tu / hang, +null */

    _disp.clearDisplay();
    _disp.setTextSize(1);
    _disp.setTextColor(SSD1306_WHITE);

    /* --- Title --- */
    _disp.setCursor(4, 0);
    _disp.print("=  5-Bar Robot v1  =");

    /* --- XY position --- */
    _disp.setCursor(0, 12);
    snprintf(buf, sizeof(buf), "XY:%+6.1f %+6.1f mm", x, y);
    _disp.print(buf);

    /* --- Z position --- */
    _disp.setCursor(0, 22);
    snprintf(buf, sizeof(buf), "Z :%+7.2f mm  %-4s",
             z, z > 0.05f ? "    " : "ZERO");
    _disp.print(buf);

    /* --- Gripper + VL53L0X --- */
    _disp.setCursor(0, 34);
    if (dist_mm >= 0)
        snprintf(buf, sizeof(buf), "Grip:%-3s   VL:%4dmm",
                 sucking ? "HUT" : "NHA", dist_mm);
    else
        snprintf(buf, sizeof(buf), "Grip:%-3s   VL: ---mm",
                 sucking ? "HUT" : "NHA");
    _disp.print(buf);

    /* --- PP State --- */
    _disp.setCursor(0, 46);
    snprintf(buf, sizeof(buf), "PP : %-15s", pp_state);
    _disp.print(buf);

    /* --- Divider --- */
    _disp.drawFastHLine(0, 56, OLED_WIDTH, SSD1306_WHITE);

    _disp.display();
}
