#include "gripper.h"

static bool _sucking = false;

/* --- Noi bo: dat relay bom theo logic kich hoat da cau hinh --- */
static inline void pump_set(bool on)
{
    bool level = PUMP_RELAY_ACTIVE_HIGH ? on : !on;
    digitalWrite(PIN_PUMP_RELAY, level ? HIGH : LOW);
}

void gripper_init()
{
    pinMode(PIN_SUCTION,    OUTPUT);
    pinMode(PIN_PUMP_RELAY, OUTPUT);

    digitalWrite(PIN_SUCTION, LOW);
    pump_set(false);
    _sucking = false;

    Serial.printf("[GRIP] Init OK  suction=GPIO%d  relay=GPIO%d\n",
                  PIN_SUCTION, PIN_PUMP_RELAY);
}

void gripper_suck()
{
    pump_set(true);
    digitalWrite(PIN_SUCTION, HIGH);
    _sucking = true;
    Serial.println("[GRIP] HUT");
}

void gripper_release()
{
    digitalWrite(PIN_SUCTION, LOW);
    delay(GRIPPER_RELEASE_PUMP_DELAY_MS);
    pump_set(false);
    _sucking = false;
    Serial.println("[GRIP] NHA");
}

void gripper_valve_open()
{
    digitalWrite(PIN_SUCTION, LOW);
    _sucking = false;
    /* Bom van chay – caller phai goi gripper_pump_off()
       sau GRIPPER_RELEASE_PUMP_DELAY_MS ms                          */
    Serial.println("[GRIP] Van mo (bom van chay)");
}

void gripper_pump_on()
{
    pump_set(true);
    Serial.println("[GRIP] BOM ON");
}

void gripper_pump_off()
{
    pump_set(false);
    Serial.println("[GRIP] BOM OFF");
}

bool gripper_is_sucking()
{
    return _sucking;
}
