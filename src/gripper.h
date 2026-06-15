#pragma once
#include <Arduino.h>

/* =====================================================================
   Module dieu khien giac hut chan khong
   - PIN_SUCTION    : dieu khien van / dau hut  (HIGH = hut, LOW = nha)
   - PIN_PUMP_RELAY : relay bat/tat bom chan khong
   ===================================================================== */

#define PIN_SUCTION      42   /* I/O dieu khien van hut/day             */
#define PIN_PUMP_RELAY   45   /* I/O dieu khien relay bom (strapping!)  */

/* true  : relay kich hoat muc HIGH (relay active-high)
   false : relay kich hoat muc LOW  (relay active-low, pho bien hon)   */
#define PUMP_RELAY_ACTIVE_HIGH  true

/* Delay xa ap am (ms) – dung bom sau khi da mo van de loc khi vao */
#define GRIPPER_RELEASE_PUMP_DELAY_MS  150

/* Khoi tao chan. Goi trong setup() truoc khi dung cac ham khac. */
void gripper_init();

/* Bat bom + kich hoat giac hut. */
void gripper_suck();

/* Nha vat (BLOCKING ~150ms): tat van roi tat bom sau delay.
   Dung cho lenh thu cong – KHONG goi tu state machine. */
void gripper_release();

/* Mo van tuc thi (non-blocking). Bom VAN DANG CHAY.
   Sau GRIPPER_RELEASE_PUMP_DELAY_MS, goi gripper_pump_off().
   Dung cho state machine PP de tranh block z_axis_update(). */
void gripper_valve_open();

/* Dieu khien rieng relay bom (khong anh huong trang thai van). */
void gripper_pump_on();
void gripper_pump_off();

/* Tra ve true neu dang o trang thai hut. */
bool gripper_is_sucking();
