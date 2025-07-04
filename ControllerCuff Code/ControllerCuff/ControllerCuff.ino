/*********************************************************************
 * ControllerCuff   –  Two Wii-extension ports (via PCA9548A) → one
 * LUFA fight-stick.  Supports Classic / NES-Classic pad / Nunchuk.
 *
 * • Digital inputs OR’ed, analog sticks averaged.
 * • No mode switching; everything always live.
 *
 * Requires:
 *   - David Madison’s “NintendoExtensionCtrl”  (>=0.8.3)
 *   - Arduino-LUFA core (Pro Micro, Leonardo, etc.)
 *********************************************************************/

#include "LUFAConfig.h"
#include <LUFA.h>
#include "Joystick.h"  // must define global ReportData

#include <Wire.h>
#include <NintendoExtensionCtrl.h>

struct PadState;

/* ------------------------------------------------------------------
 * Button / DPAD masks (match your original fight-stick descriptor)
 * -----------------------------------------------------------------*/
#define DPAD_UP_MASK_ON 0x00
#define DPAD_UPRIGHT_MASK_ON 0x01
#define DPAD_RIGHT_MASK_ON 0x02
#define DPAD_DOWNRIGHT_MASK_ON 0x03
#define DPAD_DOWN_MASK_ON 0x04
#define DPAD_DOWNLEFT_MASK_ON 0x05
#define DPAD_LEFT_MASK_ON 0x06
#define DPAD_UPLEFT_MASK_ON 0x07
#define DPAD_NOTHING_MASK_ON 0x08

#define A_MASK_ON 0x04
#define B_MASK_ON 0x02
#define X_MASK_ON 0x08
#define Y_MASK_ON 0x01
#define LB_MASK_ON 0x10
#define RB_MASK_ON 0x20
#define ZL_MASK_ON 0x40
#define ZR_MASK_ON 0x80
#define START_MASK_ON 0x200
#define SELECT_MASK_ON 0x100
#define HOME_MASK_ON 0x1000

/* Helper for hat switch */
#define H(n) DPAD_##n##_MASK_ON

/* ------------------------------------------------------------------
 * I²C multiplexer
 * -----------------------------------------------------------------*/
#define MUX_ADDR 0x70
static void selectMux(uint8_t ch) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << ch);  // enable channel ch
  Wire.endTransmission();
}

/* ------------------------------------------------------------------
 * One neutral container per port
 * -----------------------------------------------------------------*/
struct PadState {
  /* digital */
  bool up = 0, down = 0, left = 0, right = 0;
  bool a = 0, b = 0, x = 0, y = 0;
  bool lb = 0, rb = 0, zl = 0, zr = 0;
  bool start = 0, select = 0, home = 0;
  /* analog (0-255, centred 128) */
  uint8_t lx = 128, ly = 128, rx = 128, ry = 128;
};

/* ------------------------------------------------------------------
 * Two ExtensionPort objects + shared controller wrappers
 * -----------------------------------------------------------------*/
#define PORTS 2
ExtensionPort port[PORTS] = { ExtensionPort(), ExtensionPort() };
ClassicController::Shared cc[PORTS] = { { port[0] }, { port[1] } };
Nunchuk::Shared nc[PORTS] = { { port[0] }, { port[1] } };

/* ------------------------------------------------------------------
 * readPort(idx)  → PadState
 * -----------------------------------------------------------------*/
static PadState readPort(uint8_t idx) {
  PadState s;  // defaults = neutral

  selectMux(idx);
  if (!port[idx].update()) return s;  // no fresh data

  switch (port[idx].getControllerType()) {

    case ExtensionType::ClassicController:
      {  // includes NES pad
        auto &c = cc[idx];
        s.up = c.dpadUp();
        s.down = c.dpadDown();
        s.left = c.dpadLeft();
        s.right = c.dpadRight();

        s.a = c.buttonA();
        s.b = c.buttonB();
        s.x = c.buttonX();
        s.y = c.buttonY();

        s.lb = c.buttonL();
        s.rb = c.buttonR();
        s.zl = c.buttonZL();
        s.zr = c.buttonZR();

        s.start = c.buttonPlus();
        s.select = c.buttonMinus();
        s.home = c.buttonHome();

        s.lx = c.leftJoyX();
        s.ly = 255 - c.leftJoyY();
        s.rx = c.rightJoyX();
        s.ry = 255 - c.rightJoyY();
      }
      break;

    case ExtensionType::Nunchuk:
      {
        auto &n = nc[idx];
        /* Map nunchuk: stick → LX/LY,  C→A , Z→B */
        s.a = n.buttonC();
        s.b = n.buttonZ();
        s.lx = n.joyX();
        s.ly = 255 - n.joyY();
      }
      break;

    default: break; /* NoController / Unknown → neutral */
  }
  return s;
}

/* ------------------------------------------------------------------
 * merge two PadStates
 * -----------------------------------------------------------------*/
static PadState merge(const PadState &a, const PadState &b) {
  PadState o;
  /* OR digital --------------------------------------------------- */
  o.up = a.up || b.up;
  o.down = a.down || b.down;
  o.left = a.left || b.left;
  o.right = a.right || b.right;

  o.a = a.a || b.a;
  o.b = a.b || b.b;
  o.x = a.x || b.x;
  o.y = a.y || b.y;
  o.lb = a.lb || b.lb;
  o.rb = a.rb || b.rb;
  o.zl = a.zl || b.zl;
  o.zr = a.zr || b.zr;

  o.start = a.start || b.start;
  o.select = a.select || b.select;
  o.home = a.home || b.home;

  /* average analog ----------------------------------------------- */
  o.lx = uint8_t(((uint16_t)a.lx + b.lx) >> 1);
  o.ly = uint8_t(((uint16_t)a.ly + b.ly) >> 1);
  o.rx = uint8_t(((uint16_t)a.rx + b.rx) >> 1);
  o.ry = uint8_t(((uint16_t)a.ry + b.ry) >> 1);

  return o;
}

/* ------------------------------------------------------------------
 * pad → LUFA ReportData
 * -----------------------------------------------------------------*/
extern USB_JoystickReport_Input_t ReportData;

static void padToReport(const PadState &p) {
  memset(&ReportData, 0, sizeof(ReportData));

  /* Hat (DPAD) ---------------------------------------------------- */
  if (p.up && p.right) ReportData.HAT = H(UPRIGHT);
  else if (p.down && p.right) ReportData.HAT = H(DOWNRIGHT);
  else if (p.down && p.left) ReportData.HAT = H(DOWNLEFT);
  else if (p.up && p.left) ReportData.HAT = H(UPLEFT);
  else if (p.up) ReportData.HAT = H(UP);
  else if (p.down) ReportData.HAT = H(DOWN);
  else if (p.left) ReportData.HAT = H(LEFT);
  else if (p.right) ReportData.HAT = H(RIGHT);
  else ReportData.HAT = H(NOTHING);

  /* sticks -------------------------------------------------------- */
  ReportData.LX = p.lx;
  ReportData.LY = p.ly;
  ReportData.RX = p.rx;
  ReportData.RY = p.ry;

  /* buttons ------------------------------------------------------- */
  if (p.a) ReportData.Button |= A_MASK_ON;
  if (p.b) ReportData.Button |= B_MASK_ON;
  if (p.x) ReportData.Button |= X_MASK_ON;
  if (p.y) ReportData.Button |= Y_MASK_ON;
  if (p.lb) ReportData.Button |= LB_MASK_ON;
  if (p.rb) ReportData.Button |= RB_MASK_ON;
  if (p.zl) ReportData.Button |= ZL_MASK_ON;
  if (p.zr) ReportData.Button |= ZR_MASK_ON;
  if (p.start) ReportData.Button |= START_MASK_ON;
  if (p.select) ReportData.Button |= SELECT_MASK_ON;
  if (p.home) ReportData.Button |= HOME_MASK_ON;
}

/* ------------------------------------------------------------------
 * Arduino setup / loop
 * -----------------------------------------------------------------*/
void setup() {
  Wire.begin();
  delay(50);

  /* initialise each port once ------------------------------------ */
  for (uint8_t ch = 0; ch < PORTS; ++ch) {
    selectMux(ch);
    port[ch].begin();
    port[ch].connect();  // non-blocking handshake
  }

  SetupHardware();
  GlobalInterruptEnable();
}

void loop() {
  /* read both ports */
  PadState A = readPort(0);
  PadState B = readPort(1);

  /* merge & push to HID */
  padToReport(merge(A, B));

  HID_Task();
  USB_USBTask();
}
