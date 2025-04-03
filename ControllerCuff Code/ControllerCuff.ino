/***********************************************************************
 * Two Wii Classic Controllers via Multiplexer + NintendoExtensionCtrl
 * Merged into One LUFA Fight Stick Output (No Mode Switching)
 ***********************************************************************/

#include "LUFAConfig.h"  // Your LUFA config
#include <LUFA.h>        // LUFA core
#include "Joystick.h"    // Where you define/declare ReportData, etc.

#include <Wire.h>                   // For I2C
#include <NintendoExtensionCtrl.h>  // The library by dmadison

/***********************************************************************
 * Multiplexer & Wii Addresses
 ***********************************************************************/
#define MUX_ADDR 0x70      // PCA9548A I2C address
#define WII_ADDR 0x52      // Standard Wii extension address
#define NUM_CONTROLLERS 2  // We have two Wii Classic Controllers

/***********************************************************************
 * Data Structure: WiiClassicState
 * We'll store the relevant data from each controller in a simple struct.
 ***********************************************************************/
typedef struct {
  // Digital
  bool dpadUp;
  bool dpadDown;
  bool dpadLeft;
  bool dpadRight;
  bool buttonA;
  bool buttonB;
  bool buttonX;
  bool buttonY;
  bool buttonL;
  bool buttonR;
  bool buttonZL;
  bool buttonZR;
  bool buttonHome;
  bool buttonStart;
  bool buttonSelect;

  // Analog Sticks: 0..255
  uint8_t leftStickX;   // 0 = left, 128 = center, 255 = right
  uint8_t leftStickY;   // 0 = up,   128 = center, 255 = down
  uint8_t rightStickX;  // same range
  uint8_t rightStickY;
} WiiClassicState;

/***********************************************************************
 * Our two ClassicController objects from the library
 ***********************************************************************/
ClassicController classic1;
ClassicController classic2;

/***********************************************************************
 * LUFA Fight Stick Bitmasks (from your original code)
 ***********************************************************************/
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

/***********************************************************************
 * 1) selectMuxChannel - choose which channel is active on PCA9548A
 ***********************************************************************/
static void selectMuxChannel(uint8_t channel) {
  Wire.beginTransmission(MUX_ADDR);
  // For channel n, write (1 << n)
  Wire.write(1 << channel);
  Wire.endTransmission();
}

/***********************************************************************
 * 2) connectWiiClassic - call .begin() and connect() on each controller
 ***********************************************************************/
static bool connectWiiClassic(ClassicController &ctrl, uint8_t channel) {
  selectMuxChannel(channel);
  ctrl.begin();  // Initialize internals
  // Try to connect (this library auto-handshakes)
  // Return true if connected
  return ctrl.connect();
}

/***********************************************************************
 * 3) readClassicController - read data from library into a WiiClassicState
 ***********************************************************************/
static WiiClassicState readClassicController(ClassicController &ctrl, uint8_t channel) {
  WiiClassicState st = { 0 };

  // 1) Select the multiplexer channel
  selectMuxChannel(channel);

  // 2) Update the library’s data
  bool success = ctrl.update();
  if (!success) {
    // If disconnected or error, return an empty struct
    // or handle it if you prefer
    return st;
  }

  // 3) Fill our struct from the library’s calls
  // --- Digital: D-Pad ---
  st.dpadUp = ctrl.dpadUp();
  st.dpadDown = ctrl.dpadDown();
  st.dpadLeft = ctrl.dpadLeft();
  st.dpadRight = ctrl.dpadRight();

  // --- Digital: Face Buttons ---
  st.buttonA = ctrl.buttonA();
  st.buttonB = ctrl.buttonB();
  st.buttonX = ctrl.buttonX();
  st.buttonY = ctrl.buttonY();

  // --- Shoulders / Triggers (digital portion) ---
  // The library typically has `.buttonL()`, `.buttonR()`, `.buttonZL()`, `.buttonZR()`
  // If using triggers as analog, you can call `.triggerL()` or `.triggerR()` to get 0..255
  st.buttonL = ctrl.buttonL();
  st.buttonR = ctrl.buttonR();
  st.buttonZL = ctrl.buttonZL();
  st.buttonZR = ctrl.buttonZR();

  // --- Special Buttons ---
  st.buttonHome = ctrl.buttonHome();
  st.buttonStart = ctrl.buttonPlus();    // "Plus"
  st.buttonSelect = ctrl.buttonMinus();  // "Minus"

  // --- Analog Joysticks (0..255) ---
  // The library returns an int from 0..255
  // But note the library might treat Y=0 as up or down depending on orientation
  st.leftStickY = 255 - ctrl.leftJoyY();
  st.rightStickY = 255 - ctrl.rightJoyY();
  st.leftStickX = ctrl.leftJoyX();
  st.rightStickX = ctrl.rightJoyX();

  return st;
}

/***********************************************************************
 * 4) mergeStates - OR the digital, average the analog
 ***********************************************************************/
static WiiClassicState mergeStates(const WiiClassicState &s1, const WiiClassicState &s2) {
  WiiClassicState out;
  // Digital: OR
  out.dpadUp = s1.dpadUp || s2.dpadUp;
  out.dpadDown = s1.dpadDown || s2.dpadDown;
  out.dpadLeft = s1.dpadLeft || s2.dpadLeft;
  out.dpadRight = s1.dpadRight || s2.dpadRight;

  out.buttonA = s1.buttonA || s2.buttonA;
  out.buttonB = s1.buttonB || s2.buttonB;
  out.buttonX = s1.buttonX || s2.buttonX;
  out.buttonY = s1.buttonY || s2.buttonY;
  out.buttonL = s1.buttonL || s2.buttonL;
  out.buttonR = s1.buttonR || s2.buttonR;
  out.buttonZL = s1.buttonZL || s2.buttonZL;
  out.buttonZR = s1.buttonZR || s2.buttonZR;
  out.buttonHome = s1.buttonHome || s2.buttonHome;
  out.buttonStart = s1.buttonStart || s2.buttonStart;
  out.buttonSelect = s1.buttonSelect || s2.buttonSelect;

  // Analog: average
  out.leftStickX = (uint8_t)(((uint16_t)s1.leftStickX + s2.leftStickX) / 2);
  out.leftStickY = (uint8_t)(((uint16_t)s1.leftStickY + s2.leftStickY) / 2);
  out.rightStickX = (uint8_t)(((uint16_t)s1.rightStickX + s2.rightStickX) / 2);
  out.rightStickY = (uint8_t)(((uint16_t)s1.rightStickY + s2.rightStickY) / 2);

  return out;
}

/***********************************************************************
 * 5) fillReportFromWii - set up the HID report from the merged state
 ***********************************************************************/
extern USB_JoystickReport_Input_t ReportData;  // from your Joystick.h

static void fillReportFromWii(const WiiClassicState &st) {
  // Clear the struct first
  memset(&ReportData, 0, sizeof(ReportData));

  // --- D-Pad / Hat Switch logic ---
  // We'll do a simple approach for the 8 directions
  if (st.dpadUp && st.dpadRight) {
    ReportData.HAT = DPAD_UPRIGHT_MASK_ON;
  } else if (st.dpadUp && st.dpadLeft) {
    ReportData.HAT = DPAD_UPLEFT_MASK_ON;
  } else if (st.dpadDown && st.dpadRight) {
    ReportData.HAT = DPAD_DOWNRIGHT_MASK_ON;
  } else if (st.dpadDown && st.dpadLeft) {
    ReportData.HAT = DPAD_DOWNLEFT_MASK_ON;
  } else if (st.dpadUp) {
    ReportData.HAT = DPAD_UP_MASK_ON;
  } else if (st.dpadDown) {
    ReportData.HAT = DPAD_DOWN_MASK_ON;
  } else if (st.dpadLeft) {
    ReportData.HAT = DPAD_LEFT_MASK_ON;
  } else if (st.dpadRight) {
    ReportData.HAT = DPAD_RIGHT_MASK_ON;
  } else {
    ReportData.HAT = DPAD_NOTHING_MASK_ON;
  }

  // --- Analog sticks (0..255) ---
  ReportData.LX = st.leftStickX;
  ReportData.LY = st.leftStickY;
  ReportData.RX = st.rightStickX;
  ReportData.RY = st.rightStickY;

  // --- Buttons ---
  if (st.buttonA) { ReportData.Button |= A_MASK_ON; }
  if (st.buttonB) { ReportData.Button |= B_MASK_ON; }
  if (st.buttonX) { ReportData.Button |= X_MASK_ON; }
  if (st.buttonY) { ReportData.Button |= Y_MASK_ON; }
  if (st.buttonL) { ReportData.Button |= LB_MASK_ON; }
  if (st.buttonR) { ReportData.Button |= RB_MASK_ON; }
  if (st.buttonZL) { ReportData.Button |= ZL_MASK_ON; }
  if (st.buttonZR) { ReportData.Button |= ZR_MASK_ON; }
  if (st.buttonStart) { ReportData.Button |= START_MASK_ON; }
  if (st.buttonSelect) { ReportData.Button |= SELECT_MASK_ON; }
  if (st.buttonHome) { ReportData.Button |= HOME_MASK_ON; }
}

/***********************************************************************
 * setup() - Arduino-style entry point
 ***********************************************************************/
void setup() {
  // Start I2C
  Wire.begin();
  delay(50);

  // Attempt to connect both controllers
  // Channel 0 => classic1, Channel 1 => classic2
  bool connected1 = connectWiiClassic(classic1, 0);
  bool connected2 = connectWiiClassic(classic2, 1);

  // Optional: if not connected, keep retrying or show an error
  if (!connected1) {
    // e.g. Serial.println("Controller #1 not detected!");
    // but let's keep going
  }
  if (!connected2) {
    // e.g. Serial.println("Controller #2 not detected!");
  }

  // LUFA hardware initialization
  SetupHardware();
  GlobalInterruptEnable();
}

/***********************************************************************
 * loop() - Arduino-style main loop
 ***********************************************************************/
void loop() {
  // 1) Read each controller
  WiiClassicState state1 = readClassicController(classic1, 0);
  WiiClassicState state2 = readClassicController(classic2, 1);

  // 2) Merge them (OR digital, average analog)
  WiiClassicState merged = mergeStates(state1, state2);

  // 3) Fill the USB HID report from merged state
  fillReportFromWii(merged);

  // 4) Send it out via LUFA
  HID_Task();
  USB_USBTask();
}
