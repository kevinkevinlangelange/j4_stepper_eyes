//******************************************************************************
//         file name:  j4_stepper_eyes.ino
//      v0_1 created:  2026-06-17 -- CDT -KL
//      last updated:  2026-06-17 -- CDT
//            author:  Kevin Lange
//       description:  Eye-pop stepper driver board for Johnny 4, on an ESP32-DEV
//                     (WROOM) module. Last node of the stepper daisy chain:
//
//                       j4_stepper_neck <--UART--> [ j4_stepper_eyes ]
//
//                     Drives the two eye-pop steppers through TMC2209 drivers, each
//                     on its OWN step/dir pins (two independent FastAccelStepper
//                     channels) but commanded to identical targets, so the eyes pop
//                     together yet can be trimmed/homed independently later over the
//                     TMC2209 UART (separate driver addresses 0b00 / 0b01).
//
//                     It receives the eye-pop value from j4_stepper_neck, drives
//                     both motors, polls each driver's DRV_STATUS for faults, and
//                     reports a status string back upstream.
//
//                     Eye-pop motion matches the neck (speed, accel, microsteps,
//                     threshold, /2 mapping) so it has the same smooth ramp. Only
//                     the driver current is separate (small eye motors).
//       update log:  v0_1 -- Initial build. Two independent eye channels, identical
//                            command, DRV_STATUS fault polling, status reporting.
//
//
//
//  ------------------------------------------------------------------
//  LINK PROTOCOL (UART1, to/from j4_stepper_neck)
//  ------------------------------------------------------------------
//   in:  "<eye>\n"     eye-pop target value (0-3200, same scale as the neck)
//   out: "S:<text>\n"  status, e.g. "S:OK", "S:EL OT", "S:ER SHORT"
//  ------------------------------------------------------------------
//
//
//  ESP32-DEV (WROOM) Pin Connections
//  ------------------------------------------------------------------
//  Eye-pop step/dir (independent per driver):
//   26:  STEP -- eye left      25:  DIR -- eye left
//   33:  STEP -- eye right     32:  DIR -- eye right
//
//  TMC2209 single-wire UART (shared bus, addressed via MS1/MS2 straps):
//   17:  UART2 TX -- TMC2209 PDN_UART (via 1k to each driver)
//   16:  UART2 RX -- TMC2209 PDN_UART
//        addresses: eye-left = 0b00, eye-right = 0b01
//
//  Upstream link to j4_stepper_neck, UART1, 2-way:
//   04:  UART1 RX  <-- neck GPIO21 (TX)
//   13:  UART1 TX  --> neck GPIO22 (RX)
//
//  Power / enable:
//   3.3V -> driver VIO        GND -> driver GND (common with ESP32)
//   12-24V PSU -> driver VM   (PSU GND common with ESP32 GND)
//   EN pins tied to GND -- drivers always enabled.
//  ------------------------------------------------------------------
//
//  NOTE: links use plain bidirectional GPIOs; strapping pins (0/2/12/15),
//        flash pins (6-11), and input-only pins (34-39) are avoided so this
//        runs on any WROOM board (some route battery sense to 35, LED to 2).
//
//******************************************************************************



#include <Arduino.h>
#include <HardwareSerial.h>
#include <TMCStepper.h>
#include "FastAccelStepper.h"


// ---------------------------------------------------------------------------
// PIN ASSIGNMENTS
// ---------------------------------------------------------------------------
// Eye-pop step/dir (one pair per driver)
#define STEP_PIN_EL   26   // eye left
#define DIR_PIN_EL    25
#define STEP_PIN_ER   33   // eye right
#define DIR_PIN_ER    32

// TMC2209 single-wire UART bus (both drivers share it, addressed via MS1/MS2)
#define TMC2209_TX    17
#define TMC2209_RX    16

// Upstream link to j4_stepper_neck (UART1, 2-way)
#define UP_RX_PIN      4
#define UP_TX_PIN     13

#define LINK_BAUD 250000   // must match j4_stepper_neck's downstream baud


// ---------------------------------------------------------------------------
// DRIVER + MOTION CONFIG
// ---------------------------------------------------------------------------
// Eye-pop motion matches the neck so it has the same smooth ramp. Only the
// driver current is separate -- the neck's 2.1A would cook small eye motors.
#define R_SENSE_2209    0.11f   // typical BTT/FYSETC TMC2209 sense resistor -- verify on your board
#define EYE_CURRENT      800    // RMS mA -- tune to your eye-pop motors (driver current, not motion)
#define EYE_MICROSTEPS    16
#define EYE_SPEED_HZ    3200    // same as the neck
#define EYE_ACCEL      16000    // same as the neck
#define EYE_THRESHOLD     20    // min change (0-3200 scale) to re-issue moveTo


// ---------------------------------------------------------------------------
// DRIVERS  (shared UART, separate addresses)
// ---------------------------------------------------------------------------
HardwareSerial tmcSerial(2);  // UART2 -> TMC2209 PDN_UART bus
TMC2209Stepper tmc_EL(&tmcSerial, R_SENSE_2209, 0b00);
TMC2209Stepper tmc_ER(&tmcSerial, R_SENSE_2209, 0b01);


// ---------------------------------------------------------------------------
// STEP GENERATORS (FastAccelStepper -- RMT hardware). One channel per eye so
// they can be trimmed/homed independently; both are commanded to the same target.
// ---------------------------------------------------------------------------
FastAccelStepperEngine engine;
FastAccelStepper *stepper_EL = NULL;
FastAccelStepper *stepper_ER = NULL;

int last_eye = 0;


// ---------------------------------------------------------------------------
// UPSTREAM LINK (UART1) + fixed-buffer parser
// ---------------------------------------------------------------------------
HardwareSerial upSerial(1);   // UART1 -> neck board
char    upBuf[32];  uint8_t upLen = 0;

#define STATUS_TX_MS      200   // how often to read DRV_STATUS and report upstream
unsigned long lastStatusTx = 0;


// Eye-pop travel maps the same way the neck does (half a 0-3200 range).
static inline int32_t eyeTarget(int eye) { return (int32_t)eye / 2; }


// Drive both eyes to the same target (one move command per channel).
void applyEye(int eye) {
  if (abs(eye - last_eye) < EYE_THRESHOLD) return;
  last_eye = eye;
  int32_t tgt = eyeTarget(eye);
  if (stepper_EL) stepper_EL->moveTo(tgt);
  if (stepper_ER) stepper_ER->moveTo(tgt);
}


// Parse "<eye>" -- a single integer target.
void parseUpstream(const char *line) {
  int eye;
  if (sscanf(line, "%d", &eye) == 1) applyEye(eye);
}


// Return a short fault token for one TMC2209, most-severe first ("" = healthy).
const char *faultToken2209(TMC2209Stepper &d) {
  if (d.ot())                                       return "OT";     // overtemp shutdown
  if (d.s2ga() || d.s2gb() || d.s2vsa() || d.s2vsb()) return "SHORT"; // short ground/supply
  if (d.otpw())                                     return "OTPW";   // overtemp prewarning
  if (d.ola()  || d.olb())                          return "OPEN";   // open load
  return "";
}


// Build the status string sent upstream: first faulting eye driver, else OK.
void buildStatus(char *out, size_t n) {
  const char *t;
  if ((t = faultToken2209(tmc_EL))[0])      snprintf(out, n, "EL %s", t);
  else if ((t = faultToken2209(tmc_ER))[0]) snprintf(out, n, "ER %s", t);
  else { strncpy(out, "OK", n); out[n - 1] = '\0'; }
}


void configTmc2209(TMC2209Stepper &drv) {
  drv.begin();
  drv.toff(4);
  drv.blank_time(24);
  drv.rms_current(EYE_CURRENT);
  drv.microsteps(EYE_MICROSTEPS);
  drv.en_spreadCycle(false);  // StealthChop (quiet)
  drv.pwm_autoscale(true);
}


FastAccelStepper *connectStepper(uint8_t stepPin, uint8_t dirPin) {
  FastAccelStepper *s = engine.stepperConnectToPin(stepPin);
  if (s) {
    s->setDirectionPin(dirPin);
    s->setSpeedInHz(EYE_SPEED_HZ);
    s->setAcceleration(EYE_ACCEL);
    s->setCurrentPosition(0);   // eyes retracted at boot
  }
  return s;
}


void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("j4_stepper_eyes booting...");

  // --- Upstream link to the neck board ---
  upSerial.begin(LINK_BAUD, SERIAL_8N1, UP_RX_PIN, UP_TX_PIN);

  // --- TMC2209 drivers over shared UART ---
  tmcSerial.begin(115200, SERIAL_8N1, TMC2209_RX, TMC2209_TX);
  configTmc2209(tmc_EL);
  configTmc2209(tmc_ER);

  // --- Step generators (one per eye) ---
  engine.init();
  stepper_EL = connectStepper(STEP_PIN_EL, DIR_PIN_EL);
  stepper_ER = connectStepper(STEP_PIN_ER, DIR_PIN_ER);

  if (!stepper_EL || !stepper_ER) {
    Serial.println("WARNING: a stepperConnectToPin() failed -- check STEP pins");
  } else {
    Serial.println("j4_stepper_eyes ready -- 2 eye motors, waiting for packets");
  }
}


void loop() {
  // Upstream: eye-pop target from the neck board.
  while (upSerial.available()) {
    char c = (char)upSerial.read();
    if (c == '\n') {
      upBuf[upLen] = '\0';
      if (upLen > 0) parseUpstream(upBuf);
      upLen = 0;
    } else if (c != '\r' && upLen < sizeof(upBuf) - 1) {
      upBuf[upLen++] = c;
    } else if (c != '\r') {
      upLen = 0;  // overflow -- drop and resync on next newline
    }
  }

  unsigned long now = millis();

  if (now - lastStatusTx >= STATUS_TX_MS) {
    lastStatusTx = now;
    char status[24];
    buildStatus(status, sizeof(status));
    upSerial.printf("S:%s\n", status);
  }
}
