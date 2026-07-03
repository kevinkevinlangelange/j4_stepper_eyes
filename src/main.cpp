//******************************************************************************
//         file name:  j4_stepper_eyes.ino
//      v0_1 created:  2026-06-17 -- CDT -KL
//      last updated:  2026-06-17 -- CDT
//      last updated:  2026-06-18 -- CDT
//      last updated:  2026-06-19 -- CDT
//      last updated:  2026-07-03 -- CDT
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
//                    v0_2 -- Added limit switches (two NC per axis in series, one
//                            input each: GPIO 34/35), StallGuard4 config + stall
//                            detection, a CAL homing routine that stops on a limit
//                            OR a stall (so an eye already at a stop is caught), and
//                            a safety stop if a limit trips mid-move. Limit state is
//                            reported in the status string. SG thresholds and homing
//                            params still need hardware tuning.
//                    v0_3 -- Moved the ER limit input off GPIO 35 to GPIO 36 (some
//                            LiPo dev boards route battery sense to GPIO 35). Limits
//                            are now 34 (EL) / 36 (ER); 35 and 39 are free.
//                    v0_4 -- Board confirmed as a bare ESP32-D0WD (onboard LED on
//                            GPIO 22, unused here). Doc update only: the neck's
//                            downstream RX moved to GPIO 35, so this board's TX
//                            (pin 13) now wires to neck GPIO 35.
//                    v0_5 -- Doc update only: the neck does not break out GPIO21, so
//                            its downstream TX moved to GPIO 22. This board's RX
//                            (pin 4) now wires to neck GPIO 22.
//                    v0_6 -- Board revived after a brief consolidation onto the neck
//                            board. Every limit switch now has its OWN input: one NC
//                            switch per axis endpoint, four inputs total, all on the
//                            input-only pins (EL MIN 34 / EL MAX 39 / ER MIN 36 /
//                            ER MAX 35, external 10k pull-ups). Faults name the
//                            exact switch ("EL MIN", "ER MAX", ...). Homing detects
//                            "already on the home switch" directly and backs off
//                            first; StallGuard4 stays as a backup jam detector. Mid-
//                            move safety is direction-aware: only a move TOWARD a
//                            tripped end is stopped, so an eye can always drive
//                            itself back off a limit.
//
//
//
//  ------------------------------------------------------------------
//  LINK PROTOCOL (UART1, to/from j4_stepper_neck)
//  ------------------------------------------------------------------
//   in:  "<eye>\n"     eye-pop target value (0-3200, same scale as the neck)
//   in:  "CAL\n"       run a homing/calibration pass on both eyes
//   out: "S:<text>\n"  status, e.g. "S:OK", "S:EL OT", "S:EL LIMIT"
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
//   04:  UART1 RX  <-- neck GPIO22 (TX)
//   13:  UART1 TX  --> neck GPIO35 (RX)
//
//  GPIO 22 carries this board's onboard LED; it is unused by the firmware.
//
//  Limit switches -- ONE NC switch per axis endpoint, four inputs total, all on
//  the input-only pins (external 10k pull-up to 3.3V each; LOW = in range,
//  HIGH = that endpoint hit / broken wire). MIN = home end (CAL drives toward
//  it); MAX = far end. GPIO 35 is safe here (no battery-sense on this board):
//   34:  eye-left  MIN          39:  eye-left  MAX
//   36:  eye-right MIN          35:  eye-right MAX
//
//  StallGuard: read over UART (no extra pins). During calibration each eye homes
//  to its MIN switch OR a StallGuard stall (so an eye jammed short of the switch is
//  caught at once). Thresholds in the code need tuning on the real mechanism.
//
//  Power / enable:
//   3.3V -> driver VIO        GND -> driver GND (common with ESP32)
//   12-24V PSU -> driver VM   (PSU GND common with ESP32 GND)
//   EN pins tied to GND -- drivers always enabled.
//  ------------------------------------------------------------------
//
//  NOTE: links and step/dir use plain bidirectional GPIOs (strapping pins
//        0/2/12/15 and flash pins 6-11 avoided). The input-only pins 34/35/36/39
//        carry the four limit inputs, which is what they are good for -- they
//        have no internal pull-up, hence the external 10k resistors noted above.
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
// LIMIT SWITCHES + STALLGUARD
// ---------------------------------------------------------------------------
// One normally-closed switch per axis endpoint, four inputs total. Each switch
// shorts its input to GND while the eye is in range: LOW = in range, HIGH =
// that endpoint was hit OR its wire broke (fail-safe). Since every endpoint has
// its own input, faults can name the exact switch. All four are input-only pins
// (no internal pull-up), so each needs an external 10k to 3.3V. MIN = the home
// end that CAL drives toward; MAX = the far end.
#define LIMIT_PIN_EL_MIN  34
#define LIMIT_PIN_EL_MAX  39
#define LIMIT_PIN_ER_MIN  36
#define LIMIT_PIN_ER_MAX  35
static inline bool limitHit(uint8_t pin) { return digitalRead(pin) == HIGH; }

// StallGuard4 (read over UART). SG_RESULT drops toward 0 under load; near 0 is a
// stall. Used during calibration so an eye already jammed against a stop is caught
// the moment it tries to move. THESE VALUES NEED HARDWARE TUNING.
#define SG_SGTHRS         60       // StallGuard4 threshold (0-255); higher = more sensitive
#define SG_TCOOLTHRS  0xFFFFF      // enable StallGuard above near-standstill speed
#define SG_STALL_LEVEL    40       // SG_RESULT below this counts as a stall (tune)
static inline bool stalled2209(TMC2209Stepper &drv) { return drv.SG_RESULT() < SG_STALL_LEVEL; }

// Homing/calibration tunables (per-hardware -- adjust on the bench).
#define HOMING_SPEED_HZ   800      // slow approach speed
#define HOMING_ACCEL     8000
#define HOMING_BACKOFF    200      // microsteps to back off after touching a limit
#define HOMING_TIMEOUT_MS 9000     // safety cap per axis


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


bool calRequested = false;

// Parse "<eye>" (single integer target) or "CAL" (request calibration).
void parseUpstream(const char *line) {
  if (strcmp(line, "CAL") == 0) { calRequested = true; return; }
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


// Build the status string sent upstream: first faulting eye driver, then the
// first tripped limit switch (named exactly), else OK.
void buildStatus(char *out, size_t n) {
  const char *t;
  if ((t = faultToken2209(tmc_EL))[0])      snprintf(out, n, "EL %s", t);
  else if ((t = faultToken2209(tmc_ER))[0]) snprintf(out, n, "ER %s", t);
  else if (limitHit(LIMIT_PIN_EL_MIN))      { strncpy(out, "EL MIN", n); out[n - 1] = '\0'; }
  else if (limitHit(LIMIT_PIN_EL_MAX))      { strncpy(out, "EL MAX", n); out[n - 1] = '\0'; }
  else if (limitHit(LIMIT_PIN_ER_MIN))      { strncpy(out, "ER MIN", n); out[n - 1] = '\0'; }
  else if (limitHit(LIMIT_PIN_ER_MAX))      { strncpy(out, "ER MAX", n); out[n - 1] = '\0'; }
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
  // StallGuard4 (used during calibration; needs tuning on real hardware)
  drv.TCOOLTHRS(SG_TCOOLTHRS);
  drv.SGTHRS(SG_SGTHRS);
}


// Safety: if an eye is moving TOWARD a tripped endpoint, stop it. Knowing which
// end tripped means a move away from the switch is still allowed, so an eye can
// always drive itself back off a limit.
void checkAxisLimits(FastAccelStepper *s, uint8_t minPin, uint8_t maxPin) {
  if (!s || !s->isRunning()) return;
  bool movingDown = s->targetPos() < s->getCurrentPosition();
  if (movingDown  && limitHit(minPin)) s->forceStop();
  if (!movingDown && limitHit(maxPin)) s->forceStop();
}

void checkLimits() {
  checkAxisLimits(stepper_EL, LIMIT_PIN_EL_MIN, LIMIT_PIN_EL_MAX);
  checkAxisLimits(stepper_ER, LIMIT_PIN_ER_MIN, LIMIT_PIN_ER_MAX);
}


// Home one eye toward its MIN switch. An eye already sitting on the switch is
// detected immediately (the switch reads pressed) and just backs off and zeros.
// StallGuard stays as a backup: it catches a jam short of the switch. NEEDS
// HARDWARE VALIDATION -- direction/speed/backoff/SG tunable above.
void homeAxis(FastAccelStepper *s, uint8_t minPin, uint8_t maxPin, TMC2209Stepper &drv) {
  if (!s) return;
  s->setSpeedInHz(HOMING_SPEED_HZ);
  s->setAcceleration(HOMING_ACCEL);
  unsigned long t0 = millis();

  // Approach the MIN end unless we are already pressing its switch.
  if (!limitHit(minPin)) {
    s->move(-1000000);
    while (s->isRunning()) {
      if (limitHit(minPin))                  { s->forceStop(); break; }
      if (limitHit(maxPin))                  { s->forceStop(); break; }  // wrong end / wiring fault
      if (stalled2209(drv))                  { s->forceStop(); break; }  // jammed short of the switch
      if (millis() - t0 > HOMING_TIMEOUT_MS) { s->forceStop(); break; }
      delay(1);
    }
    delay(50);
  }

  // Back off until the MIN switch releases (retry in fixed nudges), then zero.
  s->setCurrentPosition(0);
  t0 = millis();
  while (limitHit(minPin) && (millis() - t0 < HOMING_TIMEOUT_MS)) {
    s->move(HOMING_BACKOFF);
    while (s->isRunning() && limitHit(minPin) &&
           (millis() - t0 < HOMING_TIMEOUT_MS)) delay(1);
  }
  s->forceStop();
  delay(50);
  s->setCurrentPosition(0);

  s->setSpeedInHz(EYE_SPEED_HZ);
  s->setAcceleration(EYE_ACCEL);
}


// Home both eyes independently (the whole point of separate channels).
void calibrate() {
  homeAxis(stepper_EL, LIMIT_PIN_EL_MIN, LIMIT_PIN_EL_MAX, tmc_EL);
  homeAxis(stepper_ER, LIMIT_PIN_ER_MIN, LIMIT_PIN_ER_MAX, tmc_ER);
  last_eye = -1;   // force a fresh move on the next packet
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

  // --- Limit switches (one NC per endpoint, external pull-ups -> plain INPUT) ---
  pinMode(LIMIT_PIN_EL_MIN, INPUT);
  pinMode(LIMIT_PIN_EL_MAX, INPUT);
  pinMode(LIMIT_PIN_ER_MIN, INPUT);
  pinMode(LIMIT_PIN_ER_MAX, INPUT);

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

  // Run a calibration pass if one was requested (blocks while homing).
  if (calRequested) {
    calRequested = false;
    calibrate();
  }

  // Safety: stop any eye whose limit trips mid-move.
  checkLimits();

  unsigned long now = millis();

  if (now - lastStatusTx >= STATUS_TX_MS) {
    lastStatusTx = now;
    char status[24];
    buildStatus(status, sizeof(status));
    upSerial.printf("S:%s\n", status);
  }
}
