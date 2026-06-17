# j4_stepper_eyes

Eye-pop stepper driver board for the Johnny 4 robot, and the last node of the stepper daisy chain. Runs on an ESP32-DEV (WROOM) module. It drives the two eye-pop steppers through TMC2209 drivers and reports their health back upstream. Step pulses run on the ESP32 RMT peripheral via FastAccelStepper.

## Daisy chain

```
  j4_receiver (TTGO)
        |  UART1
        v
  j4_stepper_neck   (3x TMC5160T)
        |  UART2
        v
  j4_stepper_eyes   (2x TMC2209)   <-- this board
```

This board is the end of the chain. It receives the eye-pop value forwarded by j4_stepper_neck and sends its status back up the same link.

## What this board does

- Receives an eye-pop target (`<eye>\n`, 0-3200) from j4_stepper_neck over UART1
- Drives 2 eye-pop steppers via TMC2209 drivers, **each on its own STEP/DIR pins** (two independent FastAccelStepper channels) but **commanded to identical targets** so the eyes pop together
- Polls each TMC2209's DRV_STATUS for faults (overtemp prewarning/shutdown, short, open-load) and reports a status string back upstream
- Filters out tiny commanded changes (threshold) so the motors do not jitter at rest

## Why two independent channels instead of one shared signal

The eyes always pop to the same value, so a single shared STEP/DIR signal would also keep them in sync. They are kept on **separate channels** so each eye can be **trimmed or homed independently** later (re-zeroing one motor after a skipped step, or sensorless homing per driver via StallGuard) without disturbing the other. The two TMC2209s sit on a shared UART bus at separate addresses (0b00 / 0b01), so each is individually addressable for that.

## Hardware

| Component | Details |
|-----------|---------|
| Microcontroller | ESP32-DEV (WROOM-32) |
| Eye-pop drivers | 2x TMC2209 (shared single-wire UART, addresses 0b00 / 0b01) |

## Link protocol (UART1, to/from j4_stepper_neck)

| Direction | Message | Meaning |
|-----------|---------|---------|
| in | `<eye>\n` | eye-pop target (0-3200, same scale as the neck) |
| out | `S:<text>\n` | status: `S:OK`, `S:EL OT`, `S:ER SHORT`, etc. |

The status tokens are `EL`/`ER` (eye left/right) plus `OT` (overtemp shutdown), `SHORT`, `OTPW` (overtemp prewarning), or `OPEN` (open load). The parser uses a fixed char buffer (no Arduino `String`).

## Pin assignments

| GPIO | Function |
|------|----------|
| 26 / 25 | eye-left STEP / DIR |
| 33 / 32 | eye-right STEP / DIR |
| 17 / 16 | TMC2209 UART TX / RX (shared PDN_UART bus) |
| 4 / 13 | Upstream UART1 RX / TX (to neck board) |

All links use plain bidirectional GPIOs; strapping pins (0/2/12/15), flash pins (6-11), and input-only pins (34-39) are avoided so the firmware runs on any of these WROOM boards.

## Pin diagram

```
                   ESP32-DEV (WROOM)  -  j4_stepper_eyes
                  +--------------------------------------+
   neck 21 (TX) ->| 4  (UART1 RX)                        |
   neck 22 (RX) <-| 13 (UART1 TX)                        |
                  |                                      |
  eye L STEP/DIR  | 26 / 25         ---> TMC2209 (eye L, addr 0b00)
  eye R STEP/DIR  | 33 / 32         ---> TMC2209 (eye R, addr 0b01)
   TMC2209 UART   | 17 TX   16 RX   (shared PDN_UART bus)|
                  |        3V3   GND   VIN   VM(12-24V)  |
                  +--------------------------------------+
```

## Power and enable

VIO on each driver goes to 3.3V, VM to the 12-24V motor supply, and all grounds are common with the ESP32. Each driver's EN pin is tied to GND, so the drivers are always enabled.

## Motion settings

Eye-pop motion matches the neck so it has the same smooth ramp: `EYE_SPEED_HZ` (3200), `EYE_ACCEL` (16000), `EYE_MICROSTEPS` (16), and `EYE_THRESHOLD` (20) mirror the neck values, and the eye value maps through `eye / 2` exactly like a neck axis. Only `EYE_CURRENT` is separate, since it sets the TMC2209 driver current for the small eye-pop motors (not a motion setting) and the neck's 2.1A would be far too much. Tune `EYE_CURRENT` to your motors.

## Building and uploading

```bash
# Build
pio run

# Upload
pio run --target upload

# Monitor serial output
pio device monitor
```

## Related projects

- **[j4_stepper_neck](https://github.com/kevinkevinlangelange/j4_stepper_neck)** -- the neck stepper board upstream of this one, which forwards the eye value and relays this board's status
