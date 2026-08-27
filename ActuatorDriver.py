#!/usr/bin/env python3
"""
---
Drive the Actuonix S20-100-38-B linear actuator back and forth through a
DRV8825 stepper driver, with selectable microstepping, sleep control, and
a fault (FLT) watchdog wired in from the start.

Pin map (BCM numbering) -- kept within physical pins 1-26, UART (pins 8/10),
I2C (pins 3/5) and SPI (pins 19/21/23/24/26) left completely free. As many
signals as possible land on even physical pins for easier wiring, but with
UART/SPI off-limits only 4 even pins remain, so two of the six control
signals (plus FLT, which is fine on either side) sit on odd pins instead:

    Driver pin   Function          Pi physical pin   BCM GPIO
    ----------   ---------------   ----------------   --------
    STP          Step pulse        12 (even)          GPIO18
    DIR          Direction         16 (even)          GPIO23
    SLP          Sleep             18 (even)          GPIO24
    M0           Microstep bit 0   22 (even)          GPIO25
    M1           Microstep bit 1   15 (odd)           GPIO22
    M2           Microstep bit 2   13 (odd)           GPIO27
    FLT          Fault (input)     11 (odd)           GPIO17

    RST          Reset             -> Pi 3.3V (physical pin 1 or 17), hardwired,
                                       always released
    EN           Enable            -> any Pi GND pin (e.g. pin 6), hardwired, always
                                       enabled -- SLP is used instead to power the
                                       coils down between moves
    GND (logic)  Ground            -> any Pi GND pin, shared with EN's ground wire
    VMOT / GND   Motor power       -> external ~12V supply, NOT the Pi

    1A / 1B      Coil 1            -> actuator Blue / Red
    2A / 2B      Coil 2            -> actuator Green / Black

    Note: physical pin 17 (3.3V, used for RST) is NOT the same as BCM
    "GPIO17" (used here for FLT, which actually lives at physical pin 11) --
    the numbers just look alike.

Requires:
    pip install RpiMotorLib RPi.GPIO --break-system-packages

Direction note: whether `clockwise=True` extends or retracts the actuator
depends on which coil wire landed on 1A vs 1B (and 2A vs 2B). Run one short
move and flip the CLOCKWISE_EXTENDS constant below if it moves the wrong way.
"""

import time
import RPi.GPIO as GPIO
from RpiMotorLib import RpiMotorLib

# ---------------------------------------------------------------------------
# Pin configuration
# ---------------------------------------------------------------------------
DIR_PIN = 23
STEP_PIN = 18
MODE_PINS = (25, 22, 27)    # M0, M1, M2
SLEEP_PIN = 24
FAULT_PIN = 17

# ---------------------------------------------------------------------------
# Motion configuration
# ---------------------------------------------------------------------------
FULL_STEP_MM = 0.01         # from the Actuonix S20 datasheet: 0.01 mm per full step
TRAVEL_MM = 2            # distance to travel each way -- keep this well inside the
                             # actuator's real end-of-travel until you've confirmed the
                             # safe range on the bench
BASE_STEP_DELAY = 0.0020    # seconds per half-pulse at full step -- this is the speed
                             # reference; it's scaled down automatically as microstepping
                             # gets finer so real-world speed (mm/s) stays roughly constant
CYCLES = 1
CLOCKWISE_EXTENDS = True    # flip this if the first test move goes the wrong way

# steptype string RpiMotorLib expects for each microstep divisor
MICROSTEP_OPTIONS = {
    1:  "Full",
    2:  "Half",
    4:  "1/4",
    8:  "1/8",
    16: "1/16",
}

SLEEP_WAKE_DELAY = 0.005    # DRV8825 needs ~1.7ms after leaving sleep before the first
                             # step is valid; 5ms gives comfortable headroom

fault_triggered = False


def _on_fault(channel):
    global fault_triggered
    fault_triggered = True
    print("!! DRV8825 FLT pin went low -- driver reports a fault (overcurrent, "
          "thermal shutdown, or undervoltage lockout). Motion will stop.")


def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)  # we deliberately re-touch already-claimed pins below
    GPIO.setup(SLEEP_PIN, GPIO.OUT)
    GPIO.setup(FAULT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(FAULT_PIN, GPIO.FALLING, callback=_on_fault, bouncetime=50)

    # rpi-lgpio (the RPi.GPIO-compatible shim used on Raspberry Pi OS Trixie)
    # has a bug/limitation when RpiMotorLib sets up the three microstep pins
    # as a single batched call (GPIO.setup(self.mode_pins, GPIO.OUT)) -- it
    # tries to read a pin's state before it has ever been claimed, which the
    # underlying lgpio backend rejects with "GPIO not allocated". Claiming
    # each pin individually here first works around it: RpiMotorLib's later
    # batched re-setup just re-touches already-claimed pins instead.
    for pin in MODE_PINS:
        GPIO.setup(pin, GPIO.OUT)

    wake()


def wake():
    GPIO.output(SLEEP_PIN, GPIO.HIGH)
    time.sleep(SLEEP_WAKE_DELAY)


def sleep_driver():
    GPIO.output(SLEEP_PIN, GPIO.LOW)


def steps_for_travel(microstep_divisor, travel_mm):
    mm_per_step = FULL_STEP_MM / microstep_divisor
    return int(round(travel_mm / mm_per_step))


def move(motor, steps, extend, microstep_divisor):
    global fault_triggered
    if fault_triggered:
        print("Refusing to move: a fault is still latched. Power-cycle VMOT and "
              "restart the script once the cause is fixed.")
        return False

    clockwise = extend if CLOCKWISE_EXTENDS else (not extend)
    stepdelay = BASE_STEP_DELAY / microstep_divisor

    motor.motor_go(
        clockwise=clockwise,
        steptype=MICROSTEP_OPTIONS[microstep_divisor],
        steps=steps,
        stepdelay=stepdelay,
        verbose=False,
        initdelay=0.05,
    )

    if fault_triggered:
        print("Fault occurred during the move -- stopping.")
        return False
    return True


def main(microstep_divisor=8):
    if microstep_divisor not in MICROSTEP_OPTIONS:
        raise ValueError(f"microstep_divisor must be one of {sorted(MICROSTEP_OPTIONS)}")

    setup_gpio()
    motor = RpiMotorLib.A4988Nema(DIR_PIN, STEP_PIN, MODE_PINS, motor_type="DRV8825")

    steps = steps_for_travel(microstep_divisor, TRAVEL_MM)
    print(f"Microstepping: 1/{microstep_divisor} ({MICROSTEP_OPTIONS[microstep_divisor]}), "
          f"{steps} steps per {TRAVEL_MM}mm move")

    try:
        for cycle in range(1, CYCLES + 1):
            if fault_triggered:
                break

            print(f"Cycle {cycle}: extending")
            if not move(motor, steps, extend=True, microstep_divisor=microstep_divisor):
                break
            time.sleep(0.5)

            print(f"Cycle {cycle}: retracting")
            if not move(motor, steps, extend=False, microstep_divisor=microstep_divisor):
                break
            time.sleep(0.5)

    finally:
        sleep_driver()
        GPIO.cleanup()


if __name__ == "__main__":
    # Set to 1, 2, 4, 8, or 16 for full, half, 1/4, 1/8, or 1/16 step
    main(microstep_divisor=8)