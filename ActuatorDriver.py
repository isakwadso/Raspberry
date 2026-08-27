#!/usr/bin/env python3
"""
Drive the Actuonix S20-100-38-B linear actuator back and forth through a
DRV8825 stepper driver, using gpiozero instead of RPi.GPIO/RpiMotorLib.

Wiring is UNCHANGED from before -- same pins, same physical connections
(BCM numbering, physical pins 1-26 only):

    Driver pin   Function          Pi physical pin   BCM GPIO
    ----------   ---------------   ----------------   --------
    STP          Step pulse        12 (even)          GPIO18
    DIR          Direction         16 (even)          GPIO23
    SLP          Sleep             18 (even)          GPIO24
    M0           Microstep bit 0    7 (odd)           GPIO4
    M1           Microstep bit 1   15 (odd)           GPIO22
    M2           Microstep bit 2   13 (odd)           GPIO27
    FLT          Fault (input)     11 (odd)           GPIO17

    NOTE: M0 was originally GPIO25 (physical pin 22), but that pin is
    permanently claimed as a "dc" (data/command) line by the Adafruit PiTFT
    touchscreen overlay already enabled in this Pi's boot config -- moved to
    GPIO4 instead, confirmed free via `gpioinfo`.

    RST          Reset             -> Pi 3.3V (physical pin 1 or 17), hardwired,
                                       always released
    EN           Enable            -> any Pi GND pin, hardwired, always enabled --
                                       SLEEP is what actually powers the coils
                                       down between moves
    GND (logic)  Ground            -> any Pi GND pin
    VMOT / GND   Motor power       -> external ~12V supply, NOT the Pi
    1A / 1B      Coil 1            -> actuator Blue / Red
    2A / 2B      Coil 2            -> actuator Green / Black

SAFETY NOTE: SLEEP is asserted LOW (driver asleep, coils de-energized) the
instant this module creates its device objects -- i.e. before any of our own
logic runs -- specifically to avoid the coils ever being left energized in
an undefined state if the script is interrupted before main() gets going.

Requires:
    pip install gpiozero
    (gpiozero picks whichever pin factory is installed; on Raspberry Pi OS
    Trixie that's lgpio via the python3-lgpio package you already have.)
"""

import time
from gpiozero import DigitalOutputDevice, DigitalInputDevice

# ---------------------------------------------------------------------------
# Pin configuration
# ---------------------------------------------------------------------------
DIR_PIN = 23
STEP_PIN = 18
MODE_PINS = (4, 22, 27)     # M0, M1, M2
SLEEP_PIN = 24
FAULT_PIN = 17

# ---------------------------------------------------------------------------
# Motion configuration
# ---------------------------------------------------------------------------
FULL_STEP_MM = 0.01         # from the Actuonix S20 datasheet: 0.01 mm per full step
TRAVEL_MM = 40.0            # distance to travel each way -- keep well inside the
                             # actuator's real end-of-travel until confirmed safe
BASE_STEP_DELAY = 0.0020    # seconds per half-pulse at full step; scaled down as
                             # microstepping gets finer so real speed stays similar
CYCLES = 5
CLOCKWISE_EXTENDS = True    # flip if the first test move goes the wrong way

# microstep divisor -> (M0, M1, M2) levels, per the DRV8825 truth table
MICROSTEP_OPTIONS = {
    1:  (0, 0, 0),
    2:  (1, 0, 0),
    4:  (0, 1, 0),
    8:  (1, 1, 0),
    16: (0, 0, 1),
}

SLEEP_WAKE_DELAY = 0.005    # DRV8825 needs ~1.7ms after leaving sleep before the
                             # first step is valid; 5ms gives headroom

fault_triggered = False

# ---------------------------------------------------------------------------
# Devices -- created with an explicit initial state so nothing is ever left
# energized in an undefined state. SLEEP starts LOW (asleep) immediately.
# ---------------------------------------------------------------------------
step = DigitalOutputDevice(STEP_PIN, initial_value=False)
direction = DigitalOutputDevice(DIR_PIN, initial_value=False)
sleep_pin = DigitalOutputDevice(SLEEP_PIN, initial_value=False)   # starts asleep
mode = [DigitalOutputDevice(pin, initial_value=False) for pin in MODE_PINS]
fault = DigitalInputDevice(FAULT_PIN, pull_up=True)   # pull_up=True -> "active"
                                                        # means the pin read LOW


def _on_fault():
    global fault_triggered
    fault_triggered = True
    print("!! DRV8825 FLT pin went low -- driver reports a fault (overcurrent, "
          "thermal shutdown, or undervoltage lockout). Motion will stop.")


fault.when_activated = _on_fault


def wake():
    sleep_pin.on()
    time.sleep(SLEEP_WAKE_DELAY)


def sleep_driver():
    sleep_pin.off()


def set_microstep(divisor):
    for pin, bit in zip(mode, MICROSTEP_OPTIONS[divisor]):
        pin.value = bit


def steps_for_travel(microstep_divisor, travel_mm):
    mm_per_step = FULL_STEP_MM / microstep_divisor
    return int(round(travel_mm / mm_per_step))


def move(steps, extend, microstep_divisor):
    global fault_triggered
    if fault_triggered:
        print("Refusing to move: a fault is still latched. Power-cycle VMOT and "
              "restart the script once the cause is fixed.")
        return False

    clockwise = extend if CLOCKWISE_EXTENDS else (not extend)
    direction.value = clockwise
    step_delay = BASE_STEP_DELAY / microstep_divisor

    for _ in range(steps):
        if fault_triggered:
            print("Fault occurred during the move -- stopping.")
            return False
        step.on()
        time.sleep(step_delay)
        step.off()
        time.sleep(step_delay)

    return True


def main(microstep_divisor=8):
    if microstep_divisor not in MICROSTEP_OPTIONS:
        raise ValueError(f"microstep_divisor must be one of {sorted(MICROSTEP_OPTIONS)}")

    set_microstep(microstep_divisor)
    wake()

    steps = steps_for_travel(microstep_divisor, TRAVEL_MM)
    print(f"Microstepping: 1/{microstep_divisor}, {steps} steps per {TRAVEL_MM}mm move")

    try:
        for cycle in range(1, CYCLES + 1):
            if fault_triggered:
                break

            print(f"Cycle {cycle}: extending")
            if not move(steps, extend=True, microstep_divisor=microstep_divisor):
                break
            time.sleep(0.5)

            print(f"Cycle {cycle}: retracting")
            if not move(steps, extend=False, microstep_divisor=microstep_divisor):
                break
            time.sleep(0.5)

    finally:
        sleep_driver()
        step.off()
        for device in (step, direction, sleep_pin, fault, *mode):
            device.close()


if __name__ == "__main__":
    # Set to 1, 2, 4, 8, or 16 for full, half, 1/4, 1/8, or 1/16 step
    main(microstep_divisor=8)