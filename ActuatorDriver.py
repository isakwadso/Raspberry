#!/usr/bin/env python3
"""
Drive the Actuonix S20-100-38-B linear actuator back and forth through a
Pololu Tic T825 USB stepper motor controller, with selectable microstepping
and the Tic's built-in acceleration ramping.

Wiring:
    Actuator Blue  -> Tic A1
    Actuator Red   -> Tic A2
    Actuator Green -> Tic B1
    Actuator Black -> Tic B2
    External ~12V supply -> Tic VIN / GND (the pair next to A1)
    Pi USB port -> Tic USB port (standard USB-A to Micro-B cable)

    No GPIO wiring at all -- everything is over USB.

One-time setup (see comments below and the accompanying message for the
full walkthrough):
    1. Install Pololu's Tic software on the Pi (gives you `ticcmd`).
    2. Run `ticcmd --current 600` once to set the current limit to 600mA,
       matching the actuator's 0.64A/phase rating. This is a persistent
       setting stored on the Tic itself -- you don't need to repeat it
       every run.

Requires:
    pip install ticlib pyusb
"""

import time
from ticlib import TicUSB

# ---------------------------------------------------------------------------
# Motion configuration
# ---------------------------------------------------------------------------
FULL_STEP_MM = 0.01         # from the Actuonix S20 datasheet: 0.01mm per full step
TRAVEL_MM = 40.0          # distance to travel each way -- keep well inside the
                             # actuator's real end-of-travel until confirmed safe
TARGET_SPEED_MM_S = 4.0     # real-world speed target, held constant across
                             # whichever microstep resolution you pick
RAMP_TIME_S = 0.3           # time to accelerate from starting speed to target speed
CYCLES = 2

# microstep divisor -> Tic's "Set step mode" protocol value (confirmed against
# Pololu's Tic command reference: 0=Full, 1=1/2, 2=1/4, 3=1/8, 4=1/16, 5=1/32)
STEP_MODE_VALUES = {
    1:  0,
    2:  1,
    4:  2,
    8:  3,
    16: 4,
}

MOVE_TIMEOUT_S = 15


def error_status_int(value):
    """The installed ticlib version returns get_error_status() as an
    array.array rather than a plain int -- normalize either form to int."""
    if isinstance(value, int):
        return value
    return int.from_bytes(bytes(value), 'little')


def configure_motion(tic, microstep_divisor):
    """Set step mode and a speed/accel profile that gives ~TARGET_SPEED_MM_S
    regardless of which microstep resolution is chosen."""
    if microstep_divisor not in STEP_MODE_VALUES:
        raise ValueError(f"microstep_divisor must be one of {sorted(STEP_MODE_VALUES)}")

    tic.set_step_mode(STEP_MODE_VALUES[microstep_divisor])

    mm_per_step = FULL_STEP_MM / microstep_divisor
    target_steps_per_sec = TARGET_SPEED_MM_S / mm_per_step
    starting_steps_per_sec = target_steps_per_sec * 0.1
    accel_steps_per_sec2 = (target_steps_per_sec - starting_steps_per_sec) / RAMP_TIME_S

    # Tic units: speed in steps per 10000 seconds, accel/decel in steps per
    # second per 100 seconds (see Pololu's Tic command reference)
    tic.set_max_speed(int(target_steps_per_sec * 10000))
    tic.set_starting_speed(int(starting_steps_per_sec * 10000))
    tic.set_max_acceleration(int(accel_steps_per_sec2 * 100))
    tic.set_max_deceleration(int(accel_steps_per_sec2 * 100))

    return int(round(TRAVEL_MM / mm_per_step))  # steps for the full travel distance


def wait_until_arrived(tic, timeout=MOVE_TIMEOUT_S):
    elapsed = 0.0
    while tic.get_current_position() != tic.get_target_position():
        error = error_status_int(tic.get_error_status())
        if error:
            print(f"!! Tic reports an active error (status bits: {error:#06x}) -- "
                  f"stopping. Check Tic Control Center or `ticcmd -s` for details.")
            return False
        time.sleep(0.05)
        elapsed += 0.05
        if elapsed > timeout:
            print("Warning: move took longer than expected, may have stalled.")
            return False
    return True


def move_to(tic, position):
    tic.set_target_position(position)
    return wait_until_arrived(tic)


def main(microstep_divisor=8):
    tic = TicUSB()

    steps = configure_motion(tic, microstep_divisor)
    print(f"Microstepping: 1/{microstep_divisor}, {steps} steps per {TRAVEL_MM}mm move, "
          f"target speed {TARGET_SPEED_MM_S} mm/s")

    try:
        tic.halt_and_set_position(0)   # treats the current physical position as
                                        # "0" -- not true homing, so start from a
                                        # known-safe position the first time
        tic.energize()
        tic.exit_safe_start()

        # Checking here (after exit_safe_start) rather than right after
        # connecting -- a fresh Tic always reports a "safe start violation"
        # error before exit_safe_start() is called, which isn't a real fault.
        # Anything still reported here is a genuine issue (e.g. Low VIN if
        # the motor power supply isn't connected).
        error = error_status_int(tic.get_error_status())
        if error:
            print(f"Tic reports an error after exit_safe_start (status bits: "
                  f"{error:#06x}) -- run `ticcmd -s` in another terminal for a "
                  f"human-readable breakdown.")
            return

        for cycle in range(1, CYCLES + 1):
            print(f"Cycle {cycle}: extending")
            if not move_to(tic, steps):
                break
            time.sleep(0.5)

            print(f"Cycle {cycle}: retracting")
            if not move_to(tic, 0):
                break
            time.sleep(0.5)

    finally:
        tic.deenergize()
        tic.enter_safe_start()


if __name__ == "__main__":
    # Set to 1, 2, 4, 8, or 16 for full, half, 1/4, 1/8, or 1/16 step
    main(microstep_divisor=8)