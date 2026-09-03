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
TRAVEL_MM = 40.0            # distance to travel each way -- keep well inside the
                             # actuator's real end-of-travel until confirmed safe
TARGET_SPEED_MM_S = 4.0     # real-world speed target, held constant across
                             # whichever microstep resolution you pick
RAMP_TIME_S = 0.3           # time to accelerate from starting speed to target speed
CYCLES = 2

# ---------------------------------------------------------------------------
# Homing configuration
# ---------------------------------------------------------------------------
# The Tic runs open-loop -- it has no way to know whether a commanded move
# actually completed or the motor stalled partway through. A stall (e.g.
# against the actuator's hard stop, or under unexpected load) doesn't error
# out or slow the Tic's own position counter, so its belief about where the
# actuator is can silently drift from reality. The only way to regain real
# ground truth without limit switches or an encoder is to deliberately drive
# into a known physical reference (here, the actuator's retracted hard stop)
# and re-zero there.
#
# Make sure the actuator has full clearance to retract before running this
# -- homing intentionally drives it into that end stop.
HOME_OVERTRAVEL_MM = 10.0    # commanded distance past the actuator's real
                             # travel when homing -- guarantees it reaches
                             # and stalls against the physical retracted end
                             # stop well before the commanded distance
HOME_SPEED_MM_S = 2.0        # slow, gentle speed for driving into the hard
                             # stop -- much slower than TARGET_SPEED_MM_S
HOME_SETTLE_S = 0.5          # extra time held against the stop before
                             # re-zeroing, so the stall is unambiguous
RECOVER_FROM_STALL = True    # if a move times out mid-cycle (see
                             # wait_until_arrived), re-home before trusting
                             # position again rather than continuing blind

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


def home_actuator(tic, microstep_divisor):
    """Re-establish a known zero position by driving into the actuator's
    physical retracted hard stop and re-zeroing there.

    This deliberately commands a target well beyond the actuator's real
    travel, at reduced speed, so it is guaranteed to hit the hard stop and
    stall out before the commanded distance is reached. We don't (and can't)
    wait for the Tic to report arrival -- that would never happen, by
    design -- we just wait long enough for the stall to certainly have
    happened, plus a short settle, then call halt_and_set_position(0) to
    treat wherever the actuator physically is right now as the new zero.

    Note this overrides the max speed set by configure_motion(); call
    configure_motion() again afterward to restore the normal move profile
    before commanding further moves.
    """
    mm_per_step = FULL_STEP_MM / microstep_divisor
    home_target_steps = -int(round(HOME_OVERTRAVEL_MM / mm_per_step))
    home_speed_steps_per_sec = HOME_SPEED_MM_S / mm_per_step

    tic.set_max_speed(int(home_speed_steps_per_sec * 10000))
    tic.set_target_position(home_target_steps)

    max_travel_time_s = HOME_OVERTRAVEL_MM / HOME_SPEED_MM_S
    time.sleep(max_travel_time_s + HOME_SETTLE_S)

    tic.halt_and_set_position(0)
    print("Homed: re-zeroed against the retracted hard stop.")


def wait_until_arrived(tic, timeout=MOVE_TIMEOUT_S):
    elapsed = 0.0
    while tic.get_current_position() != tic.get_target_position():
        # A long move can otherwise trip the Tic's own command-timeout
        # watchdog, since it only saw one real command (set_target_position)
        # at the very start -- this tells it the host is still present.
        tic.reset_command_timeout()
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

    try:
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

        steps = configure_motion(tic, microstep_divisor)
        print(f"Microstepping: 1/{microstep_divisor}, {steps} steps per {TRAVEL_MM}mm move, "
              f"target speed {TARGET_SPEED_MM_S} mm/s")

        # Replaces the old "assume you started from a known-safe position"
        # halt_and_set_position(0) -- this actually re-establishes zero
        # against a physical reference instead of just trusting wherever the
        # actuator happened to be when the script started.
        print("Homing: driving into the retracted hard stop to establish a known zero...")
        home_actuator(tic, microstep_divisor)
        steps = configure_motion(tic, microstep_divisor)  # restore normal speed/accel after homing's slower profile

        for cycle in range(1, CYCLES + 1):
            print(f"Cycle {cycle}: extending")
            if not move_to(tic, steps):
                if not RECOVER_FROM_STALL:
                    break
                print("  Move may have stalled -- re-homing to recover a known position.")
                home_actuator(tic, microstep_divisor)
                steps = configure_motion(tic, microstep_divisor)
                continue
            time.sleep(0.5)

            print(f"Cycle {cycle}: retracting")
            if not move_to(tic, 0):
                if not RECOVER_FROM_STALL:
                    break
                print("  Move may have stalled -- re-homing to recover a known position.")
                home_actuator(tic, microstep_divisor)
                steps = configure_motion(tic, microstep_divisor)
                continue
            time.sleep(0.5)

    finally:
        tic.deenergize()
        tic.enter_safe_start()


if __name__ == "__main__":
    # Set to 1, 2, 4, 8, or 16 for full, half, 1/4, 1/8, or 1/16 step
    main(microstep_divisor=16)