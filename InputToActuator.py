#!/usr/bin/env python3
"""
Interactively jog the Actuonix S20-100-38-B linear actuator through a
Pololu Tic T825 USB stepper motor controller, with selectable microstepping
and the Tic's built-in acceleration ramping.

After homing, type a distance in mm and press Enter to move by that much
relative to the current position (e.g. 40, then 10, then -15). Press Ctrl+C
(or Ctrl+D) to stop and exit.

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
TARGET_SPEED_MM_S = 6.0     # real-world speed target, held constant across
                            # whichever microstep resolution you pick
RAMP_TIME_S = 0.3           # time to accelerate from starting speed to target speed

MIN_POSITION_MM = 0.0       # 0 = the homed (retracted hard stop) position
MAX_POSITION_MM = 100.0      # keep well inside the actuator's real end-of-travel
                             # until confirmed safe; raise once you trust it.
                             # Requested moves that would land outside
                             # [MIN_POSITION_MM, MAX_POSITION_MM] are refused.

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
HOME_OVERTRAVEL_MM = 5.0    # commanded distance past the actuator's real
                             # travel when homing -- guarantees it reaches
                             # and stalls against the physical retracted end
                             # stop well before the commanded distance
HOME_SPEED_MM_S = 1.0        # slow, gentle speed for driving into the hard
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
    tic.exit_safe_start()  # see move_to() -- guards against a command-timeout
                            # having re-armed safe-start while we were idle
                            # sitting at the interactive prompt
    tic.set_target_position(home_target_steps)

    # Deliberately not a single time.sleep() -- a long uninterrupted sleep
    # leaves nobody feeding the Tic's command-timeout watchdog, which can
    # trip and halt the motor early, well before it actually reaches the
    # hard stop. Poll in small steps and ping reset_command_timeout(), the
    # same way wait_until_arrived() does for a normal move.
    max_travel_time_s = HOME_OVERTRAVEL_MM / HOME_SPEED_MM_S
    elapsed = 0.0
    while elapsed < max_travel_time_s + HOME_SETTLE_S:
        tic.reset_command_timeout()
        time.sleep(0.05)
        elapsed += 0.05

    tic.halt_and_set_position(0)

    # halt_and_set_position() re-arms the Tic's safe-start interlock (this is
    # by design -- any halt requires a fresh exit_safe_start() before the Tic
    # will move again). Without this, the next move command is silently
    # ignored and wait_until_arrived() times out reporting a safe start
    # violation (status bit 0x0080), even though homing itself succeeded.
    tic.exit_safe_start()

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
    # Sitting idle at the interactive prompt between moves can be long
    # enough for the Tic's command-timeout watchdog to trip on its own,
    # which (like halt_and_set_position()) re-arms the safe-start
    # interlock. Calling exit_safe_start() here is a no-op if it wasn't
    # needed, and clears the timeout condition either way since it's a
    # valid command.
    tic.exit_safe_start()
    tic.set_target_position(position)
    return wait_until_arrived(tic)


def jog_loop(tic, microstep_divisor):
    """Read distances in mm from the terminal and move by that much relative
    to the current position, until Ctrl+C or Ctrl+D."""
    mm_per_step = FULL_STEP_MM / microstep_divisor
    position_mm = 0.0  # matches the zero home_actuator() just established

    print(f"Ready. Enter a move distance in mm, relative to the current "
          f"position (e.g. 40, 10, -15), and press Enter. Allowed range: "
          f"{MIN_POSITION_MM:g} to {MAX_POSITION_MM:g}mm. Ctrl+C to quit.")

    while True:
        try:
            raw = input(f"[{position_mm:.2f}mm] move (mm)> ")
        except EOFError:
            print()
            return

        raw = raw.strip()
        if not raw:
            continue

        try:
            delta_mm = float(raw)
        except ValueError:
            print(f"  Not a number: {raw!r}")
            continue

        target_mm = position_mm + delta_mm
        if not (MIN_POSITION_MM <= target_mm <= MAX_POSITION_MM):
            print(f"  Skipped: {target_mm:.2f}mm would be outside the allowed "
                  f"{MIN_POSITION_MM:g}-{MAX_POSITION_MM:g}mm range "
                  f"(currently at {position_mm:.2f}mm).")
            continue

        target_steps = int(round(target_mm / mm_per_step))
        print(f"  Moving {delta_mm:+.2f}mm -> {target_mm:.2f}mm ({target_steps} steps)")

        if move_to(tic, target_steps):
            position_mm = target_mm
        elif RECOVER_FROM_STALL:
            print("  Move may have stalled -- re-homing to recover a known position.")
            home_actuator(tic, microstep_divisor)
            configure_motion(tic, microstep_divisor)  # restore normal speed/accel after homing's slower profile
            position_mm = 0.0
        else:
            print("  Move failed; position is now uncertain. Consider re-homing.")


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

        configure_motion(tic, microstep_divisor)
        print(f"Microstepping: 1/{microstep_divisor}, target speed {TARGET_SPEED_MM_S} mm/s")

        # Replaces the old "assume you started from a known-safe position"
        # halt_and_set_position(0) -- this actually re-establishes zero
        # against a physical reference instead of just trusting wherever the
        # actuator happened to be when the script started.
        print("Homing: driving into the retracted hard stop to establish a known zero...")
        home_actuator(tic, microstep_divisor)
        configure_motion(tic, microstep_divisor)  # restore normal speed/accel after homing's slower profile

        jog_loop(tic, microstep_divisor)

    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        tic.deenergize()
        tic.enter_safe_start()


if __name__ == "__main__":
    # Set to 1, 2, 4, 8, or 16 for full, half, 1/4, 1/8, or 1/16 step
    main(microstep_divisor=16)