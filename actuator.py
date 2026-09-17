#!/usr/bin/env python3
"""
Non-blocking control layer for an Actuonix S20-100 linear actuator driven by a
Pololu Tic T825 over USB.

The point of this module is that NOTHING here blocks. Every long operation
(a move, a home, a ramped stop) is started with one call and then advanced by
repeated poll() calls from the application's main loop. That is what makes a
STOP button possible: the same ~20ms loop can read the touchscreen, repaint
the display and supervise the actuator, instead of sitting inside a while-loop
waiting for the Tic to report arrival.

Typical use:

    act = Actuator(microstep_divisor=16)
    act.start_home()
    while True:
        act.poll()                      # call this EVERY loop iteration
        if some_button_pressed:
            act.start_move_relative(10.0)
        if stop_button_pressed:
            act.stop()
        time.sleep(0.02)

Requires:
    pip install ticlib pyusb
"""

import time

from ticlib import TicUSB


# ---------------------------------------------------------------------------
# Motion configuration
# ---------------------------------------------------------------------------
FULL_STEP_MM = 0.01          # Actuonix S20 datasheet: 0.01mm per full step
TARGET_SPEED_MM_S = 6.0      # real-world speed, held constant across whichever
                             # microstep resolution is selected
RAMP_TIME_S = 0.3            # accelerate from starting speed to target speed

MIN_POSITION_MM = -10000.0        # 0 = the homed (retracted hard stop) position
MAX_POSITION_MM = 10000.0      # moves that would land outside
                             # [MIN_POSITION_MM, MAX_POSITION_MM] are refused

# The STOP ramp. Deliberately much shorter than RAMP_TIME_S -- this is a safety
# control, so it should stop promptly -- but still a ramp rather than a hard
# halt. A hard halt (halt_and_hold) stops sooner but can lose steps, which
# makes the position uncertain and forces a re-home. Decelerating keeps the
# Tic's step count accurate, so after a STOP the position is still known.
STOP_RAMP_TIME_S = 0.08

# ---------------------------------------------------------------------------
# Homing configuration
# ---------------------------------------------------------------------------
# The Tic runs open-loop: it cannot tell whether a commanded move completed or
# the motor stalled partway. A stall does not error out and does not slow the
# Tic's position counter, so its belief about where the actuator is can
# silently drift from reality. The only way to regain ground truth without
# limit switches or an encoder is to deliberately drive into a known physical
# reference -- here the retracted hard stop -- and re-zero there.
#
# Make sure the actuator has clearance to retract fully before homing.
HOME_OVERTRAVEL_MM = 5.0     # commanded distance past real travel, guaranteeing
                             # the hard stop is reached and the motor stalls
HOME_SPEED_MM_S = 1.0        # slow and gentle for driving into the stop
HOME_SETTLE_S = 0.5          # held against the stop before re-zeroing

# microstep divisor -> Tic "Set step mode" protocol value
# (Pololu Tic command reference: 0=Full, 1=1/2, 2=1/4, 3=1/8, 4=1/16, 5=1/32)
STEP_MODE_VALUES = {1: 0, 2: 1, 4: 2, 8: 3, 16: 4}

# A move that takes longer than its predicted duration plus this margin is
# assumed to have stalled.
MOVE_TIMEOUT_MARGIN_S = 3.0


# ---------------------------------------------------------------------------
# States
# ---------------------------------------------------------------------------
IDLE = "idle"
MOVING = "moving"
HOMING = "homing"
STOPPING = "stopping"
ERROR = "error"
STALLED = "stalled"


def to_int(value):
    """Some ticlib getters return array.array rather than a plain int
    depending on version -- normalize either form."""
    if isinstance(value, int):
        return value
    return int.from_bytes(bytes(value), "little")


class Actuator:
    def __init__(self, microstep_divisor=16):
        if microstep_divisor not in STEP_MODE_VALUES:
            raise ValueError(f"microstep_divisor must be one of {sorted(STEP_MODE_VALUES)}")

        self.microstep_divisor = microstep_divisor
        self.mm_per_step = FULL_STEP_MM / microstep_divisor

        self.tic = TicUSB()
        self.tic.energize()
        self.tic.exit_safe_start()

        self.state = IDLE
        self.message = ""
        self.homed = False          # position is meaningless until this is True

        self._deadline = 0.0        # when the current operation must be done by
        self._target_steps = 0

        # Checked after exit_safe_start, not before: a fresh Tic always reports
        # a safe-start violation until exit_safe_start() is called, which is
        # not a real fault. Anything reported here is genuine (e.g. Low VIN if
        # the motor supply is not connected).
        error = to_int(self.tic.get_error_status())
        if error:
            self.state = ERROR
            self.message = f"Tic error at startup: {error:#06x}"

        self._configure_motion()

    # -- configuration ------------------------------------------------------

    def _configure_motion(self):
        """Normal move profile. Gives ~TARGET_SPEED_MM_S regardless of the
        microstep resolution chosen."""
        self.tic.set_step_mode(STEP_MODE_VALUES[self.microstep_divisor])

        target_sps = TARGET_SPEED_MM_S / self.mm_per_step
        starting_sps = target_sps * 0.1
        accel_sps2 = (target_sps - starting_sps) / RAMP_TIME_S

        # Tic units: speed in steps per 10000 seconds, accel/decel in steps per
        # second per 100 seconds (Pololu Tic command reference)
        self.tic.set_max_speed(int(target_sps * 10000))
        self.tic.set_starting_speed(int(starting_sps * 10000))
        self.tic.set_max_acceleration(int(accel_sps2 * 100))
        self.tic.set_max_deceleration(int(accel_sps2 * 100))

    def _set_stop_deceleration(self):
        """Steepen deceleration just for a STOP, then restore afterwards."""
        target_sps = TARGET_SPEED_MM_S / self.mm_per_step
        self.tic.set_max_deceleration(int((target_sps / STOP_RAMP_TIME_S) * 100))

    # -- position -----------------------------------------------------------

    @property
    def position_mm(self):
        """Read from the Tic's own counter rather than tracking separately in
        Python. After a ramped stop the counter is still accurate, so this
        stays correct even when a move is aborted partway."""
        return to_int(self.tic.get_current_position()) * self.mm_per_step

    @property
    def busy(self):
        return self.state in (MOVING, HOMING, STOPPING)

    # -- commands -----------------------------------------------------------

    def start_home(self):
        """Begin homing. Returns immediately; poll() advances it."""
        home_target = -int(round(HOME_OVERTRAVEL_MM / self.mm_per_step))
        home_sps = HOME_SPEED_MM_S / self.mm_per_step

        self.tic.set_max_speed(int(home_sps * 10000))
        self.tic.exit_safe_start()
        self.tic.set_target_position(home_target)

        # We cannot wait for arrival -- by design it never arrives, it stalls.
        # So we wait long enough for the stall to have certainly happened.
        travel_time = HOME_OVERTRAVEL_MM / HOME_SPEED_MM_S
        self._deadline = time.monotonic() + travel_time + HOME_SETTLE_S
        self.state = HOMING
        self.message = "homing"
        self.homed = False

    def start_move_to(self, target_mm):
        """Move to an absolute position in mm. Returns (ok, message)."""
        if self.busy:
            return False, "busy"
        if not self.homed:
            return False, "not homed"
        if not (MIN_POSITION_MM <= target_mm <= MAX_POSITION_MM):
            return False, (f"{target_mm:.2f}mm outside "
                           f"{MIN_POSITION_MM:g}-{MAX_POSITION_MM:g}mm")

        start_mm = self.position_mm
        self._target_steps = int(round(target_mm / self.mm_per_step))

        self.tic.exit_safe_start()
        self.tic.set_target_position(self._target_steps)

        distance = abs(target_mm - start_mm)
        self._deadline = (time.monotonic()
                          + distance / TARGET_SPEED_MM_S
                          + RAMP_TIME_S
                          + MOVE_TIMEOUT_MARGIN_S)
        self.state = MOVING
        self.message = f"moving to {target_mm:.2f}mm"
        return True, self.message

    def start_move_relative(self, delta_mm):
        return self.start_move_to(self.position_mm + delta_mm)

    def stop(self):
        """Ramped stop. Safe to call at any time, including when idle.

        set_target_velocity(0) makes the Tic decelerate to zero using its
        deceleration setting while keeping an accurate step count -- unlike
        halt_and_hold(), which stops immediately but can lose steps and leave
        the position uncertain.
        """
        if self.state not in (MOVING, HOMING):
            return

        self._set_stop_deceleration()
        self.tic.exit_safe_start()
        self.tic.set_target_velocity(0)

        self._deadline = time.monotonic() + STOP_RAMP_TIME_S + 0.5
        self.state = STOPPING
        self.message = "stopping"

    # -- the poll -----------------------------------------------------------

    def poll(self):
        """Advance whatever is in progress. MUST be called regularly (every
        20ms or so) whenever the Tic is energized, in every state.

        It is not optional even when idle: the Tic has a command-timeout
        watchdog that halts the motor if the host goes quiet, and halting
        re-arms the safe-start interlock, after which the next move command is
        silently ignored. reset_command_timeout() here is what keeps the Tic
        confident the host is still present.
        """
        self.tic.reset_command_timeout()

        if self.state == ERROR:
            return self.state

        error = to_int(self.tic.get_error_status())
        if error and self.state != HOMING:
            # Errors are expected during homing: stalling against the hard
            # stop is the whole point, and the Tic may flag it.
            self.state = ERROR
            self.message = f"Tic error {error:#06x} -- see `ticcmd -s`"
            return self.state

        now = time.monotonic()

        if self.state == MOVING:
            if to_int(self.tic.get_current_position()) == self._target_steps:
                self.state = IDLE
                self.message = "arrived"
            elif now > self._deadline:
                # Open-loop: the Tic cannot tell us it stalled, so a move that
                # overruns its predicted duration is the only signal we get.
                # The position is now untrustworthy.
                self.state = STALLED
                self.homed = False
                self.message = "move overran -- position lost, re-home"

        elif self.state == HOMING:
            if now > self._deadline:
                # Treat wherever the actuator physically is right now as zero.
                self.tic.halt_and_set_position(0)
                # halt_and_set_position re-arms safe-start by design; without
                # this the next move is silently ignored.
                self.tic.exit_safe_start()
                self._configure_motion()   # restore normal speed after homing's
                                           # slower profile
                self.state = IDLE
                self.homed = True
                self.message = "homed"

        elif self.state == STOPPING:
            velocity = to_int(self.tic.get_current_velocity())
            if velocity == 0 or now > self._deadline:
                # Position is still trustworthy after a ramped stop, so hand
                # control back to position mode at wherever we actually are.
                current = to_int(self.tic.get_current_position())
                self._configure_motion()   # restore normal deceleration
                self.tic.exit_safe_start()
                self.tic.set_target_position(current)
                self.state = IDLE
                self.message = f"stopped at {current * self.mm_per_step:.2f}mm"

        return self.state

    # -- shutdown -----------------------------------------------------------

    def close(self):
        try:
            self.tic.deenergize()
            self.tic.enter_safe_start()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Terminal self-test -- no screen involved
# ---------------------------------------------------------------------------

def main():
    """Exercises the non-blocking layer from the terminal. Note that the loop
    never blocks: it prints a live position while moving, which is exactly
    where the old code sat frozen inside wait_until_arrived()."""
    act = Actuator(microstep_divisor=16)
    if act.state == ERROR:
        print(act.message)
        return

    print("Homing...")
    act.start_home()

    last_report = 0.0
    try:
        while True:
            act.poll()

            now = time.monotonic()
            if act.busy and now - last_report > 0.25:
                last_report = now
                print(f"  [{act.state}] {act.position_mm:7.2f}mm")

            if act.state in (IDLE, STALLED, ERROR) and not act.busy:
                if act.state != IDLE:
                    print(f"!! {act.message}")
                    break
                break

            time.sleep(0.02)

        print(f"Ready at {act.position_mm:.2f}mm. "
              f"Enter mm to move, or 's' during a move is not possible here "
              f"(that needs the touchscreen loop). Ctrl+C to quit.")

        while True:
            raw = input(f"[{act.position_mm:.2f}mm] move (mm)> ").strip()
            if not raw:
                continue
            try:
                delta = float(raw)
            except ValueError:
                print(f"  not a number: {raw!r}")
                continue

            ok, msg = act.start_move_relative(delta)
            if not ok:
                print(f"  refused: {msg}")
                continue

            while act.busy:
                act.poll()
                now = time.monotonic()
                if now - last_report > 0.25:
                    last_report = now
                    print(f"  [{act.state}] {act.position_mm:7.2f}mm")
                time.sleep(0.02)

            print(f"  {act.message}")

    except (KeyboardInterrupt, EOFError):
        print("\nstopping")
        act.stop()
        while act.state == STOPPING:
            act.poll()
            time.sleep(0.02)
    finally:
        act.close()


if __name__ == "__main__":
    main()