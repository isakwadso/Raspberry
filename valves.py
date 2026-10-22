#!/usr/bin/env python3
"""
Takasago WLB-3 latching 3-way diaphragm valve, driven by a DRV8871 H-bridge.

PLUMBING (datasheet port letters):
    A (common) -> syringe
    B and C    -> one to the target line (heater block and nozzle), the other
                  to the refill line (pouch). Which is which is set by the
                  plumbing; the bench test below tells the code.

DATASHEET (WLB-3R(K)-N4F(G)):
    energising time 0.2-1 s, duty cycle 10 % or less (5 min max on),
    rated 12 or 24 V DC, 8 W, media and ambient 5-50 C.
    12 V lead colours and resulting port states:
        +12 V on BLUE, GND on RED  -> C opens, B closes   (status "X")
        +12 V on RED,  GND on BLUE -> B opens, C closes   (status "Y")
    An IN1 pulse drives OUT1 high and OUT2 low. So with BLUE on OUT1 and RED
    on OUT2, IN1 opens C; with the leads the other way round, IN1 opens B.

THE VALVE HAS NO CLOSED STATE. The syringe is always connected to exactly one
of the two lines, so the interface is a position, TARGET or REFILL, not
open/close. The pouch and the target line can never be connected to each
other: the valve body itself guarantees it.

LATCHING. A short current pulse in one direction moves the valve to one
position, and a permanent magnet holds it there with the power off. A pulse
in the other direction moves it back. Two consequences:
  * There is no position feedback. `position` is what was last commanded.
  * After a restart the real position is unknown, so the constructor pulses
    the valve to REST_POSITION. Every command pulses, even if the valve should
    already be there: re-pulsing a latched valve is harmless, and it corrects
    a valve that was moved while the program was not running.

Only switch the valve while the plunger is still. Switching while liquid is
moving pushes it into (or pulls it from) the wrong line. jog_ui.py enforces
this; anything else that uses this module must do the same.

WIRING
    GPIO17 (PiTFT header pin 11) -> DRV8871 IN1
    GPIO27 (PiTFT header pin 13) -> DRV8871 IN2
    GND    (PiTFT header pin 9)  -> DRV8871 logic GND
    DRV8871 VM / GND             -> 12 V supply (shared with the Tic), with its
                                    own ground wire back to the supply
    DRV8871 OUT1 / OUT2          -> valve coil
No flyback diode: the DRV8871's body diodes return the coil energy to VM, and
a diode across the coil would short the reverse pulse.

DRV8871 truth table, IN1 IN2:
    0 0  coast (off)     1 0  OUT1 -> OUT2     0 1  OUT2 -> OUT1     1 1  brake
Both inputs have pull-downs inside the driver, and GPIO17/27 are pulled down
while the Pi boots, so the coil stays off until this module takes the pins.

A missing 12 V supply cannot be detected here (no feedback), but the Tic runs
from the same supply and reports Low VIN, which stops jog_ui.py at startup.

BENCH TEST
    python3 valves.py
pulses IN1 or IN2 on demand, so you can find which pulse opens which line and
set IN1_SETS below.
"""

import time
from collections import deque

import pins


TARGET = "target"
REFILL = "refill"
POSITIONS = (TARGET, REFILL)


# ===========================================================================
# CONFIGURATION
# ===========================================================================
# True: print only, never touch the GPIO. For running the UI on a desk without
# the valve or the driver board.
SIMULATE = False

# Coil energising time per switch. The datasheet range is 0.2-1 s; 0.3 s gives
# margin over the minimum without heating the coil more than needed (8 W while
# it is on). If the valve ever fails to switch reliably, raise this towards
# 0.5 s before suspecting anything else.
PULSE_S = 0.3

# Wait after a switch before pushing or pulling liquid through the valve, so
# the diaphragm has seated. The valve moves during the pulse itself; this is
# extra margin.
VALVE_SETTLE_S = 0.15

# Minimum time between two pulses, so a double tap does not fire two pulses
# back to back. A command that arrives sooner waits out the remainder.
MIN_PULSE_INTERVAL_S = 0.25

# Duty-cycle limit from the datasheet: the coil may be on at most 10 % of the
# time. Checked over a rolling window. A normal inject cycle uses about
# 3 x 0.3 s = 0.9 s per minute (1.5 %), so only rapid manual toggling can
# reach the limit; the UI then refuses the toggle until the coil has cooled.
DUTY_MAX = 0.10
DUTY_WINDOW_S = 60.0

# Which position a pulse on IN1 gives (current OUT1 -> OUT2 through the coil).
# ASSUMPTION until the bench test. If it is wrong, change it here or swap the
# two coil wires -- nothing is damaged meanwhile, the positions are just
# swapped.
IN1_SETS = TARGET

# Where the valve sits when nothing is happening, and where startup, abort and
# shutdown leave it. TARGET isolates the pouch, so taking the syringe off
# cannot drain the pouch through the open common port.
REST_POSITION = TARGET
# ===========================================================================


class Valves:
    """The one 3-way valve. The class name is kept from the placeholder era.

    Note that set_position() BLOCKS for PULSE_S (plus up to
    MIN_PULSE_INTERVAL_S if the previous pulse was very recent). That is a
    deliberate exception to the non-blocking rule elsewhere: the pulse must
    end even if the program is interrupted, and a blocking pulse inside
    try/finally guarantees it. 0.3 s (0.55 s at most) stays inside the Tic's
    command timeout (1 s by default) -- but poll the actuator right before
    switching, as jog_ui.py does, so the timeout starts fresh.

    Commands from the program's own sequences (cycle, homing, abort,
    shutdown) always pulse, so the valve always reaches a safe position.
    Manual controls should check duty_ok() first.
    """

    def __init__(self, verbose=True, initial=REST_POSITION):
        """initial: position to pulse to now, or None to leave the valve
        untouched (bench test)."""
        self.verbose = verbose
        self.position = None                # unknown until the first pulse
        self._last_pulse = float("-inf")
        self._history = deque()             # (start, end) of recent pulses
        self._lines = {}                    # 1 -> IN1 pin, 2 -> IN2 pin

        if not SIMULATE:
            # Imported here so SIMULATE works on a machine without Blinka.
            import board
            import digitalio
            for n, gpio in ((1, pins.VALVE_IN1), (2, pins.VALVE_IN2)):
                line = digitalio.DigitalInOut(getattr(board, f"D{gpio}"))
                line.switch_to_output(value=False)      # coast from the start
                self._lines[n] = line

        if initial is not None:
            self.set_position(initial)

    # -- commands -----------------------------------------------------------

    def set_position(self, position):
        """Pulse the valve to TARGET or REFILL. Always pulses -- see the
        module docstring. Only call while the plunger is still."""
        if position not in POSITIONS:
            raise ValueError(f"unknown valve position {position!r}")
        line = 1 if position == IN1_SETS else 2

        self.position = None        # unknown if the pulse fails partway
        self._pulse(line)
        self.position = position

        if self.verbose:
            sim = " (simulated)" if SIMULATE else ""
            print(f"  [valve] -> {position.upper()}  IN{line} pulse{sim}")

    def to_rest(self):
        self.set_position(REST_POSITION)

    # -- duty cycle -----------------------------------------------------------

    def _on_time_in_window(self):
        cutoff = time.monotonic() - DUTY_WINDOW_S
        while self._history and self._history[0][1] < cutoff:
            self._history.popleft()
        return sum(end - max(start, cutoff) for start, end in self._history)

    def duty_ok(self):
        """True if one more pulse keeps the coil within its rated duty cycle
        (DUTY_MAX over the last DUTY_WINDOW_S)."""
        return self._on_time_in_window() + PULSE_S <= DUTY_MAX * DUTY_WINDOW_S

    # -- the pulse ------------------------------------------------------------

    def _pulse(self, line):
        wait = self._last_pulse + MIN_PULSE_INTERVAL_S - time.monotonic()
        if wait > 0:
            time.sleep(wait)

        if not self.duty_ok() and self.verbose:
            # Sequences still pulse (see the class docstring); one pulse over
            # the budget does no harm, a pattern of them would.
            print("  [valve] WARNING: coil duty cycle above "
                  f"{DUTY_MAX:.0%} over the last {DUTY_WINDOW_S:.0f} s")

        start = time.monotonic()
        if SIMULATE:
            end = start + PULSE_S           # count it as if it had happened
        else:
            active = self._lines[line]
            self._lines[3 - line].value = False     # the other input: 1 <-> 2
            try:
                active.value = True
                time.sleep(PULSE_S)
            finally:
                # Whatever happens -- Ctrl+C, an exception -- the coil must not
                # stay energised: a latching coil is rated for pulses only.
                active.value = False
            end = time.monotonic()

        self._history.append((start, end))
        self._last_pulse = end

    # -- shutdown -----------------------------------------------------------

    def close(self):
        """Release the pins with both driver inputs low (coast). Does NOT move
        the valve: call to_rest() first if it should be left at rest. Once the
        pins are released, the driver's own pull-downs keep it off."""
        for line in self._lines.values():
            try:
                line.value = False
                line.deinit()
            except Exception:
                pass
        self._lines = {}


# ---------------------------------------------------------------------------
# Bench test
# ---------------------------------------------------------------------------

def bench_test():
    print("Valve bench test: 1 = pulse IN1, 2 = pulse IN2, q = quit.")
    print("After each pulse, push a little air or water into the common port A")
    print("with a syringe and see which line it leaves through. Then set")
    print("IN1_SETS = TARGET if an IN1 pulse opens the line to the nozzle,")
    print("or REFILL if it opens the line to the pouch.")
    print(f"Pulse length {PULSE_S * 1000:.0f} ms.")
    if SIMULATE:
        print("SIMULATE is True -- nothing will move.")

    v = Valves(initial=None)
    try:
        while True:
            cmd = input("> ").strip().lower()
            if cmd in ("q", "quit"):
                break
            if cmd in ("1", "2"):
                if not v.duty_ok():
                    print("  coil duty cycle limit reached -- wait a little")
                    continue
                v._pulse(int(cmd))
                print(f"  pulsed IN{cmd}")
            elif cmd:
                print("  1, 2 or q")
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        v.close()


if __name__ == "__main__":
    bench_test()