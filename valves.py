#!/usr/bin/env python3
"""
Takasago WLB-3 latching 3-way diaphragm valve, driven by a DRV8871 H-bridge.

PLUMBING (assumed -- confirm with the bench test below):
    common (IN) port -> syringe
    NO port          -> target line (heater block and nozzle)
    NC port          -> refill line (pouch)

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
pulses IN1 or IN2 on demand, so you can find which pulse opens which port and
set IN1_SETS below.
"""

import time

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

# Coil energising time per switch. ASSUMED VALUE -- check it against the WLB
# datasheet. Too short and the valve may not latch reliably; longer than
# needed only heats the coil (about 8 W while it is on).
PULSE_S = 0.05

# Wait after a switch before pushing or pulling liquid through the valve, so
# the diaphragm has seated. Check against the datasheet response time.
VALVE_SETTLE_S = 0.15

# Minimum time between two pulses, to protect a coil rated for intermittent
# duty only (someone hammering the toggle). A command that arrives sooner
# waits out the remainder, so this is also the longest extra blocking time.
MIN_PULSE_INTERVAL_S = 0.25

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
    try/finally guarantees it. Tens of milliseconds is far inside the Tic's
    command timeout.
    """

    def __init__(self, verbose=True, initial=REST_POSITION):
        """initial: position to pulse to now, or None to leave the valve
        untouched (bench test)."""
        self.verbose = verbose
        self.position = None                # unknown until the first pulse
        self._last_pulse = float("-inf")
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

    def _pulse(self, line):
        wait = self._last_pulse + MIN_PULSE_INTERVAL_S - time.monotonic()
        if wait > 0:
            time.sleep(wait)

        if not SIMULATE:
            active = self._lines[line]
            self._lines[3 - line].value = False     # the other input: 1 <-> 2
            try:
                active.value = True
                time.sleep(PULSE_S)
            finally:
                # Whatever happens -- Ctrl+C, an exception -- the coil must not
                # stay energised: a latching coil is rated for pulses only.
                active.value = False

        self._last_pulse = time.monotonic()

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
    print("After each pulse, push a little air or water into the common (IN)")
    print("port with a syringe and see whether it leaves through NO or NC.")
    print("NO goes to the target line and NC to the pouch, so set")
    print("IN1_SETS = TARGET if an IN1 pulse opens NO, or REFILL if it opens NC.")
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
