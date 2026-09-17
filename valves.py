#!/usr/bin/env python3
"""
Solenoid valve control -- PLACEHOLDER.

Nothing here touches hardware yet. Every call prints what it would do and
returns immediately, so the injection cycle can be written, run and debugged
in full before the solenoids exist.

TO MAKE THIS REAL:
    1. Decide which GPIO pins drive the two valve channels. Note that GPIO7,
       GPIO8, GPIO24 and GPIO25 are already taken by the PiTFT (chip selects
       and D/C), and GPIO9/10/11 are the SPI bus itself. Free candidates on
       the PiTFT's unused header pins include GPIO5, GPIO6, GPIO12, GPIO13,
       GPIO16, GPIO19, GPIO20, GPIO21, GPIO26.
    2. A solenoid CANNOT be driven from a GPIO pin directly -- it needs a
       MOSFET or relay driver board, plus a flyback diode across the coil, and
       its own supply. A coil's collapsing field will destroy a GPIO pin.
    3. Replace the bodies of _set() with the actual pin writes, e.g.
           import digitalio, board
           self._target = digitalio.DigitalInOut(board.D5)
           self._target.switch_to_output(value=False)
    4. Check VALVE_SETTLE_S against the real valves' datasheet response time.

The interface below is what the injection cycle calls. Keeping it stable means
swapping the placeholder for real hardware touches only this file.
"""

import time


# How long to wait after commanding a valve before assuming it has actually
# moved. Solenoids take milliseconds to tens of milliseconds to seat, and
# pushing fluid against a valve that has not finished opening is how you build
# pressure where you did not intend to. Verify against the real datasheet.
VALVE_SETTLE_S = 0.15

TARGET = "target"
REFILL = "refill"


class Valves:
    """Two-channel valve manifold: one line to the target container, one to
    the refill reservoir.

    Only one should ever be open at a time. The class enforces that rather
    than trusting the caller, because both open at once means the refill
    reservoir is connected straight to the target.
    """

    def __init__(self, verbose=True):
        self.verbose = verbose
        self.state = {TARGET: False, REFILL: False}
        # TODO: claim the GPIO pins here once the driver board exists.

    def _set(self, channel, value):
        # TODO: replace with the real pin write.
        self.state[channel] = value
        if self.verbose:
            print(f"  [valve] {channel} -> {'OPEN' if value else 'CLOSED'}")

    def open(self, channel):
        # Close the other line first. Never both open.
        other = REFILL if channel == TARGET else TARGET
        if self.state[other]:
            self._set(other, False)
        self._set(channel, True)

    def close(self, channel):
        self._set(channel, False)

    def close_all(self):
        for channel in (TARGET, REFILL):
            if self.state[channel]:
                self._set(channel, False)

    @property
    def any_open(self):
        return any(self.state.values())


if __name__ == "__main__":
    v = Valves()
    print("placeholder valve exercise:")
    v.open(TARGET)
    time.sleep(VALVE_SETTLE_S)
    v.open(REFILL)          # should close TARGET first
    time.sleep(VALVE_SETTLE_S)
    v.close_all()