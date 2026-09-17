#!/usr/bin/env python3
"""
PiTFT 3.5" resistive (HX8357D display + STMPE610 touch) on a Raspberry Pi 3,
driven entirely from userspace over SPI -- no kernel framebuffer, no overlay,
no X/Wayland.

Requires in the venv:
    pip install Adafruit-Blinka adafruit-circuitpython-rgb-display \
                adafruit-circuitpython-stmpe610 pillow numpy

Requires in /boot/firmware/config.txt:
    dtparam=spi=on          (above any [pi4]/[pi5] filter, or under [all])
    and NO dtoverlay=pitft35-... line -- that overlay disables spidev0.0/0.1,
    which this approach needs.

Run directly for a display+touch check:
    python3 pitft.py

Or import it:
    from pitft import PiTFT
    tft = PiTFT()
    tft.draw_full(img)
    tft.draw_region(img, x, y)
    hit = tft.get_touch()
"""

import time

import board
import digitalio
import numpy  # noqa: F401  -- not called directly, but adafruit_rgb_display
              # uses it for the RGB->RGB565 packing step if it can import it,
              # which is the difference between ~0.18s and ~0.5s per full
              # frame. Imported here so a missing install fails loudly instead
              # of silently falling back to the slow pure-Python path.
from PIL import Image, ImageDraw, ImageFont

import adafruit_rgb_display.hx8357 as hx8357
from adafruit_stmpe610 import Adafruit_STMPE610_SPI


# ===========================================================================
# THE ONLY SWITCH YOU NEED TO TOUCH FOR THE ISOLATION TEST
# ===========================================================================
# False = normal operation: the readout box redraws when you press the screen.
# True  = diagnostic: the readout box redraws on a 500ms timer and the touch
#         chip is NEVER read. If the creeping red lines appear in this mode,
#         they are not caused by touch traffic on the shared SPI bus.
TIMER_ONLY = False
# ===========================================================================


# ---------------------------------------------------------------------------
# Display configuration
# ---------------------------------------------------------------------------
SPI_BAUDRATE = 24000000

# HX8357D memory-access-control register. Set once at init so mirroring and
# colour order are handled in HARDWARE -- every draw, full or partial, is then
# correct with no per-frame transform and no coordinate fiddling inside
# draw_region(). Determined empirically with madctl_test.py:
#   0x20 MV  - row/column exchange, i.e. 480x320 landscape
#   0x08 BGR - this panel wants BGR byte order, not RGB
# Do NOT also set display.rotation: that applies a second, software transform
# inside image(), which fights this one and misplaces draw_region's offsets.
MADCTL_REG = 0x36
MADCTL_VALUE = 0x28


# ---------------------------------------------------------------------------
# Touch calibration
# ---------------------------------------------------------------------------
# Measured by touching the four corners and averaging opposite edges, then
# re-verified after the MADCTL change. Raw Y is INVERTED relative to pixel Y
# (largest at the TOP of the screen); raw X runs the same direction as pixel X.
# If the panel is ever re-seated or replaced, re-run the corner check below
# rather than assuming these still hold.
RAW_X_MIN, RAW_X_MAX = 403, 3696    # left edge, right edge
RAW_Y_MIN, RAW_Y_MAX = 510, 3615    # BOTTOM edge, TOP edge

# Measured pressure on a deliberate press was 51-77. Anything well below that
# is noise or the tail end of a release.
TOUCH_PRESSURE_MIN = 20

# The resistive panel is a few percent non-rectangular (the left edge read 408
# at the top but 399 at the bottom). A linear map is therefore a handful of
# pixels out near the edges -- irrelevant for large buttons, so no affine
# correction here.


class PiTFT:
    """Owns both SPI devices on the PiTFT: the HX8357D display on CE0 and the
    STMPE610 touch controller on CE1. Both must be owned by the SAME process.

    Construction order matters: the touch chip must be created BEFORE the
    display. Creating it afterwards disturbs the display's initialisation
    sequence and leaves the panel blank.
    """

    def __init__(self):
        spi = board.SPI()

        # --- touch first, see docstring ---
        self.touch = Adafruit_STMPE610_SPI(spi, digitalio.DigitalInOut(board.CE1))

        # --- then the display ---
        self.display = hx8357.HX8357(
            spi,
            cs=digitalio.DigitalInOut(board.CE0),
            dc=digitalio.DigitalInOut(board.D25),
            rst=None,                      # not wired to a GPIO on the PiTFT
            baudrate=SPI_BAUDRATE,
        )
        self.display.write(MADCTL_REG, bytes([MADCTL_VALUE]))

        # Take the size from the driver rather than hardcoding 480x320.
        self.width = self.display.width
        self.height = self.display.height

    # -- drawing ------------------------------------------------------------

    def draw_full(self, img):
        """Push a full-screen image. ~0.18s on a Pi 3 with numpy present."""
        self.display.image(img)

    def draw_region(self, img, x, y):
        """Push a small image to a sub-rectangle, leaving the rest of the
        screen untouched.

        Cost is proportional to AREA, not to the call: a 200x50 box is 10,000
        of the screen's 153,600 pixels, about 6.5%, so ~12ms against ~180ms
        for a full frame. Keep changing regions small and draw the static
        furniture (buttons, labels) once with draw_full().
        """
        self.display.image(img, x=x, y=y)

    def blank(self, colour="black"):
        self.draw_full(Image.new("RGB", (self.width, self.height), colour))

    # -- touch --------------------------------------------------------------

    def get_touch(self):
            """Return (x, y) in pixels for the most recent press, or None.

            Draining on RELEASE matters as much as draining on press: the STMPE610
            keeps buffering through the release, and those trailing samples would
            otherwise be read at the start of the next press and reported as its
            coordinates -- which shows up as the previously-pressed button
            activating instead of the intended one.
            """
            if not self.touch.touched:
                while not self.touch.buffer_empty:
                    self.touch.touch_point      # discard release stragglers
                return None

            samples = []
            while not self.touch.buffer_empty:
                raw_x, raw_y, pressure = self.touch.touch_point
                if pressure >= TOUCH_PRESSURE_MIN:
                    samples.append((raw_x, raw_y))

            if not samples:
                return None

            # Median rather than last: resistive panels give unsettled coordinates
            # as pressure ramps up, and a median discards those outliers without
            # needing a settling delay.
            samples.sort()
            raw_x, raw_y = samples[len(samples) // 2]
            return self._to_pixels(raw_x, raw_y)

    def _to_pixels(self, raw_x, raw_y):
        px = (raw_x - RAW_X_MIN) / (RAW_X_MAX - RAW_X_MIN) * self.width
        py = (RAW_Y_MAX - raw_y) / (RAW_Y_MAX - RAW_Y_MIN) * self.height  # inverted
        return (
            min(max(int(px), 0), self.width - 1),
            min(max(int(py), 0), self.height - 1),
        )

    def close(self):
        """Leave the panel dark on exit so a half-drawn UI isn't left sitting
        there looking live."""
        try:
            self.blank()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------------

# Deliberately small: area is what costs time. 200x50 ~= 12ms.
READOUT_W, READOUT_H = 200, 50


def build_background(width, height, font):
    """Static furniture, drawn once. Corner ticks let you eyeball the touch
    calibration: press each tick and check the reported pixels."""
    bg = Image.new("RGB", (width, height), (0, 0, 0))
    d = ImageDraw.Draw(bg)
    d.rectangle((0, 0, width - 1, height - 1), outline=(80, 80, 80))
    d.text((20, 20), "PiTFT check -- press the corner ticks", font=font, fill="white")
    for cx, cy in ((0, 0), (width - 1, 0), (0, height - 1), (width - 1, height - 1)):
        d.line((cx, cy, cx, cy + (20 if cy == 0 else -20)), fill=(255, 0, 0))
        d.line((cx, cy, cx + (20 if cx == 0 else -20), cy), fill=(255, 0, 0))
    return bg


def build_readout(text, subtext, font):
    box = Image.new("RGB", (READOUT_W, READOUT_H), (0, 0, 60))
    d = ImageDraw.Draw(box)
    d.rectangle((0, 0, READOUT_W - 1, READOUT_H - 1), outline=(120, 120, 255))
    d.text((12, 10), text, font=font, fill="white")
    d.text((12, 30), subtext, font=font, fill=(160, 160, 160))
    return box


def main():
    tft = PiTFT()
    print(f"display reports {tft.width}x{tft.height}")

    # get_version is a property in this library version, not a method.
    version = tft.touch.get_version
    if callable(version):
        version = version()
    print("touch chip id:", hex(version))

    font = ImageFont.load_default()

    t0 = time.monotonic()
    tft.draw_full(build_background(tft.width, tft.height, font))
    print(f"full-frame redraw: {time.monotonic() - t0:.3f}s")

    readout_x = (tft.width - READOUT_W) // 2
    readout_y = (tft.height - READOUT_H) // 2

    count = 0
    last = None
    # A deadline compared against a fast loop, rather than time.sleep(0.5).
    # The loop stays free to do other work -- which is exactly what a STOP
    # button needs while a move is running.
    next_redraw = time.monotonic()

    if TIMER_ONLY:
        print("TIMER_ONLY: redrawing every 500ms, touch chip never read.")
    else:
        print("Press the corner ticks and check the reported pixels.")
    print("Ctrl+C to quit.")

    try:
        while True:
            if TIMER_ONLY:
                now = time.monotonic()
                if now >= next_redraw:
                    next_redraw = now + 0.5
                    count += 1
                    box = build_readout(f"tick {count}", "no touch reads", font)
                    t0 = time.monotonic()
                    tft.draw_region(box, readout_x, readout_y)
                    print(f"tick {count}  partial redraw "
                          f"{(time.monotonic() - t0) * 1000:.0f}ms")
            else:
                hit = tft.get_touch()
                if hit and hit != last:
                    count += 1
                    last = hit
                    box = build_readout(f"x={hit[0]}  y={hit[1]}",
                                        f"presses: {count}", font)
                    t0 = time.monotonic()
                    tft.draw_region(box, readout_x, readout_y)
                    print(f"{hit}  partial redraw "
                          f"{(time.monotonic() - t0) * 1000:.0f}ms")

            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\nstopping")
    finally:
        tft.close()


if __name__ == "__main__":
    main()