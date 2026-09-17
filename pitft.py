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
    tft.draw_region(img, x, y)
    hit = tft.get_touch()
"""

import time

import board
import digitalio
import numpy  # noqa: F401  -- not called directly, but adafruit_rgb_display
              # uses it for the RGB->RGB565 packing step if it can import it,
              # which is the difference between ~0.5s and ~0.1s per full frame.
              # Imported here so a missing install fails loudly instead of
              # silently falling back to the slow pure-Python path.
from PIL import Image, ImageDraw, ImageFont

import adafruit_rgb_display.hx8357 as hx8357
from adafruit_stmpe610 import Adafruit_STMPE610_SPI

# ---------------------------------------------------------------------------
# Touch calibration
# ---------------------------------------------------------------------------
# Measured by touching the four corners and averaging opposite edges.
# Note RAW_Y is INVERTED relative to pixel Y: the raw value is largest at the
# TOP of the screen and smallest at the bottom. RAW_X runs the same direction
# as pixel X. If you ever re-seat or replace the panel, re-run the corner
# check (see main() below) rather than assuming these still hold.
RAW_X_MIN, RAW_X_MAX = 403, 3696    # left edge, right edge
RAW_Y_MIN, RAW_Y_MAX = 510, 3615    # BOTTOM edge, TOP edge

# Measured pressure on a deliberate press was 51-77. Anything well below that
# is noise or the tail end of a release.
TOUCH_PRESSURE_MIN = 20

# The resistive panel is a few percent non-rectangular (the left edge read 408
# at the top but 399 at the bottom). A linear map is therefore a handful of
# pixels out near the edges -- irrelevant for large buttons, so no affine
# correction here.

SPI_BAUDRATE = 24000000


class PiTFT:
    """Owns both SPI devices on the PiTFT: the HX8357D display on CE0 and the
    STMPE610 touch controller on CE1.

    Both must be owned by the SAME process. The display's chip-select has to
    be actively driven high whenever traffic goes to the touch chip; if CE0 is
    left floating (which is what happens when a display-owning process exits,
    or if a touch-only script never claims CE0), the HX8357D also latches the
    touch traffic and writes it into its own frame memory. That shows up as
    the screen slowly filling with garbage line by line -- it looks like a
    hardware fault but is only bus contention.
    """

    def __init__(self, rotation=0):
        spi = board.SPI()

        # The touch chip tolerates far less SPI speed than the display; the
        # library sets its own baudrate when it locks the bus, so the two
        # coexist on SPI0 without interfering.
        self.touch = Adafruit_STMPE610_SPI(spi, digitalio.DigitalInOut(board.CE1))
 
        self.display = hx8357.HX8357(
            spi,
            cs=digitalio.DigitalInOut(board.CE0),
            dc=digitalio.DigitalInOut(board.D25),
            rst=None,                      # not wired to a GPIO on the PiTFT
            baudrate=SPI_BAUDRATE,
        )
        self.display.rotation = rotation
 
        # Take the size from the driver rather than hardcoding 480x320, so
        # this stays correct if the rotation is ever changed.
        self.width = self.display.width
        self.height = self.display.height
 
    # -- drawing ------------------------------------------------------------
 

    def draw_full(self, img):
        """Push a full-screen image. Roughly 0.1s with numpy present."""
        self.display.image(img)

    def draw_region(self, img, x, y):
        """Push a small image to a sub-rectangle, leaving the rest of the
        screen untouched. This is the whole trick for a responsive UI: a
        200x60 readout is about 2% of the frame, so it costs ~2% of the time.
        Draw the static furniture (buttons, labels) once with draw_full(),
        then only ever repaint the parts that actually change."""
        self.display.image(img, x=x, y=y)

    def blank(self, colour="black"):
        self.draw_full(Image.new("RGB", (self.width, self.height), colour))

    # -- touch --------------------------------------------------------------

    def get_touch(self):
        """Return (x, y) in pixels for the most recent press, or None.

        The STMPE610 buffers samples, so a single press yields several. We
        drain the buffer and keep the last valid sample -- draining matters,
        because a buffer left full makes the next press read as stale
        coordinates from the previous one.
        """
        if not self.touch.touched:
            return None

        point = None
        while not self.touch.buffer_empty:
            raw_x, raw_y, pressure = self.touch.touch_point
            if pressure >= TOUCH_PRESSURE_MIN:
                point = self._to_pixels(raw_x, raw_y)
        return point

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

READOUT_W, READOUT_H = 300, 90


def main():
    tft = PiTFT()
    print(f"display reports {tft.width}x{tft.height}")

    #version = tft.touch.get_version
    #if callable(version):
    #    version = version()
    #print("touch chip id:", hex(version), "(expect 0x811)")
    
    font = ImageFont.load_default()

    # --- static background, drawn once ---
    bg = Image.new("RGB", (tft.width, tft.height), (0, 0, 0))
    d = ImageDraw.Draw(bg)
    d.rectangle((0, 0, tft.width - 1, tft.height - 1), outline=(80, 80, 80))
    d.text((20, 20), "PiTFT check -- press anywhere", font=font, fill="white")
    # Corner ticks to eyeball the calibration against.
    for cx, cy in ((0, 0), (tft.width - 1, 0), (0, tft.height - 1),
                   (tft.width - 1, tft.height - 1)):
        d.line((cx, cy, cx, cy + (20 if cy == 0 else -20)), fill=(255, 0, 0))
        d.line((cx, cy, cx + (20 if cx == 0 else -20), cy), fill=(255, 0, 0))

    t0 = time.monotonic()
    tft.draw_full(bg)
    print(f"full-frame redraw: {time.monotonic() - t0:.3f}s")

    readout_x = (tft.width - READOUT_W) // 2
    readout_y = (tft.height - READOUT_H) // 2
    presses = 0
    last = None

    print("Press the corners and check the reported pixels. Ctrl+C to quit.")
    try:
        while True:
            hit = None #tft.get_touch()
            if hit and hit != last:
                presses += 1
                last = hit

                # Only this 300x90 box is redrawn, not the whole screen.
                box = Image.new("RGB", (READOUT_W, READOUT_H), (0, 0, 60))
                bd = ImageDraw.Draw(box)
                bd.rectangle((0, 0, READOUT_W - 1, READOUT_H - 1),
                             outline=(120, 120, 255))
                bd.text((15, 20), f"x={hit[0]}  y={hit[1]}", font=font, fill="white")
                bd.text((15, 50), f"presses: {presses}", font=font, fill=(160, 160, 160))

                t0 = time.monotonic()
                tft.draw_region(box, readout_x, readout_y)
                dt = time.monotonic() - t0

                print(f"{hit}  partial redraw {dt * 1000:.0f}ms")

            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\nstopping")
    finally:
        tft.close()


if __name__ == "__main__":
    main()