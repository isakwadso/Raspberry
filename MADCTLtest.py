#!/usr/bin/env python3
"""
Find the right MADCTL value for the PiTFT's HX8357D.

MADCTL (register 0x36) controls mirroring and colour order in HARDWARE, so
fixing it here costs nothing per frame and keeps partial redraws (draw_region)
working with plain, untransformed coordinates. The alternative -- flipping and
channel-swapping every PIL image before pushing -- costs CPU on every redraw
and forces every partial redraw to transform its x coordinate too.

Run it, then touch the screen to step through the candidates. Note the value
printed when the pattern looks correct.

The pattern is deliberately asymmetric in both axes and uses pure R/G/B, so a
mirror, a rotation and a red/blue swap each look different from one another:

    +-------------------------+
    | RED  "TL"               |     RED    square = TOP LEFT
    |                         |     GREEN  square = TOP RIGHT
    |                   GREEN |     BLUE   square = BOTTOM LEFT
    |                         |     (bottom right deliberately empty)
    | BLUE                    |
    +-------------------------+

Reading the result:
  - Text mirrored / RED square on the right  -> wrong MX
  - RED square at the bottom                 -> wrong MY
  - RED square shows as BLUE                 -> wrong BGR bit
"""

import time

import board
import digitalio
from PIL import Image, ImageDraw, ImageFont

import adafruit_rgb_display.hx8357 as hx8357
from adafruit_stmpe610 import Adafruit_STMPE610_SPI

# MADCTL bit meanings, for reference when adjusting by hand:
MY  = 0x80   # mirror rows (vertical flip)
MX  = 0x40   # mirror columns (horizontal flip / the mirrored-text symptom)
MV  = 0x20   # exchange row/column -- keeps the panel in 480x320 landscape
BGR = 0x08   # send colour as BGR instead of RGB (the red/blue swap symptom)

# All eight combinations of MX/MY/BGR, with MV held set so the panel stays
# landscape. One of these is correct.
CANDIDATES = [
    MV,                 # 0x20
    MV | BGR,           # 0x28
    MV | MX,            # 0x60
    MV | MX | BGR,      # 0x68
    MV | MY,            # 0xA0
    MV | MY | BGR,      # 0xA8
    MV | MX | MY,       # 0xE0
    MV | MX | MY | BGR, # 0xE8
]

MADCTL = 0x36


def build_pattern(width, height):
    img = Image.new("RGB", (width, height), (0, 0, 0))
    d = ImageDraw.Draw(img)
    font = ImageFont.load_default()

    box = 70
    margin = 10

    # top left, pure red
    d.rectangle((margin, margin, margin + box, margin + box), fill=(255, 0, 0))
    d.text((margin + box + 15, margin + 25), "TL", font=font, fill="white")

    # top right, pure green
    d.rectangle((width - margin - box, margin, width - margin, margin + box),
                fill=(0, 255, 0))

    # bottom left, pure blue
    d.rectangle((margin, height - margin - box, margin + box, height - margin),
                fill=(0, 0, 255))

    # bottom right left empty on purpose

    d.rectangle((0, 0, width - 1, height - 1), outline=(90, 90, 90))
    return img


def main():
    spi = board.SPI()

    # Touch chip first -- initialising it after the display disturbs the
    # display's init sequence and leaves the panel blank.
    touch = Adafruit_STMPE610_SPI(spi, digitalio.DigitalInOut(board.CE1))

    display = hx8357.HX8357(
        spi,
        cs=digitalio.DigitalInOut(board.CE0),
        dc=digitalio.DigitalInOut(board.D25),
        rst=None,
        baudrate=24000000,
    )

    pattern = build_pattern(display.width, display.height)

    index = 0
    while True:
        value = CANDIDATES[index]
        display.write(MADCTL, bytes([value]))
        display.image(pattern)
        print(f"MADCTL = {value:#04x}   "
              f"(MX={'1' if value & MX else '0'} "
              f"MY={'1' if value & MY else '0'} "
              f"BGR={'1' if value & BGR else '0'})   touch to advance")

        # wait for a press, then for the release, so one touch = one step
        while not touch.touched:
            time.sleep(0.02)
        while touch.touched:
            while not touch.buffer_empty:
                touch.touch_point
            time.sleep(0.02)

        index = (index + 1) % len(CANDIDATES)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nstopping")