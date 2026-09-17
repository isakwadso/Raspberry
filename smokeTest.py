#!/usr/bin/env python3
import board, digitalio
from PIL import Image, ImageDraw
import adafruit_rgb_display.hx8357 as hx8357

spi = board.SPI()
display = hx8357.HX8357(
    spi,
    cs=digitalio.DigitalInOut(board.CE0),   # PiTFT display CS
    dc=digitalio.DigitalInOut(board.D25),   # PiTFT D/C
    rst=None,                               # not wired to a GPIO on the PiTFT
    baudrate=24000000,
)

w, h = display.width, display.height
print("panel reports", w, "x", h)

img = Image.new("RGB", (w, h), "red")
d = ImageDraw.Draw(img)
d.rectangle((20, 20, w - 20, h - 20), fill="black")
d.text((40, 40), "PiTFT alive", fill="white")
display.image(img)