import board, digitalio
from adafruit_stmpe610 import Adafruit_STMPE610_SPI

ts = Adafruit_STMPE610_SPI(board.SPI(), digitalio.DigitalInOut(board.CE1))
while True:
    if ts.touched:
        while not ts.buffer_empty:
            x, y, z = ts.touch_point
            if z > 20:
                print(to_pixels(x, y, 480, 320))