import board, digitalio
from adafruit_stmpe610 import Adafruit_STMPE610_SPI

RAW_X_MIN, RAW_X_MAX = 403, 3696
RAW_Y_MIN, RAW_Y_MAX = 510, 3615   # MIN is the bottom of the screen

def to_pixels(raw_x, raw_y, width, height):
    px = (raw_x - RAW_X_MIN) / (RAW_X_MAX - RAW_X_MIN) * width
    py = (RAW_Y_MAX - raw_y) / (RAW_Y_MAX - RAW_Y_MIN) * height   # inverted
    return (
        min(max(int(px), 0), width - 1),
        min(max(int(py), 0), height - 1),
    )

ts = Adafruit_STMPE610_SPI(board.SPI(), digitalio.DigitalInOut(board.CE1))
while True:
    if ts.touched:
        while not ts.buffer_empty:
            x, y, z = ts.touch_point
            if z > 20:
                print(to_pixels(x, y, 480, 320))