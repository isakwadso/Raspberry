import board, digitalio
from adafruit_stmpe610 import Adafruit_STMPE610_SPI

tft_cs = digitalio.DigitalInOut(board.CE0)
tft_cs.switch_to_output(value=True)   # park display CS inactive

ts = Adafruit_STMPE610_SPI(board.SPI(), digitalio.DigitalInOut(board.CE1))