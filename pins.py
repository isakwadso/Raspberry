#!/usr/bin/env python3
"""
GPIO allocation for the dispenser -- the one place where pins are assigned.

All numbers are BCM GPIO numbers (what Blinka's board.Dnn and gpiozero use),
not physical header pin numbers. Every module that claims a pin imports it
from here, so a new function (heater, temperature sensors) cannot silently
land on a pin that is already in use.

ACCESS: the PiTFT covers the Pi's 40-pin header. Its 2x13 'classic Pi' header
on the underside carries pins 1-26 of the 40-pin header; GPIOs that only exist
on pins 27-40 are broken out to solder pads on the PiTFT instead.

BOOT STATE: GPIO0-8 are pulled UP while the Pi boots, GPIO9-27 are pulled
DOWN. Anything that drives a power stage and must stay off until the program
takes control (H-bridge inputs, a heater MOSFET gate) belongs on a pulled-down
pin.
"""

# ---------------------------------------------------------------------------
# Taken by the PiTFT -- do not use
# ---------------------------------------------------------------------------
# SPI0 (GPIO9/10/11), display chip select CE0 (GPIO8), touch chip select CE1
# (GPIO7), touch interrupt (GPIO24), display data/command (GPIO25).
# GPIO18 too, but only if the PiTFT's backlight PWM jumper is bridged.
PITFT_RESERVED = {7, 8, 9, 10, 11, 24, 25}

# ---------------------------------------------------------------------------
# Valve: Takasago WLB-3 latching valve through a DRV8871 H-bridge
# ---------------------------------------------------------------------------
VALVE_IN1 = 17      # header pin 11 -> DRV8871 IN1
VALVE_IN2 = 27      # header pin 13 -> DRV8871 IN2
                    # header pin 9 (GND) -> DRV8871 logic GND

# ---------------------------------------------------------------------------
# Planned, not yet claimed by any code
# ---------------------------------------------------------------------------
# HEATER_GATE = 22  # header pin 15. Pulled down at boot, so the heater stays
#                   # off until the program takes over. Fit a gate pull-down
#                   # resistor on the MOSFET board as well, so the heater is
#                   # also off if this pin is ever left floating.
#
# Temperature measurement depends on the sensor choice:
#   - 1-Wire sensors (e.g. DS18B20) default to GPIO4, header pin 7.
#   - An SPI ADC (e.g. ADS124S08) would share SPI0 with the PiTFT. Both of
#     SPI0's hardware chip selects are taken, so it would need a GPIO as chip
#     select, or SPI1, whose pins are only on the PiTFT's solder pads.
#   - The HX1 control plan has a Pico over USB doing the RTD front end and the
#     heater loop, which needs no GPIO here at all.
#
# Still free on the 2x13 header after the above: GPIO23 (pin 16), and
# GPIO2/3 (I2C, pins 3/5) and GPIO14/15 (UART, pins 8/10) if those buses are
# not needed. GPIO2/3 have fixed pull-up resistors on the Pi -- never use them
# for anything that must be low at boot.

_CLAIMED = {"VALVE_IN1": VALVE_IN1, "VALVE_IN2": VALVE_IN2}


def _check():
    used = {}
    for name, gpio in _CLAIMED.items():
        if gpio in PITFT_RESERVED:
            raise RuntimeError(f"pins.py: {name}=GPIO{gpio} is used by the PiTFT")
        if gpio in used:
            raise RuntimeError(f"pins.py: {name} and {used[gpio]} both use GPIO{gpio}")
        used[gpio] = name


_check()
