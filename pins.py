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
# Heater: 24 V 50 W cartridge through a logic-level MOSFET (heater.py)
# ---------------------------------------------------------------------------
HEATER_GATE = 22    # header pin 15. Pulled down at boot, so the heater stays
                    # off until the program takes over. Fit a gate pull-down
                    # resistor on the MOSFET board as well, so the heater is
                    # also off if this pin is ever left floating.

# ---------------------------------------------------------------------------
# Block temperature: DS18B20 on the kernel 1-Wire bus (heater.py)
# ---------------------------------------------------------------------------
ONEWIRE_DATA = 4    # header pin 7. Claimed by the kernel, not by Python:
                    # dtoverlay=w1-gpio in /boot/firmware/config.txt (GPIO4
                    # is the overlay's default; for another pin add
                    # ,gpiopin=N). Listed here so nothing else lands on it.
                    # 4.7 kOhm pull-up to 3.3 V (pin 1), sensor VDD on 3.3 V.

# ---------------------------------------------------------------------------
# Still free on the 2x13 header: GPIO23 (pin 16), and GPIO2/3 (I2C, pins
# 3/5) and GPIO14/15 (UART, pins 8/10) if those buses are not needed.
# GPIO2/3 have fixed pull-up resistors on the Pi -- never use them for
# anything that must be low at boot. A second DS18B20 (e.g. at the inlet)
# needs no pin of its own: it shares the 1-Wire bus on GPIO4.
# ---------------------------------------------------------------------------

_CLAIMED = {"VALVE_IN1": VALVE_IN1, "VALVE_IN2": VALVE_IN2,
            "HEATER_GATE": HEATER_GATE, "ONEWIRE_DATA": ONEWIRE_DATA}


def _check():
    used = {}
    for name, gpio in _CLAIMED.items():
        if gpio in PITFT_RESERVED:
            raise RuntimeError(f"pins.py: {name}=GPIO{gpio} is used by the PiTFT")
        if gpio in used:
            raise RuntimeError(f"pins.py: {name} and {used[gpio]} both use GPIO{gpio}")
        used[gpio] = name


_check()
