#!/usr/bin/env python3
"""
Touchscreen syringe-dispenser interface.

    Keypad 0-9 with a decimal point and backspace, a readout showing the
    entered volume and its equivalent travel in mm, a MOVE button and a STOP
    button.

    STOP while moving   -> ramped stop, position stays known
    STOP while idle     -> clears the entered value

Run:
    python3 jog_ui.py

Imports pitft.py and actuator.py unchanged from the same directory.

BEFORE RUNNING, one edit in actuator.py -- the travel limits are being removed
until physical end stops are fitted:

    MIN_POSITION_MM = -1000000.0
    MAX_POSITION_MM =  1000000.0

Nothing then prevents driving into either end stop. The stall watchdog in
poll() will notice the overrun and invalidate the position, but the actuator
will have been pushed against a hard stop first. Keep an eye on it.
"""

import time

from PIL import Image, ImageDraw, ImageFont

from pitft import PiTFT
import actuator as act_mod
from actuator import Actuator, IDLE, MOVING, HOMING, STOPPING, ERROR, STALLED


# ===========================================================================
# CHANGE THIS WHEN YOU CHANGE SYRINGE
# ===========================================================================
SYRINGE_BORE_MM = 5.0        # internal diameter of the syringe barrel
# ===========================================================================

# Travel needed per millilitre. A 5mm bore gives 19.63mm^2 of cross-section,
# so 1ml (1000mm^3) is about 51mm of travel -- i.e. a 100mm actuator covers
# under 2ml. That is why the keypad has a decimal point.
_BORE_AREA_MM2 = 3.141592653589793 * (SYRINGE_BORE_MM / 2.0) ** 2
MM_PER_ML = 1000.0 / _BORE_AREA_MM2

# Dispensing extends the actuator, i.e. the positive direction away from the
# retracted home position. Flip this if your mechanics are the other way round.
DISPENSE_SIGN = +1

MAX_ENTRY_CHARS = 6


# ---------------------------------------------------------------------------
# Colours
# ---------------------------------------------------------------------------
BG          = (0, 0, 0)
PANEL       = (18, 18, 28)
KEY_FILL    = (38, 38, 52)
KEY_EDGE    = (90, 90, 120)
KEY_PRESSED = (110, 110, 150)
TEXT        = (255, 255, 255)
DIM         = (150, 150, 165)
MOVE_FILL   = (20, 110, 45)
MOVE_EDGE   = (60, 210, 100)
HOME_FILL   = (110, 80, 15)
HOME_EDGE   = (230, 180, 60)
STOP_FILL   = (140, 25, 25)
STOP_EDGE   = (255, 90, 90)
ACCENT      = (120, 120, 255)


def load_font(size, bold=False):
    """PIL's default font is tiny and unreadable at arm's length. Fall back to
    it only if DejaVu is missing."""
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    try:
        return ImageFont.truetype(f"/usr/share/fonts/truetype/dejavu/{name}", size)
    except OSError:
        return ImageFont.load_default()


class Button:
    def __init__(self, key, label, x, y, w, h, fill, edge, text_colour=TEXT,
                 font=None):
        self.key = key
        self.label = label
        self.x, self.y, self.w, self.h = x, y, w, h
        self.fill, self.edge = fill, edge
        self.text_colour = text_colour
        self.font = font

    def contains(self, px, py):
        return self.x <= px < self.x + self.w and self.y <= py < self.y + self.h

    def render(self, pressed=False):
        img = Image.new("RGB", (self.w, self.h),
                        KEY_PRESSED if pressed else self.fill)
        d = ImageDraw.Draw(img)
        d.rectangle((0, 0, self.w - 1, self.h - 1), outline=self.edge)
        bbox = d.textbbox((0, 0), self.label, font=self.font)
        d.text(((self.w - (bbox[2] - bbox[0])) / 2 - bbox[0],
                (self.h - (bbox[3] - bbox[1])) / 2 - bbox[1]),
               self.label, font=self.font, fill=self.text_colour)
        return img


class JogUI:
    # Readout regions, kept small on purpose: a partial redraw costs time in
    # proportion to its AREA, and the loop must stay responsive so STOP is
    # always reachable. A full-screen redraw is ~180ms, which is far too long
    # to ignore a safety button for.
    ENTRY_X, ENTRY_Y, ENTRY_W, ENTRY_H = 8, 6, 300, 40
    STATUS_X, STATUS_Y, STATUS_W, STATUS_H = 8, 50, 464, 30

    def __init__(self):
        self.tft = PiTFT()
        self.act = Actuator(microstep_divisor=16)

        self.font_key = load_font(26, bold=True)
        self.font_big = load_font(30, bold=True)
        self.font_mid = load_font(18)
        self.font_small = load_font(15)

        self.entry = ""
        self.status = "starting"
        self.buttons = []
        self._build_buttons()

        self._last_entry_drawn = None
        self._last_status_drawn = None
        self._touch_down = False

    # -- layout -------------------------------------------------------------

    def _build_buttons(self):
        # Keypad: 3 columns x 4 rows on the left.
        keys = [["7", "8", "9"],
                ["4", "5", "6"],
                ["1", "2", "3"],
                [".", "0", "<"]]
        x0, y0 = 6, 88
        cw, ch, gap = 80, 55, 4

        for row, labels in enumerate(keys):
            for col, label in enumerate(labels):
                self.buttons.append(Button(
                    label, label,
                    x0 + col * (cw + gap), y0 + row * (ch + gap),
                    cw, ch, KEY_FILL, KEY_EDGE, font=self.font_key))

        # MOVE and STOP on the right. STOP is the taller of the two and sits
        # at the bottom edge where a thumb naturally lands.
        self.move_button = Button("MOVE", "MOVE", 266, 88, 208, 100,
                                  MOVE_FILL, MOVE_EDGE, font=self.font_big)
        self.stop_button = Button("STOP", "STOP", 266, 196, 208, 118,
                                  STOP_FILL, STOP_EDGE, font=self.font_big)
        self.buttons.append(self.move_button)
        self.buttons.append(self.stop_button)

    # -- rendering ----------------------------------------------------------

    def draw_static(self):
        """Everything that never changes, pushed once as a single full frame."""
        bg = Image.new("RGB", (self.tft.width, self.tft.height), BG)
        d = ImageDraw.Draw(bg)
        d.rectangle((0, 0, self.tft.width - 1, 82), fill=PANEL, outline=ACCENT)
        self.tft.draw_full(bg)

        for b in self.buttons:
            self.tft.draw_region(b.render(), b.x, b.y)

    def _entry_text(self):
        if not self.entry:
            return "-- ml", ""
        try:
            ml = float(self.entry)
        except ValueError:
            return self.entry + " ml", "invalid"
        return f"{self.entry} ml", f"= {ml * MM_PER_ML:.1f} mm"

    def draw_entry(self, force=False):
        value, converted = self._entry_text()
        key = (value, converted)
        if key == self._last_entry_drawn and not force:
            return
        self._last_entry_drawn = key

        img = Image.new("RGB", (self.ENTRY_W, self.ENTRY_H), PANEL)
        d = ImageDraw.Draw(img)
        d.text((4, 4), value, font=self.font_big, fill=TEXT)
        d.text((150, 12), converted, font=self.font_mid, fill=DIM)
        self.tft.draw_region(img, self.ENTRY_X, self.ENTRY_Y)

    def draw_status(self, force=False):
        pos = self.act.position_mm
        if self.act.homed:
            pos_text = f"pos {pos:7.2f} mm / {pos / MM_PER_ML:5.3f} ml"
        else:
            pos_text = "position unknown"
        key = (self.status, pos_text)
        if key == self._last_status_drawn and not force:
            return
        self._last_status_drawn = key

        img = Image.new("RGB", (self.STATUS_W, self.STATUS_H), PANEL)
        d = ImageDraw.Draw(img)
        d.text((4, 4), pos_text, font=self.font_small, fill=TEXT)
        d.text((250, 4), self.status, font=self.font_small, fill=DIM)
        self.tft.draw_region(img, self.STATUS_X, self.STATUS_Y)

    def refresh_move_button(self):
        """MOVE becomes HOME whenever the position is not trusted, so a stall
        never leaves the interface with no way forward."""
        if self.act.homed:
            self.move_button.label = "MOVE"
            self.move_button.fill, self.move_button.edge = MOVE_FILL, MOVE_EDGE
        else:
            self.move_button.label = "HOME"
            self.move_button.fill, self.move_button.edge = HOME_FILL, HOME_EDGE
        self.tft.draw_region(self.move_button.render(),
                             self.move_button.x, self.move_button.y)

    # -- input --------------------------------------------------------------

    def handle_key(self, key):
        if key == "STOP":
            if self.act.busy:
                self.act.stop()
                self.status = "stopping"
            else:
                self.entry = ""
                self.status = "cleared"
            return

        if key == "MOVE":
            if self.act.busy:
                return
            if not self.act.homed:
                self.act.start_home()
                self.status = "homing"
                return
            if not self.entry:
                self.status = "no volume set"
                return
            try:
                ml = float(self.entry)
            except ValueError:
                self.status = "invalid entry"
                return
            if ml <= 0:
                self.status = "volume must be > 0"
                return
            delta_mm = DISPENSE_SIGN * ml * MM_PER_ML
            ok, msg = self.act.start_move_relative(delta_mm)
            self.status = msg if ok else f"refused: {msg}"
            return

        # keypad
        if self.act.busy:
            return
        if key == "<":
            self.entry = self.entry[:-1]
        elif key == ".":
            if "." not in self.entry and len(self.entry) < MAX_ENTRY_CHARS:
                self.entry = (self.entry or "0") + "."
        elif len(self.entry) < MAX_ENTRY_CHARS:
            if self.entry == "0":
                self.entry = key
            else:
                self.entry += key

    def find_button(self, px, py):
        for b in self.buttons:
            if b.contains(px, py):
                return b
        return None

    # -- main loop ----------------------------------------------------------

    def run(self):
        self.draw_static()
        self.draw_entry(force=True)
        self.draw_status(force=True)

        if self.act.state == ERROR:
            self.status = self.act.message
            self.draw_status(force=True)
            print(self.act.message)
            return

        self.act.start_home()
        self.status = "homing"
        last_state = None

        try:
            while True:
                # Poll the actuator EVERY iteration, in every state -- it feeds
                # the Tic's command-timeout watchdog as well as advancing moves.
                state = self.act.poll()

                if state != last_state:
                    last_state = state
                    self.status = self.act.message
                    self.refresh_move_button()

                hit = self.tft.get_touch()
                if hit is not None and not self._touch_down:
                    # Act on the press edge only: the panel reports continuously
                    # while held, which would otherwise repeat the key.
                    self._touch_down = True
                    button = self.find_button(*hit)
                    if button is not None:
                        self.tft.draw_region(button.render(pressed=True),
                                             button.x, button.y)
                        self.handle_key(button.key)
                        # Repaint unpressed. Poll once more first so a STOP is
                        # acted on before spending time drawing.
                        self.act.poll()
                        self.tft.draw_region(button.render(),
                                             button.x, button.y)
                elif hit is None:
                    self._touch_down = False

                self.draw_entry()
                self.draw_status()

                time.sleep(0.02)

        except KeyboardInterrupt:
            print("\nstopping")
            self.act.stop()
            while self.act.state == STOPPING:
                self.act.poll()
                time.sleep(0.02)
        finally:
            self.act.close()
            self.tft.close()


def main():
    print(f"syringe bore {SYRINGE_BORE_MM}mm -> {MM_PER_ML:.2f} mm travel per ml")
    JogUI().run()


if __name__ == "__main__":
    main()