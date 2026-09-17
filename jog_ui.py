#!/usr/bin/env python3
"""
Touchscreen syringe-dispenser interface.

    Keypad 0-9 with a "00" key and backspace, a readout showing the entered
    volume in microlitres and its equivalent travel in mm, and three action
    buttons:

        PUSH   dispense the entered volume (extends the plunger)
        PULL   draw up the entered volume (retracts the plunger)
        STOP   while moving -> ramped stop, position stays known
               while idle   -> clears the entered value

    When the position is not trusted (at startup, or after a stall), PULL
    becomes HOME and PUSH is refused. Homing IS a retraction, so that mapping
    stays physically honest.

Run:
    python3 jog_ui.py

Imports pitft.py and actuator.py unchanged from the same directory.

BEFORE RUNNING, one edit in actuator.py -- the travel limits are removed until
physical end stops are fitted:

    MIN_POSITION_MM = -1000000.0
    MAX_POSITION_MM =  1000000.0

Nothing then prevents driving into either end stop. The stall watchdog in
poll() notices the overrun and invalidates the position, but only after the
actuator has been pushed against a hard stop. Keep an eye on it.
"""

import time

from PIL import Image, ImageDraw, ImageFont

from pitft import PiTFT
from actuator import Actuator, IDLE, MOVING, HOMING, STOPPING, ERROR, STALLED


# ===========================================================================
# CHANGE THIS WHEN YOU CHANGE SYRINGE
# ===========================================================================
SYRINGE_BORE_MM = 5.0        # internal diameter of the syringe barrel
# ===========================================================================

# Travel per microlitre. A 5mm bore gives 19.63mm^2 of cross-section, so 1ul
# (1mm^3) is about 0.051mm of travel and the full 100mm stroke is roughly
# 1960ul. Whole microlitres therefore give ample resolution without needing a
# decimal point -- which is why the keypad has "00" where the "." used to be.
_BORE_AREA_MM2 = 3.141592653589793 * (SYRINGE_BORE_MM / 2.0) ** 2
MM_PER_UL = 1.0 / _BORE_AREA_MM2

# PUSH extends the actuator, away from the retracted home position.
# Flip this to -1 if your mechanics are the other way round.
PUSH_SIGN = +1

MAX_ENTRY_CHARS = 5
MICRO = "\u00b5"             # the letter mu, for "ul"

DEBUG_DOT = True        # draw a white marker wherever a press registers

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
PUSH_FILL   = (20, 110, 45)
PUSH_EDGE   = (60, 210, 100)
PULL_FILL   = (25, 70, 125)
PULL_EDGE   = (80, 160, 255)
HOME_FILL   = (110, 80, 15)
HOME_EDGE   = (230, 180, 60)
STOP_FILL   = (140, 25, 25)
STOP_EDGE   = (255, 90, 90)
DISABLED    = (45, 45, 55)
ACCENT      = (120, 120, 255)


def load_font(size, bold=False):
    """PIL's default font is tiny and unreadable at arm's length, and lacks the
    mu glyph. Fall back to it only if DejaVu is missing."""
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    try:
        return ImageFont.truetype(f"/usr/share/fonts/truetype/dejavu/{name}", size)
    except OSError:
        return ImageFont.load_default()


class Button:
    def __init__(self, key, label, x, y, w, h, fill, edge, font=None):
        self.key = key
        self.label = label
        self.x, self.y, self.w, self.h = x, y, w, h
        self.fill, self.edge = fill, edge
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
               self.label, font=self.font, fill=TEXT)
        return img


class JogUI:
    # Readout regions, kept small on purpose: a partial redraw costs time in
    # proportion to its AREA, and the loop must stay responsive so STOP is
    # always reachable. A full-screen redraw is ~180ms, far too long to ignore
    # a safety button for.
    ENTRY_X, ENTRY_Y, ENTRY_W, ENTRY_H = 8, 6, 300, 40
    STATUS_X, STATUS_Y, STATUS_W, STATUS_H = 8, 50, 464, 30

    def __init__(self):
        self.tft = PiTFT()
        self.act = Actuator(microstep_divisor=16)

        self.font_key = load_font(26, bold=True)
        self.font_big = load_font(30, bold=True)
        self.font_act = load_font(24, bold=True)
        self.font_mid = load_font(18)
        self.font_small = load_font(15)

        self.entry = ""
        self.status = "starting"
        self.buttons = []
        self._build_buttons()

        self._last_entry_drawn = None
        self._last_status_drawn = None
        self._last_homed = None
        self._touch_down = False

    # -- layout -------------------------------------------------------------

    def _build_buttons(self):
        # Keypad: 3 columns x 4 rows on the left.
        keys = [["7", "8", "9"],
                ["4", "5", "6"],
                ["1", "2", "3"],
                ["00", "0", "<"]]
        x0, y0 = 6, 88
        cw, ch, gap = 80, 55, 4

        for row, labels in enumerate(keys):
            for col, label in enumerate(labels):
                self.buttons.append(Button(
                    label, label,
                    x0 + col * (cw + gap), y0 + row * (ch + gap),
                    cw, ch, KEY_FILL, KEY_EDGE, font=self.font_key))

        # Action column on the right. STOP is the largest and sits at the
        # bottom edge, where a thumb lands naturally and where it cannot be
        # confused with the two motion buttons above it.
        bx, bw = 266, 208
        self.push_button = Button("PUSH", "PUSH", bx, 88, bw, 66,
                                  PUSH_FILL, PUSH_EDGE, font=self.font_act)
        self.pull_button = Button("PULL", "PULL", bx, 160, bw, 66,
                                  PULL_FILL, PULL_EDGE, font=self.font_act)
        self.stop_button = Button("STOP", "STOP", bx, 232, bw, 82,
                                  STOP_FILL, STOP_EDGE, font=self.font_big)
        self.buttons += [self.push_button, self.pull_button, self.stop_button]

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
            return f"--- {MICRO}l", ""
        ul = int(self.entry)
        return f"{ul} {MICRO}l", f"= {ul * MM_PER_UL:.2f} mm"

    def draw_entry(self, force=False):
        value, converted = self._entry_text()
        key = (value, converted)
        if key == self._last_entry_drawn and not force:
            return
        self._last_entry_drawn = key

        img = Image.new("RGB", (self.ENTRY_W, self.ENTRY_H), PANEL)
        d = ImageDraw.Draw(img)
        d.text((4, 4), value, font=self.font_big, fill=TEXT)
        d.text((155, 12), converted, font=self.font_mid, fill=DIM)
        self.tft.draw_region(img, self.ENTRY_X, self.ENTRY_Y)

    def draw_status(self, force=False):
        pos = self.act.position_mm
        if self.act.homed:
            pos_text = f"pos {pos:7.2f} mm / {pos / MM_PER_UL:6.0f} {MICRO}l"
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

    def refresh_action_buttons(self, force=False):
        """PULL becomes HOME whenever the position is not trusted, so a stall
        never leaves the interface with no way forward. PUSH is greyed out in
        that state -- driving forward from an unknown position is exactly how
        you crash into the far end stop."""
        if self.act.homed == self._last_homed and not force:
            return
        self._last_homed = self.act.homed

        if self.act.homed:
            self.pull_button.label = "PULL"
            self.pull_button.fill, self.pull_button.edge = PULL_FILL, PULL_EDGE
            self.push_button.fill, self.push_button.edge = PUSH_FILL, PUSH_EDGE
        else:
            self.pull_button.label = "HOME"
            self.pull_button.fill, self.pull_button.edge = HOME_FILL, HOME_EDGE
            self.push_button.fill, self.push_button.edge = DISABLED, KEY_EDGE

        for b in (self.push_button, self.pull_button):
            self.tft.draw_region(b.render(), b.x, b.y)

    # -- input --------------------------------------------------------------

    def _start_move(self, sign):
        if not self.entry:
            self.status = "no volume set"
            return
        ul = int(self.entry)
        if ul <= 0:
            self.status = "volume must be > 0"
            return
        ok, msg = self.act.start_move_relative(sign * ul * MM_PER_UL)
        self.status = msg if ok else f"refused: {msg}"

    def handle_key(self, key):
        if key == "STOP":
            if self.act.busy:
                self.act.stop()
                self.status = "stopping"
            else:
                self.entry = ""
                self.status = "cleared"
            return

        if self.act.busy:
            # Everything except STOP is inert while the actuator is moving.
            return

        if key == "PUSH":
            if not self.act.homed:
                self.status = "home first"
                return
            self._start_move(PUSH_SIGN)
            return

        if key == "PULL":
            if not self.act.homed:
                self.act.start_home()
                self.status = "homing"
                return
            self._start_move(-PUSH_SIGN)
            return

        # keypad
        if key == "<":
            self.entry = self.entry[:-1]
        elif key == "00":
            if self.entry and self.entry != "0":
                self.entry = (self.entry + "00")[:MAX_ENTRY_CHARS]
        elif len(self.entry) < MAX_ENTRY_CHARS:
            self.entry = key if self.entry in ("", "0") else self.entry + key

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
        self.refresh_action_buttons(force=True)

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
                        # Poll again before spending time repainting, so a STOP
                        # is acted on at once rather than after the redraw.
                        self.act.poll()
                        self.tft.draw_region(button.render(),
                                             button.x, button.y)
                elif hit is None:
                    self._touch_down = False

                self.refresh_action_buttons()
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
    print(f"syringe bore {SYRINGE_BORE_MM}mm -> "
          f"{MM_PER_UL:.4f} mm travel per {MICRO}l "
          f"({1.0 / MM_PER_UL:.1f} {MICRO}l per mm)")
    JogUI().run()


if __name__ == "__main__":
    main()