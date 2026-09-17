#!/usr/bin/env python3
"""
Touchscreen syringe-dispenser interface, two screens.

    SETUP      manual control for rigging up: keypad, PUSH, PULL, STOP.
               Same as before.

    OPERATION  routine use: keypad and a single INJECT button that runs the
               full cycle -- open target valve, push, close target, open
               refill, pull, close refill.

    The mode button sits in the top right corner and switches between them.

Pressing SETUP while an injection cycle is running ABORTS the cycle (ramped
stop, valves closed). That is deliberate: the Operation screen has no STOP
button by request, and until physical end stops are fitted there needs to be
some way to interrupt a cycle that was started with a wrong volume. To remove
it, delete the marked block in handle_key().

Run:
    python3 jog_ui.py

Imports pitft.py, actuator.py and valves.py from the same directory.

REQUIRES in actuator.py, until physical end stops are fitted:
    MIN_POSITION_MM = -1000000.0
    MAX_POSITION_MM =  1000000.0
"""

import time

from PIL import Image, ImageDraw, ImageFont

from pitft import PiTFT
from actuator import Actuator, IDLE, STOPPING, ERROR
import valves as valve_mod
from valves import Valves, TARGET, REFILL


# ===========================================================================
# CHANGE THIS WHEN YOU CHANGE SYRINGE
# ===========================================================================
SYRINGE_BORE_MM = 5.0        # internal diameter of the syringe barrel
# ===========================================================================

_BORE_AREA_MM2 = 3.141592653589793 * (SYRINGE_BORE_MM / 2.0) ** 2
MM_PER_UL = 1.0 / _BORE_AREA_MM2

# PUSH extends the actuator, away from the retracted home position.
PUSH_SIGN = +1

MAX_ENTRY_CHARS = 5
MICRO = "\u00b5"

MODE_SETUP = "setup"
MODE_OPERATION = "operation"

# Set False to remove the cycle-abort behaviour described above.
SETUP_BUTTON_ABORTS_CYCLE = True


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
INJECT_FILL = (25, 95, 95)
INJECT_EDGE = (70, 215, 215)
MODE_FILL   = (50, 40, 70)
MODE_EDGE   = (150, 130, 200)
DISABLED    = (45, 45, 55)
ACCENT      = (120, 120, 255)


def load_font(size, bold=False):
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


# ---------------------------------------------------------------------------
# The injection cycle
# ---------------------------------------------------------------------------

class InjectCycle:
    """Runs the six-step inject sequence without blocking.

    Like Actuator, nothing here waits: start() kicks it off and poll() advances
    it one step at a time from the main loop, so the display stays live and the
    touch panel stays responsive throughout.

    Steps:
        1. open valve to target container
        2. push the entered volume forward
        3. close target valve
        4. open valve to refill container
        5. pull the same volume back
        6. close refill valve
    """

    IDLE = "idle"
    OPEN_TARGET = "open target"
    PUSH = "pushing"
    CLOSE_TARGET = "close target"
    OPEN_REFILL = "open refill"
    PULL = "drawing up"
    CLOSE_REFILL = "close refill"
    DONE = "done"
    FAILED = "failed"

    def __init__(self, act, valves):
        self.act = act
        self.valves = valves
        self.state = self.IDLE
        self.message = ""
        self._deadline = 0.0
        self._volume_ul = 0

    @property
    def running(self):
        return self.state not in (self.IDLE, self.DONE, self.FAILED)

    def start(self, volume_ul):
        if self.running:
            return False, "cycle already running"
        if not self.act.homed:
            return False, "home first"
        if volume_ul <= 0:
            return False, "volume must be > 0"

        self._volume_ul = volume_ul
        self.valves.open(TARGET)
        self._deadline = time.monotonic() + valve_mod.VALVE_SETTLE_S
        self.state = self.OPEN_TARGET
        self.message = f"cycle: {volume_ul} {MICRO}l"
        return True, self.message

    def abort(self, reason="aborted"):
        if not self.running:
            return
        self.act.stop()
        self.valves.close_all()
        self.state = self.FAILED
        self.message = reason

    def poll(self):
        """Advance the cycle. Call every loop iteration. The caller is
        responsible for calling act.poll() as well."""
        if not self.running:
            return self.state

        now = time.monotonic()
        mm = self._volume_ul * MM_PER_UL

        # Any actuator fault stops the cycle where it stands, with the valves
        # closed. Leaving a valve open after a failed move would leave the
        # target line connected to a syringe in an unknown state.
        if self.act.state in (ERROR,) or self.act.state == "stalled":
            self.abort(f"actuator: {self.act.message}")
            return self.state

        if self.state == self.OPEN_TARGET:
            if now >= self._deadline:
                ok, msg = self.act.start_move_relative(PUSH_SIGN * mm)
                if not ok:
                    self.abort(f"push refused: {msg}")
                else:
                    self.state = self.PUSH

        elif self.state == self.PUSH:
            if not self.act.busy:
                self.valves.close(TARGET)
                self._deadline = now + valve_mod.VALVE_SETTLE_S
                self.state = self.CLOSE_TARGET

        elif self.state == self.CLOSE_TARGET:
            if now >= self._deadline:
                self.valves.open(REFILL)
                self._deadline = now + valve_mod.VALVE_SETTLE_S
                self.state = self.OPEN_REFILL

        elif self.state == self.OPEN_REFILL:
            if now >= self._deadline:
                ok, msg = self.act.start_move_relative(-PUSH_SIGN * mm)
                if not ok:
                    self.abort(f"pull refused: {msg}")
                else:
                    self.state = self.PULL

        elif self.state == self.PULL:
            if not self.act.busy:
                self.valves.close(REFILL)
                self._deadline = now + valve_mod.VALVE_SETTLE_S
                self.state = self.CLOSE_REFILL

        elif self.state == self.CLOSE_REFILL:
            if now >= self._deadline:
                self.state = self.DONE
                self.message = f"done: {self._volume_ul} {MICRO}l"

        return self.state


# ---------------------------------------------------------------------------
# UI
# ---------------------------------------------------------------------------

class JogUI:
    ENTRY_X, ENTRY_Y, ENTRY_W, ENTRY_H = 8, 6, 290, 40
    STATUS_X, STATUS_Y, STATUS_W, STATUS_H = 8, 50, 464, 30
    BODY_TOP = 84

    def __init__(self):
        self.tft = PiTFT()
        self.act = Actuator(microstep_divisor=16)
        self.valves = Valves()
        self.cycle = InjectCycle(self.act, self.valves)

        self.font_key = load_font(26, bold=True)
        self.font_big = load_font(30, bold=True)
        self.font_act = load_font(24, bold=True)
        self.font_mode = load_font(15, bold=True)
        self.font_mid = load_font(18)
        self.font_small = load_font(15)

        self.mode = MODE_SETUP
        self.entry = ""
        self.status = "starting"

        self._build_buttons()
        self._last_entry_drawn = None
        self._last_status_drawn = None
        self._last_homed = None
        self._touch_down = False

    # -- layout -------------------------------------------------------------

    def _build_buttons(self):
        # Shared keypad, present on both screens.
        keys = [["7", "8", "9"],
                ["4", "5", "6"],
                ["1", "2", "3"],
                ["00", "0", "<"]]
        x0, y0 = 6, 88
        cw, ch, gap = 80, 55, 4

        self.keypad = []
        for row, labels in enumerate(keys):
            for col, label in enumerate(labels):
                self.keypad.append(Button(
                    label, label,
                    x0 + col * (cw + gap), y0 + row * (ch + gap),
                    cw, ch, KEY_FILL, KEY_EDGE, font=self.font_key))

        # Mode switch, top right corner, present on both screens.
        self.mode_button = Button("MODE", "OPERATION", 386, 6, 88, 36,
                                  MODE_FILL, MODE_EDGE, font=self.font_mode)

        bx, bw = 266, 208
        # Setup screen actions.
        self.push_button = Button("PUSH", "PUSH", bx, 88, bw, 66,
                                  PUSH_FILL, PUSH_EDGE, font=self.font_act)
        self.pull_button = Button("PULL", "PULL", bx, 160, bw, 66,
                                  PULL_FILL, PULL_EDGE, font=self.font_act)
        self.stop_button = Button("STOP", "STOP", bx, 232, bw, 82,
                                  STOP_FILL, STOP_EDGE, font=self.font_big)

        # Operation screen action: one tall button.
        self.inject_button = Button("INJECT", "INJECT", bx, 88, bw, 226,
                                    INJECT_FILL, INJECT_EDGE, font=self.font_big)

    def active_buttons(self):
        common = self.keypad + [self.mode_button]
        if self.mode == MODE_SETUP:
            return common + [self.push_button, self.pull_button, self.stop_button]
        return common + [self.inject_button]

    # -- rendering ----------------------------------------------------------

    def draw_screen(self):
        """Full repaint. Only happens at startup and on a mode switch -- never
        inside the running loop, where 180ms of blindness would matter."""
        bg = Image.new("RGB", (self.tft.width, self.tft.height), BG)
        d = ImageDraw.Draw(bg)
        d.rectangle((0, 0, self.tft.width - 1, self.BODY_TOP - 2),
                    fill=PANEL, outline=ACCENT)
        self.tft.draw_full(bg)

        self.mode_button.label = ("OPERATION" if self.mode == MODE_SETUP
                                  else "SETUP")
        for b in self.active_buttons():
            self.tft.draw_region(b.render(), b.x, b.y)

        self._last_entry_drawn = None
        self._last_status_drawn = None
        self._last_homed = None
        self.draw_entry(force=True)
        self.draw_status(force=True)
        self.refresh_action_buttons(force=True)

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
        d.text((150, 12), converted, font=self.font_mid, fill=DIM)
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
        """PULL becomes HOME when the position is not trusted; PUSH and INJECT
        grey out, since driving forward from an unknown position is how you
        crash into the far end stop."""
        if self.act.homed == self._last_homed and not force:
            return
        self._last_homed = self.act.homed

        if self.mode == MODE_SETUP:
            if self.act.homed:
                self.pull_button.label = "PULL"
                self.pull_button.fill, self.pull_button.edge = PULL_FILL, PULL_EDGE
                self.push_button.fill, self.push_button.edge = PUSH_FILL, PUSH_EDGE
            else:
                self.pull_button.label = "HOME"
                self.pull_button.fill, self.pull_button.edge = HOME_FILL, HOME_EDGE
                self.push_button.fill, self.push_button.edge = DISABLED, KEY_EDGE
            targets = (self.push_button, self.pull_button)
        else:
            if self.act.homed:
                self.inject_button.label = "INJECT"
                self.inject_button.fill = INJECT_FILL
                self.inject_button.edge = INJECT_EDGE
            else:
                self.inject_button.label = "HOME"
                self.inject_button.fill, self.inject_button.edge = HOME_FILL, HOME_EDGE
            targets = (self.inject_button,)

        for b in targets:
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
        if key == "MODE":
            # ---- cycle abort on leaving Operation ----
            # Delete this block to make the mode button purely a screen switch.
            if SETUP_BUTTON_ABORTS_CYCLE and self.cycle.running:
                self.cycle.abort("aborted by mode switch")
                self.status = self.cycle.message
            # ------------------------------------------
            if self.cycle.running:
                return
            self.mode = (MODE_OPERATION if self.mode == MODE_SETUP
                         else MODE_SETUP)
            self.draw_screen()
            return

        if self.mode == MODE_SETUP:
            self._handle_setup_key(key)
        else:
            self._handle_operation_key(key)

    def _handle_setup_key(self, key):
        if key == "STOP":
            if self.act.busy:
                self.act.stop()
                self.status = "stopping"
            else:
                self.entry = ""
                self.status = "cleared"
            return

        if self.act.busy:
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

        self._handle_keypad(key)

    def _handle_operation_key(self, key):
        if self.cycle.running or self.act.busy:
            return

        if key == "INJECT":
            if not self.act.homed:
                self.act.start_home()
                self.status = "homing"
                return
            if not self.entry:
                self.status = "no volume set"
                return
            ok, msg = self.cycle.start(int(self.entry))
            self.status = msg if ok else f"refused: {msg}"
            return

        self._handle_keypad(key)

    def _handle_keypad(self, key):
        if key == "<":
            self.entry = self.entry[:-1]
        elif key == "00":
            if self.entry and self.entry != "0":
                self.entry = (self.entry + "00")[:MAX_ENTRY_CHARS]
        elif key in "0123456789" and len(self.entry) < MAX_ENTRY_CHARS:
            self.entry = key if self.entry in ("", "0") else self.entry + key

    def find_button(self, px, py):
        for b in self.active_buttons():
            if b.contains(px, py):
                return b
        return None

    # -- main loop ----------------------------------------------------------

    def run(self):
        self.draw_screen()

        if self.act.state == ERROR:
            self.status = self.act.message
            self.draw_status(force=True)
            print(self.act.message)
            return

        self.act.start_home()
        self.status = "homing"
        last_state = None
        last_cycle_state = None

        try:
            while True:
                # Both pollers, every iteration. act.poll() also feeds the
                # Tic's command-timeout watchdog, so it is never optional.
                state = self.act.poll()
                cycle_state = self.cycle.poll()

                if cycle_state != last_cycle_state:
                    last_cycle_state = cycle_state
                    if self.cycle.running:
                        self.status = cycle_state
                    elif cycle_state in (InjectCycle.DONE, InjectCycle.FAILED):
                        self.status = self.cycle.message
                elif state != last_state:
                    last_state = state
                    if not self.cycle.running:
                        self.status = self.act.message

                hit = self.tft.get_touch()
                if hit is not None and not self._touch_down:
                    self._touch_down = True
                    button = self.find_button(*hit)
                    if button is not None:
                        self.tft.draw_region(button.render(pressed=True),
                                             button.x, button.y)
                        self.handle_key(button.key)
                        self.act.poll()
                        # handle_key may have repainted the whole screen on a
                        # mode switch, in which case this button no longer
                        # exists on screen -- only unpress it if it is still
                        # part of the current screen.
                        if button in self.active_buttons():
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
            self.cycle.abort("interrupted")
            self.act.stop()
            while self.act.state == STOPPING:
                self.act.poll()
                time.sleep(0.02)
        finally:
            self.valves.close_all()
            self.act.close()
            self.tft.close()


def main():
    print(f"syringe bore {SYRINGE_BORE_MM}mm -> "
          f"{MM_PER_UL:.4f} mm per {MICRO}l "
          f"({1.0 / MM_PER_UL:.1f} {MICRO}l per mm)")
    print("valves are PLACEHOLDERS -- see valves.py")
    JogUI().run()


if __name__ == "__main__":
    main()