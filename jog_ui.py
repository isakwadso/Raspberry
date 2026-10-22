#!/usr/bin/env python3
"""
Touchscreen syringe-dispenser interface, two screens.

    SETUP      manual control for rigging up and priming: keypad, PUSH, PULL,
               STOP, and a valve toggle in the header, labelled with the
               position it will switch TO.

    OPERATION  routine use: keypad and a single INJECT button that runs the
               full cycle -- valve to target, push, valve to refill, pull,
               valve back to rest.

    The mode button sits in the top right corner and switches between them.

Valve rules (see valves.py for the valve itself):
    * The valve is only switched while the plunger is still. Switching while
      liquid moves would push it into, or pull it from, the wrong line.
    * Every homing run starts by switching the valve to REFILL. Homing drags
      the plunger back an unknown distance; with the valve on the target line
      that would suck liquid and air back in through the nozzle. The valve
      returns to rest once homing has finished.
    * Startup, abort and shutdown leave the valve at REST_POSITION.

Pressing SETUP while an injection cycle is running ABORTS the cycle (ramped
stop, then the valve to rest). That is deliberate: the Operation screen has no STOP
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
from actuator import Actuator, IDLE, STOPPING, ERROR, STALLED
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
    """Runs the inject sequence without blocking.

    Like Actuator, nothing here waits: start() kicks it off and poll() advances
    it one step at a time from the main loop, so the display stays live and the
    touch panel stays responsive throughout. (The one exception is the valve
    pulse itself, which blocks for valves.PULSE_S -- 0.3 s -- while the
    plunger is still.)

    Steps:
        1. valve to TARGET (syringe -> heater block and nozzle), settle
        2. push the entered volume forward
        3. valve to REFILL (syringe -> pouch), settle
        4. pull the same volume back
        5. valve back to rest, settle

    The valve is only ever switched while the plunger is still. That is also
    why abort() does not move the valve at once: it stops the actuator, waits
    in ABORTING until the plunger has actually stopped, and only then sends
    the valve to rest.

    Later additions (the HX1 post-dose pull-back ramp with the valve still at
    TARGET, purge doses) slot in as extra states between PUSH and TO_REFILL.
    """

    IDLE = "idle"
    TO_TARGET = "valve to target"
    PUSH = "pushing"
    TO_REFILL = "valve to refill"
    PULL = "drawing up"
    TO_REST = "valve to rest"
    ABORTING = "aborting"
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

    @property
    def aborting(self):
        return self.state == self.ABORTING

    def start(self, volume_ul):
        if self.running:
            return False, "cycle already running"
        if not self.act.homed:
            return False, "home first"
        if self.act.busy:
            return False, "actuator busy"
        if volume_ul <= 0:
            return False, "volume must be > 0"

        self._volume_ul = volume_ul
        self._switch_valve(TARGET, self.TO_TARGET)
        self.message = f"cycle: {volume_ul} {MICRO}l"
        return True, self.message

    def abort(self, reason="aborted"):
        """Stop the plunger now; the valve follows once it has stopped."""
        if not self.running or self.aborting:
            return
        self.act.stop()             # ramped; a no-op if the actuator is idle
        self.state = self.ABORTING
        self.message = reason

    def poll(self):
        """Advance the cycle. Call every loop iteration. The caller is
        responsible for calling act.poll() as well."""
        if not self.running:
            return self.state

        if self.aborting:
            if not self.act.busy:
                try:
                    self.act.poll()     # fresh Tic command timeout
                    self.valves.to_rest()
                finally:
                    self.state = self.FAILED
            return self.state

        # Any actuator fault stops the cycle where it stands. A fault means
        # the plunger is already halted, so the abort puts the valve to rest
        # on the next poll.
        if self.act.state in (ERROR, STALLED):
            self.abort(f"actuator: {self.act.message}")
            return self.state

        now = time.monotonic()
        mm = self._volume_ul * MM_PER_UL

        if self.state == self.TO_TARGET:
            if now >= self._deadline:
                self._start_stroke(PUSH_SIGN * mm, self.PUSH, "push")

        elif self.state == self.PUSH:
            if not self.act.busy:
                self._switch_valve(REFILL, self.TO_REFILL)

        elif self.state == self.TO_REFILL:
            if now >= self._deadline:
                self._start_stroke(-PUSH_SIGN * mm, self.PULL, "pull")

        elif self.state == self.PULL:
            if not self.act.busy:
                self._switch_valve(valve_mod.REST_POSITION, self.TO_REST)

        elif self.state == self.TO_REST:
            if now >= self._deadline:
                self.state = self.DONE
                self.message = f"done: {self._volume_ul} {MICRO}l"

        return self.state

    def _switch_valve(self, position, next_state):
        # The pulse blocks for valves.PULSE_S; poll first so the Tic's
        # command timeout starts fresh.
        self.act.poll()
        self.valves.set_position(position)
        # Settle time counts from the end of the pulse, not from the command.
        self._deadline = time.monotonic() + valve_mod.VALVE_SETTLE_S
        self.state = next_state

    def _start_stroke(self, delta_mm, next_state, name):
        ok, msg = self.act.start_move_relative(delta_mm)
        if ok:
            self.state = next_state
        else:
            self.abort(f"{name} refused: {msg}")


# ---------------------------------------------------------------------------
# UI
# ---------------------------------------------------------------------------

class JogUI:
    # The entry field is narrower than before to make room for the valve
    # toggle between it and the mode button.
    ENTRY_X, ENTRY_Y, ENTRY_W, ENTRY_H = 8, 6, 272, 40
    STATUS_X, STATUS_Y, STATUS_W, STATUS_H = 8, 50, 464, 30
    # Column offsets inside the status line: position, valve, message.
    STATUS_VALVE_DX, STATUS_MSG_DX = 168, 284
    BODY_TOP = 84

    def __init__(self):
        self.tft = PiTFT()
        # Valve first: its startup pulse (to rest) blocks for PULSE_S, and
        # doing it before the Tic is energised keeps that time out of the
        # Tic's command timeout.
        self.valves = Valves()
        self.act = Actuator()       # microstep divisor and speed: actuator.py
        self.cycle = InjectCycle(self.act, self.valves)

        self.font_key = load_font(26, bold=True)
        self.font_big = load_font(30, bold=True)
        self.font_act = load_font(24, bold=True)
        # Header buttons (mode, valve). 12 px is the largest size at which
        # "OPERATION" fits the 88 px mode button.
        self.font_mode = load_font(12, bold=True)
        self.font_mid = load_font(18)
        self.font_small = load_font(15)

        self.mode = MODE_SETUP
        self.entry = ""
        self.status = "starting"

        self._build_buttons()
        self._last_entry_drawn = None
        self._last_status_drawn = None
        self._last_homed = None
        self._last_valve_drawn = None
        self._touch_down = False
        # Set when a homing run has moved the valve to REFILL; the main loop
        # returns it to rest once the plunger has stopped.
        self._valve_back_to_rest = False

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

        # Valve toggle, Setup screen only, between the volume field and the
        # mode button. Label and colour are set by refresh_valve_button().
        self.valve_button = Button("VALVE", "VALVE", 284, 6, 96, 36,
                                   KEY_FILL, KEY_EDGE, font=self.font_mode)

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
            return common + [self.valve_button, self.push_button,
                             self.pull_button, self.stop_button]
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
        self._last_valve_drawn = None
        self.draw_entry(force=True)
        self.draw_status(force=True)
        self.refresh_action_buttons(force=True)
        self.refresh_valve_button(force=True)

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
            pos_text = f"{pos:7.2f} mm {pos / MM_PER_UL:6.0f} {MICRO}l"
        else:
            pos_text = "position unknown"
        valve = self.valves.position
        valve_text = f"valve {valve.upper()}" if valve else "valve ?"
        key = (self.status, pos_text, valve_text)
        if key == self._last_status_drawn and not force:
            return
        self._last_status_drawn = key

        img = Image.new("RGB", (self.STATUS_W, self.STATUS_H), PANEL)
        d = ImageDraw.Draw(img)
        d.text((4, 4), pos_text, font=self.font_small, fill=TEXT)
        d.text((self.STATUS_VALVE_DX, 4), valve_text,
               font=self.font_small, fill=TEXT)
        d.text((self.STATUS_MSG_DX, 4), self.status,
               font=self.font_small, fill=DIM)
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

    def refresh_valve_button(self, force=False):
        """Setup screen only. The label names the position a tap switches TO,
        coloured like the action that uses that line: teal (INJECT) for the
        target line, blue (PULL) for the refill line. Greyed out while the
        plunger moves or a cycle runs, because switching then would push
        liquid into the wrong line."""
        if self.mode != MODE_SETUP:
            return
        # Also greyed out while the coil is at its duty-cycle limit.
        enabled = not (self.act.busy or self.cycle.running
                       or not self.valves.duty_ok())
        key = (self.valves.position, enabled)
        if key == self._last_valve_drawn and not force:
            return
        self._last_valve_drawn = key

        b = self.valve_button
        next_pos = REFILL if self.valves.position == TARGET else TARGET
        b.label = f"\u2192 {next_pos.upper()}"
        if not enabled:
            b.fill, b.edge = DISABLED, KEY_EDGE
        elif next_pos == TARGET:
            b.fill, b.edge = INJECT_FILL, INJECT_EDGE
        else:
            b.fill, b.edge = PULL_FILL, PULL_EDGE
        self.tft.draw_region(b.render(), b.x, b.y)

    # -- input --------------------------------------------------------------

    def _start_home(self):
        """Home with the syringe connected to the pouch.

        Homing drags the plunger back until it stalls against the retracted
        stop -- an unknown distance. With the valve on the target line that
        would suck liquid and air back in through the nozzle, so the valve
        goes to REFILL first. The main loop puts it back to rest once the
        actuator has stopped, whether homing succeeded or was stopped."""
        self._valve_to(REFILL)
        self.act.start_home()
        self._valve_back_to_rest = True
        self.status = "homing"

    def _valve_to(self, position):
        """Every valve switch from the UI goes through here. The pulse blocks
        the loop for valves.PULSE_S (plus up to MIN_PULSE_INTERVAL_S), so the
        actuator is polled first: the Tic's command timeout (1 s by default)
        then starts fresh and the block stays well inside it."""
        self.act.poll()
        self.valves.set_position(position)

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
            # An aborting cycle finishes in the background (the main loop
            # keeps polling it), so the screen can switch right away.
            if self.cycle.running and not self.cycle.aborting:
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

        if self.act.busy or self.cycle.running:
            return

        if key == "VALVE":
            if not self.valves.duty_ok():
                self.status = "valve coil cooling, wait"
                return
            new = REFILL if self.valves.position == TARGET else TARGET
            self._valve_to(new)
            self.status = f"valve to {new}"
            return

        if key == "PUSH":
            if not self.act.homed:
                self.status = "home first"
                return
            self._start_move(PUSH_SIGN)
            return

        if key == "PULL":
            if not self.act.homed:
                self._start_home()
                return
            self._start_move(-PUSH_SIGN)
            return

        self._handle_keypad(key)

    def _handle_operation_key(self, key):
        if self.cycle.running or self.act.busy:
            return

        if key == "INJECT":
            if not self.act.homed:
                self._start_home()
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

        last_state = None
        last_cycle_state = None

        try:
            self._start_home()

            while True:
                # Both pollers, every iteration. act.poll() also feeds the
                # Tic's command-timeout watchdog, so it is never optional.
                state = self.act.poll()
                cycle_state = self.cycle.poll()

                # After a homing run, back to rest -- once the plunger has
                # stopped, never while it moves.
                if self._valve_back_to_rest and not self.act.busy:
                    self._valve_back_to_rest = False
                    self._valve_to(valve_mod.REST_POSITION)

                # Future pollers (temperature measurement, heater control) go
                # here. A valve switch blocks this loop for valves.PULSE_S
                # (0.3 s), so anything time-critical, such as a heater PWM,
                # should run in hardware or tolerate a 0.3-0.55 s gap.

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
                self.refresh_valve_button()
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
            # Order matters: the plunger stops (de-energised) before the valve
            # moves, the valve is left at rest, then everything is released.
            self.act.close()
            try:
                self.valves.to_rest()
            except Exception as exc:
                print(f"could not put the valve to rest: {exc}")
            self.valves.close()
            self.tft.close()


def main():
    print(f"syringe bore {SYRINGE_BORE_MM}mm -> "
          f"{MM_PER_UL:.4f} mm per {MICRO}l "
          f"({1.0 / MM_PER_UL:.1f} {MICRO}l per mm)")
    print(f"valve: {'SIMULATED' if valve_mod.SIMULATE else 'GPIO'}, "
          f"rest {valve_mod.REST_POSITION.upper()}, "
          f"IN1 -> {valve_mod.IN1_SETS.upper()}, "
          f"pulse {valve_mod.PULSE_S * 1000:.0f} ms "
          f"(pulse length and IN1 mapping still to be verified)")
    InjectCycle().run()


if __name__ == "__main__":
    main()