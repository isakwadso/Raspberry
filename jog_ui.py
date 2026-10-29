#!/usr/bin/env python3
"""
Touchscreen syringe-dispenser interface, three screens.

    SETUP        manual control for rigging up and priming: keypad, PUSH, PULL,
                 STOP, and a valve toggle in the header, labelled with the
                 position it will switch TO.

    OPERATION    routine use: keypad and a single INJECT button that runs the
                 full cycle -- valve to target, push, valve to refill, pull,
                 valve back to rest.

    TEMPERATURE  opened by tapping the temperature tile: keypad for the
                 setpoint in whole degrees, SET (applies, saves and switches
                 heating on), a heater on/off toggle labelled with what a tap
                 will do, BACK, and a 30-minute trend.

    The mode button sits in the top right corner and switches between SETUP
    and OPERATION (on the temperature screen it reads BACK).

The temperature tile sits at the right end of the status row on every screen:
green READY, amber HEATING / SETTLING / COOLING, grey heater off, red fault.
It can only be opened while the plunger is still and no cycle runs. The
heater itself runs in heater.py's own threads and keeps regulating whatever
screen is showing.

Heater rules:
    * INJECT is refused while the heater reports a fault (the INJECT button
      greys out). Set REQUIRE_READY_TO_INJECT = True to also refuse it until
      the tile is green. A heater that is switched OFF is not a fault: dosing
      at room temperature stays possible.
    * Every stroke that pushes liquid through the target line -- an INJECT
      cycle, a manual PUSH with the valve on TARGET, or either of them stopped
      partway -- is reported to the heater with the volume actually moved, for
      the feed-forward (see heater.py).
    * The setpoint and on/off state are saved in settings.json and restored at
      the next start, so the block starts heating as soon as the program runs.
      Heating stops when the program stops: for overnight equilibration leave
      it running (in tmux, or as a systemd service).

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

Imports pitft.py, actuator.py, valves.py and heater.py from the same directory.

REQUIRES in actuator.py, until physical end stops are fitted:
    MIN_POSITION_MM = -1000000.0
    MAX_POSITION_MM =  1000000.0
"""

import signal
import time

from PIL import Image, ImageDraw, ImageFont

from pitft import PiTFT
from actuator import Actuator, IDLE, STOPPING, ERROR, STALLED, HOMING
import valves as valve_mod
from valves import Valves, TARGET, REFILL
import heater as heater_mod
from heater import Heater


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
MAX_TEMP_CHARS = 2
MICRO = "\u00b5"
DEG = "\u00b0"

MODE_SETUP = "setup"
MODE_OPERATION = "operation"
MODE_TEMP = "temperature"

# Set False to remove the cycle-abort behaviour described above.
SETUP_BUTTON_ABORTS_CYCLE = True

# False: INJECT is refused only on a heater fault. True: also until READY.
REQUIRE_READY_TO_INJECT = False

# Strokes through the target line smaller than this are not reported to the
# heater (rounding noise, a nudge while priming).
MIN_DOSE_FOR_FF_UL = 5.0


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
SET_FILL    = (40, 40, 95)
SET_EDGE    = (120, 120, 255)
DISABLED    = (45, 45, 55)
ACCENT      = (120, 120, 255)
BAND_FILL   = (22, 55, 32)

# Temperature tile: heater state -> (fill, edge, word on the second line).
TILE_STYLES = {
    heater_mod.READY:    (PUSH_FILL, PUSH_EDGE, "READY"),
    heater_mod.SETTLING: (HOME_FILL, HOME_EDGE, "SETTLING"),
    heater_mod.HEATING:  (HOME_FILL, HOME_EDGE, "HEATING"),
    heater_mod.COOLING:  (HOME_FILL, HOME_EDGE, "COOLING"),
    heater_mod.OFF:      (DISABLED,  KEY_EDGE,  "HEATER OFF"),
    heater_mod.FAULT:    (STOP_FILL, STOP_EDGE, "FAULT"),
}


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


class TempTile(Button):
    """The temperature tile: a button with two centred lines, the reading on
    top and the state below. Its text and colours are set by
    JogUI.refresh_tile()."""

    def __init__(self, x, y, w, h, font, font_sub):
        super().__init__("TEMP", "", x, y, w, h, DISABLED, KEY_EDGE, font)
        self.font_sub = font_sub
        self.sub = ""

    def render(self, pressed=False):
        img = Image.new("RGB", (self.w, self.h),
                        KEY_PRESSED if pressed else self.fill)
        d = ImageDraw.Draw(img)
        d.rectangle((0, 0, self.w - 1, self.h - 1), outline=self.edge)
        bbox = d.textbbox((0, 0), self.label, font=self.font)
        d.text(((self.w - (bbox[2] - bbox[0])) / 2 - bbox[0], 4 - bbox[1]),
               self.label, font=self.font, fill=TEXT)
        bbox = d.textbbox((0, 0), self.sub, font=self.font_sub)
        d.text(((self.w - (bbox[2] - bbox[0])) / 2 - bbox[0],
                self.h - 4 - (bbox[3] - bbox[1]) - bbox[1]),
               self.sub, font=self.font_sub, fill=TEXT)
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

    The heater's feed-forward is not triggered from here: JogUI reports every
    stroke through the target line, from whichever source, once it has ended.

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
    # Header: volume (or setpoint) field, valve or heater toggle, mode button.
    ENTRY_X, ENTRY_Y, ENTRY_W, ENTRY_H = 8, 6, 272, 40
    # Status row, two lines: position and valve on top, message below. It
    # leaves the right end of the row to the temperature tile.
    STATUS_X, STATUS_Y, STATUS_W, STATUS_H = 8, 50, 340, 42
    STATUS_VALVE_DX = 168
    STATUS_LINE2_DY = 23
    TILE_X, TILE_Y, TILE_W, TILE_H = 352, 50, 122, 42
    BODY_TOP = 96
    # Temperature screen: trend plot under the SET button.
    TREND_X, TREND_Y, TREND_W, TREND_H = 266, 160, 208, 152
    TREND_SPAN_K = 1.0          # plot shows setpoint +- this

    def __init__(self):
        self.tft = PiTFT()
        # Valve first: its startup pulse (to rest) blocks for PULSE_S, and
        # doing it before the Tic is energised keeps that time out of the
        # Tic's command timeout.
        self.valves = Valves()
        # Heater next: opens its gate pin low and loads settings.json. Its
        # threads only start in run().
        self.heater = Heater()
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
        self.font_tile = load_font(21, bold=True)
        self.font_axis = load_font(11)

        self.mode = MODE_SETUP
        self._return_mode = MODE_SETUP      # where BACK goes from TEMPERATURE
        self.entry = ""
        self.temp_entry = ""
        self.status = "starting"

        self._build_buttons()
        self._last_entry_drawn = None
        self._last_status_drawn = None
        self._last_action_drawn = None
        self._last_valve_drawn = None
        self._last_tile_drawn = None
        self._last_heat_drawn = None
        self._last_trend_drawn = None
        self._touch_down = False
        # Set when a homing run has moved the valve to REFILL; the main loop
        # returns it to rest once the plunger has stopped.
        self._valve_back_to_rest = False
        # Stroke bookkeeping for the heater's feed-forward: see _track_dose().
        self._idle_pos_mm = None
        self._stroke = None

    # -- layout -------------------------------------------------------------

    def _build_buttons(self):
        # Shared keypad, present on every screen.
        keys = [["7", "8", "9"],
                ["4", "5", "6"],
                ["1", "2", "3"],
                ["00", "0", "<"]]
        x0, y0 = 6, 98
        cw, ch, gap = 80, 52, 4

        self.keypad = []
        for row, labels in enumerate(keys):
            for col, label in enumerate(labels):
                self.keypad.append(Button(
                    label, label,
                    x0 + col * (cw + gap), y0 + row * (ch + gap),
                    cw, ch, KEY_FILL, KEY_EDGE, font=self.font_key))
        self.key_00 = next(b for b in self.keypad if b.key == "00")

        # Mode switch, top right corner, present on every screen.
        self.mode_button = Button("MODE", "OPERATION", 386, 6, 88, 36,
                                  MODE_FILL, MODE_EDGE, font=self.font_mode)

        # Valve toggle, Setup screen only, between the volume field and the
        # mode button. Label and colour are set by refresh_valve_button().
        self.valve_button = Button("VALVE", "VALVE", 284, 6, 96, 36,
                                   KEY_FILL, KEY_EDGE, font=self.font_mode)

        # Heater toggle, temperature screen only, in the same place.
        self.heat_button = Button("HEAT", "HEAT", 284, 6, 96, 36,
                                  KEY_FILL, KEY_EDGE, font=self.font_mode)

        # Temperature tile, every screen; a button on Setup and Operation.
        self.tile = TempTile(self.TILE_X, self.TILE_Y, self.TILE_W,
                             self.TILE_H, self.font_tile, self.font_mode)

        bx, bw = 266, 208
        # Setup screen actions.
        self.push_button = Button("PUSH", "PUSH", bx, 98, bw, 60,
                                  PUSH_FILL, PUSH_EDGE, font=self.font_act)
        self.pull_button = Button("PULL", "PULL", bx, 164, bw, 60,
                                  PULL_FILL, PULL_EDGE, font=self.font_act)
        self.stop_button = Button("STOP", "STOP", bx, 230, bw, 82,
                                  STOP_FILL, STOP_EDGE, font=self.font_big)

        # Operation screen action: one tall button.
        self.inject_button = Button("INJECT", "INJECT", bx, 98, bw, 214,
                                    INJECT_FILL, INJECT_EDGE, font=self.font_big)

        # Temperature screen action.
        self.set_button = Button("SET", "SET", bx, 98, bw, 56,
                                 SET_FILL, SET_EDGE, font=self.font_act)

    def active_buttons(self):
        if self.mode == MODE_TEMP:
            return self.keypad + [self.mode_button, self.heat_button,
                                  self.set_button]
        common = self.keypad + [self.mode_button, self.tile]
        if self.mode == MODE_SETUP:
            return common + [self.valve_button, self.push_button,
                             self.pull_button, self.stop_button]
        return common + [self.inject_button]

    # -- rendering ----------------------------------------------------------

    def draw_screen(self):
        """Full repaint. Only happens at startup and on a screen switch --
        never inside the running loop, where 180ms of blindness would
        matter."""
        bg = Image.new("RGB", (self.tft.width, self.tft.height), BG)
        d = ImageDraw.Draw(bg)
        d.rectangle((0, 0, self.tft.width - 1, self.BODY_TOP - 2),
                    fill=PANEL, outline=ACCENT)
        self.tft.draw_full(bg)

        if self.mode == MODE_TEMP:
            self.mode_button.label = "BACK"
            self.key_00.label = ""          # setpoints are two digits
        else:
            self.mode_button.label = ("OPERATION" if self.mode == MODE_SETUP
                                      else "SETUP")
            self.key_00.label = "00"
        for b in self.active_buttons():
            if b is not self.tile:
                self.tft.draw_region(b.render(), b.x, b.y)

        self._last_entry_drawn = None
        self._last_status_drawn = None
        self._last_action_drawn = None
        self._last_valve_drawn = None
        self._last_tile_drawn = None
        self._last_heat_drawn = None
        self._last_trend_drawn = None
        snap = self.heater.snapshot()
        self.draw_entry(force=True)
        self.draw_status(self.act.position_mm, snap, force=True)
        self.refresh_tile(snap, force=True)
        self.refresh_action_buttons(snap, force=True)
        self.refresh_valve_button(force=True)
        self.refresh_heat_button(snap, force=True)
        self.draw_trend(snap, force=True)

    def _entry_text(self):
        if self.mode == MODE_TEMP:
            value = (f"{int(self.temp_entry)} {DEG}C" if self.temp_entry
                     else f"-- {DEG}C")
            return value, (f"range {heater_mod.SETPOINT_MIN_C}\u2013"
                           f"{heater_mod.SETPOINT_MAX_C}")
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

    def _status_columns(self, pos, snap):
        """Top line of the status row as [(x offset, text), ...]."""
        if self.mode == MODE_TEMP:
            cols = [(0, f"set {snap.setpoint_c} {DEG}C")]
            if snap.fault:
                cols.append((100, f"FAULT: {snap.fault}"))
            elif not snap.enabled:
                cols.append((100, "heater off"))
            else:
                cols.append((100, f"heater {snap.power_frac * 100:3.0f} %"))
                if snap.in_band_s > 0:
                    m, s = divmod(int(snap.in_band_s), 60)
                    cols.append((212, f"in band {m}:{s:02d}"))
                elif snap.temp_c is not None:
                    cols.append((212, f"\u0394 {snap.temp_c - snap.setpoint_c:+.2f} K"))
            return cols

        if self.act.homed:
            pos_text = f"{pos:7.2f} mm {pos / MM_PER_UL:6.0f} {MICRO}l"
        else:
            pos_text = "position unknown"
        valve = self.valves.position
        valve_text = f"valve {valve.upper()}" if valve else "valve ?"
        return [(0, pos_text), (self.STATUS_VALVE_DX, valve_text)]

    def draw_status(self, pos, snap, force=False):
        cols = self._status_columns(pos, snap)
        key = (self.status, tuple(cols))
        if key == self._last_status_drawn and not force:
            return
        self._last_status_drawn = key

        img = Image.new("RGB", (self.STATUS_W, self.STATUS_H), PANEL)
        d = ImageDraw.Draw(img)
        for dx, text in cols:
            d.text((4 + dx, 3), text, font=self.font_small, fill=TEXT)
        d.text((4, self.STATUS_LINE2_DY), self.status,
               font=self.font_small, fill=DIM)
        self.tft.draw_region(img, self.STATUS_X, self.STATUS_Y)

    def refresh_tile(self, snap, force=False):
        """One decimal on purpose: the DS18B20 reads in 0.0625 K steps, so a
        second decimal would only show the step pattern."""
        fill, edge, word = TILE_STYLES.get(snap.state, TILE_STYLES[heater_mod.OFF])
        text = (f"--.- {DEG}C" if snap.temp_c is None
                else f"{snap.temp_c:.1f} {DEG}C")
        if snap.state == heater_mod.FAULT:
            sub = (snap.fault or "fault").upper()
        elif snap.state == heater_mod.OFF:
            sub = word
        else:
            sub = f"{word} \u2192 {snap.setpoint_c}"
        key = (text, sub, fill)
        if key == self._last_tile_drawn and not force:
            return
        self._last_tile_drawn = key
        t = self.tile
        t.label, t.sub, t.fill, t.edge = text, sub, fill, edge
        self.tft.draw_region(t.render(), t.x, t.y)

    def _inject_refusal(self, snap):
        """Why INJECT would be refused because of the heater, or None."""
        if snap.fault:
            return f"heater {snap.fault}"
        if (REQUIRE_READY_TO_INJECT and snap.enabled
                and snap.state != heater_mod.READY):
            return "temperature not ready"
        return None

    def refresh_action_buttons(self, snap, force=False):
        """PULL becomes HOME when the position is not trusted; PUSH and INJECT
        grey out, since driving forward from an unknown position is how you
        crash into the far end stop. INJECT also greys out while the heater
        would refuse it."""
        if self.mode == MODE_TEMP:
            return
        blocked = self._inject_refusal(snap) is not None
        key = (self.mode, self.act.homed, blocked)
        if key == self._last_action_drawn and not force:
            return
        self._last_action_drawn = key

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
            if not self.act.homed:
                self.inject_button.label = "HOME"
                self.inject_button.fill, self.inject_button.edge = HOME_FILL, HOME_EDGE
            elif blocked:
                self.inject_button.label = "INJECT"
                self.inject_button.fill, self.inject_button.edge = DISABLED, KEY_EDGE
            else:
                self.inject_button.label = "INJECT"
                self.inject_button.fill = INJECT_FILL
                self.inject_button.edge = INJECT_EDGE
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

    def refresh_heat_button(self, snap, force=False):
        """Temperature screen only. Like the valve toggle, the label names
        what a tap does: red HEAT -> OFF while heating, amber HEAT -> ON while
        off."""
        if self.mode != MODE_TEMP:
            return
        if snap.enabled == self._last_heat_drawn and not force:
            return
        self._last_heat_drawn = snap.enabled
        b = self.heat_button
        if snap.enabled:
            b.label = "HEAT \u2192 OFF"
            b.fill, b.edge = STOP_FILL, STOP_EDGE
        else:
            b.label = "HEAT \u2192 ON"
            b.fill, b.edge = HOME_FILL, HOME_EDGE
        self.tft.draw_region(b.render(), b.x, b.y)

    def draw_trend(self, snap, force=False):
        """Temperature screen only: the last 30 min against setpoint +-
        TREND_SPAN_K, with the READY band shaded. Values outside the window
        are drawn on its edge. Redrawn when a new point arrives (every
        heater.HISTORY_STEP_S), ~35 ms each time."""
        if self.mode != MODE_TEMP:
            return
        key = (snap.history_version, snap.setpoint_c)
        if key == self._last_trend_drawn and not force:
            return
        self._last_trend_drawn = key

        w, h = self.TREND_W, self.TREND_H
        img = Image.new("RGB", (w, h), PANEL)
        d = ImageDraw.Draw(img)
        d.rectangle((0, 0, w - 1, h - 1), outline=KEY_EDGE)
        sp = snap.setpoint_c
        lo, hi = sp - self.TREND_SPAN_K, sp + self.TREND_SPAN_K
        top, bottom = 4, h - 18

        def y_of(v):
            v = min(max(v, lo), hi)
            return int(round(top + (hi - v) / (hi - lo) * (bottom - top)))

        band = heater_mod.STABLE_BAND_K
        d.rectangle((1, y_of(sp + band), w - 2, y_of(sp - band)), fill=BAND_FILL)
        for x in range(2, w - 2, 8):
            d.line((x, y_of(sp), x + 4, y_of(sp)), fill=PUSH_EDGE)

        history = self.heater.history()
        if history:
            t_end = time.monotonic()
            span = heater_mod.HISTORY_S
            pts = [(2 + (t - (t_end - span)) / span * (w - 4), y_of(v))
                   for t, v in history if t >= t_end - span]
            if len(pts) >= 2:
                d.line(pts, fill=TEXT, width=1)
            elif pts:
                d.point(pts, fill=TEXT)

        f = self.font_axis
        d.text((4, h - 15), f"-{heater_mod.HISTORY_S / 60:.0f} min", font=f, fill=DIM)
        bb = d.textbbox((0, 0), "now", font=f)
        d.text((w - 6 - (bb[2] - bb[0]), h - 15), "now", font=f, fill=DIM)
        d.text((w - 48, y_of(hi) - 1), f"{hi:.1f}{DEG}", font=f, fill=DIM)
        d.text((w - 48, y_of(lo) - 12), f"{lo:.1f}{DEG}", font=f, fill=DIM)
        self.tft.draw_region(img, self.TREND_X, self.TREND_Y)

    # -- feed-forward bookkeeping ---------------------------------------------

    def _track_dose(self, pos):
        """Report every stroke that pushed liquid through the target line to
        the heater, with the volume it actually moved.

        Watches the actuator rather than the cycle, so an INJECT cycle, a
        manual PUSH with the valve on TARGET, and either of them stopped
        partway are all covered. Pulls, homing (valve on REFILL, and the
        position is re-zeroed) and pushes back into the pouch are not. The
        valve cannot change during a stroke, so its position on the first
        busy iteration is the stroke's line; the start position is the last
        reading while the plunger was still."""
        if self.act.busy:
            if self._stroke is None:
                self._stroke = {"start_mm": self._idle_pos_mm,
                                "valve": self.valves.position,
                                "homing": False}
            if self.act.state == HOMING:
                self._stroke["homing"] = True
            return

        s, self._stroke = self._stroke, None
        self._idle_pos_mm = pos
        if (s is None or s["homing"] or s["valve"] != TARGET
                or s["start_mm"] is None or not self.act.homed):
            return
        moved_ul = PUSH_SIGN * (pos - s["start_mm"]) / MM_PER_UL
        if moved_ul >= MIN_DOSE_FOR_FF_UL:
            self.heater.add_dose(moved_ul)

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
        if key == "TEMP":
            # Only while everything is still: the temperature screen has no
            # STOP, and leaving it must not be the only way to see a move.
            if self.act.busy or self.cycle.running:
                return
            self._return_mode = self.mode
            self.mode = MODE_TEMP
            self.temp_entry = ""
            self.draw_screen()
            return

        if key == "MODE":
            if self.mode == MODE_TEMP:          # labelled BACK
                self.mode = self._return_mode
                self.draw_screen()
                return
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
        elif self.mode == MODE_OPERATION:
            self._handle_operation_key(key)
        else:
            self._handle_temp_key(key)

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

        self.entry = self._edit(self.entry, key, MAX_ENTRY_CHARS)

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
            refusal = self._inject_refusal(self.heater.snapshot())
            if refusal:
                self.status = f"refused: {refusal}"
                return
            ok, msg = self.cycle.start(int(self.entry))
            self.status = msg if ok else f"refused: {msg}"
            return

        self.entry = self._edit(self.entry, key, MAX_ENTRY_CHARS)

    def _handle_temp_key(self, key):
        if key == "SET":
            if not self.temp_entry:
                self.status = "no temperature entered"
                return
            ok, msg = self.heater.set_setpoint(int(self.temp_entry))
            self.status = msg if ok else f"refused: {msg}"
            if ok:
                self.temp_entry = ""
            return

        if key == "HEAT":
            self.status = self.heater.set_enabled(not self.heater.enabled)
            return

        if key == "00":
            return
        self.temp_entry = self._edit(self.temp_entry, key, MAX_TEMP_CHARS)

    @staticmethod
    def _edit(buf, key, max_chars):
        """Keypad editing of a digit string."""
        if key == "<":
            return buf[:-1]
        if key == "00":
            if buf and buf != "0":
                return (buf + "00")[:max_chars]
            return buf
        if key in "0123456789" and len(key) == 1 and len(buf) < max_chars:
            return key if buf in ("", "0") else buf + key
        return buf

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
            self.draw_status(0.0, self.heater.snapshot(), force=True)
            print(self.act.message)
            self.heater.close()
            return

        last_state = None
        last_cycle_state = None

        try:
            # Heating starts here (if settings.json says it was on) and runs
            # in the heater's own threads from now on.
            self.heater.start()
            self._start_home()

            while True:
                # Both pollers, every iteration. act.poll() also feeds the
                # Tic's command-timeout watchdog, so it is never optional.
                state = self.act.poll()
                cycle_state = self.cycle.poll()
                # The heater switches itself off if this stops coming.
                self.heater.heartbeat()

                # After a homing run, back to rest -- once the plunger has
                # stopped, never while it moves.
                if self._valve_back_to_rest and not self.act.busy:
                    self._valve_back_to_rest = False
                    self._valve_to(valve_mod.REST_POSITION)

                # One position read per iteration (a USB transfer), shared by
                # the feed-forward bookkeeping and the status row.
                pos = self.act.position_mm
                self._track_dose(pos)

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

                snap = self.heater.snapshot()
                self.refresh_action_buttons(snap)
                self.refresh_valve_button()
                self.refresh_heat_button(snap)
                self.refresh_tile(snap)
                self.draw_entry()
                self.draw_status(pos, snap)
                self.draw_trend(snap)

                time.sleep(0.02)

        except KeyboardInterrupt:
            print("\nstopping")
            self.cycle.abort("interrupted")
            self.act.stop()
            while self.act.state == STOPPING:
                self.act.poll()
                time.sleep(0.02)
        finally:
            # Heater off first: it is the one output that does harm if left
            # on. Then, as before, the plunger stops (de-energised) before the
            # valve moves, the valve is left at rest, then everything is
            # released.
            try:
                self.heater.close()
            except Exception as exc:
                print(f"could not close the heater cleanly: {exc}")
            self.act.close()
            try:
                self.valves.to_rest()
            except Exception as exc:
                print(f"could not put the valve to rest: {exc}")
            self.valves.close()
            self.tft.close()


def _install_signal_handlers():
    """SIGTERM (systemctl stop, kill) and SIGHUP (the SSH session closed)
    would otherwise end the program on the spot, skipping the shutdown in
    run() -- and a GPIO keeps its last level when the process dies, so the
    heater could stay on. Both are turned into the same clean stop as
    Ctrl+C. A SIGHUP that is being ignored (nohup) is left ignored."""
    def _interrupt(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _interrupt)
    if signal.getsignal(signal.SIGHUP) is not signal.SIG_IGN:
        signal.signal(signal.SIGHUP, _interrupt)


def main():
    _install_signal_handlers()
    print(f"syringe bore {SYRINGE_BORE_MM}mm -> "
          f"{MM_PER_UL:.4f} mm per {MICRO}l "
          f"({1.0 / MM_PER_UL:.1f} {MICRO}l per mm)")
    print(f"valve: {'SIMULATED' if valve_mod.SIMULATE else 'GPIO'}, "
          f"rest {valve_mod.REST_POSITION.upper()}, "
          f"IN1 -> {valve_mod.IN1_SETS.upper()}, "
          f"pulse {valve_mod.PULSE_S * 1000:.0f} ms "
          f"(pulse length and IN1 mapping still to be verified)")
    saved = heater_mod.load_settings()
    print(f"heater: {'SIMULATED' if heater_mod.SIMULATE else 'DS18B20 + GPIO'}, "
          f"setpoint {saved.get('setpoint_c', heater_mod.DEFAULT_SETPOINT_C)} {DEG}C, "
          f"{'ON' if saved.get('heater_on') else 'off'} at start, "
          f"offset {saved.get('sensor_offset_k', 0.0):+.2f} K, "
          f"PI {heater_mod.KP_W_PER_K:g} W/K / {heater_mod.TI_S:g} s, "
          f"feed-forward x{heater_mod.FF_GAIN:g}")
    JogUI().run()


if __name__ == "__main__":
    main()
