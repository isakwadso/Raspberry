#!/usr/bin/env python3
"""
Temperature control of the heater block, as built (Nov 2026): an aluminium
cylinder about 60 mm in diameter and 80 mm long with 10 turns of stainless
tube, about 1 kg in all. A 24 V 60 W cartridge heater sits in the middle at
the top (inlet end), and one DS18B20 at the bottom (outlet end), between the
axis and the side. The heater is switched by a logic-level MOSFET on GPIO22.

WHAT RUNS WHERE
    Two background threads, so nothing here ever blocks jog_ui.py's loop:

      sensor thread   reads the DS18B20 through the kernel's 1-Wire driver.
                      One read blocks for ~0.75 s (12-bit conversion) -- far
                      too long for the 20 ms UI loop, which must also keep the
                      Tic's 1 s command timeout fed.
      control thread  every WINDOW_S works out a heater power (PI plus dose
                      feed-forward) and holds the gate on for that fraction of
                      the window ("time-proportioning"). The block's time
                      constants are minutes, so a 2 s on/off window is
                      invisible to it, and a 0.3 s valve pulse in the UI loop
                      does not disturb it.

    The UI calls heartbeat() every loop, reads snapshot(), and uses the
    setters. Nothing else.

CONTROL
    PI on the block probe, output 0..HEATER_MAX_W:
      * P = KP_W_PER_K * (setpoint - T)
      * I integrates only within INTEGRAL_BAND_K of the setpoint, so the long
        heat-up cannot wind it up. It starts preloaded with the estimated
        steady loss (LOSS_W_PER_K), so the loop does not spend ten minutes
        finding it.
    The heater can only heat. Cooling is by losses alone -- roughly 0.1 K per
    minute at 60 C for an insulated HX1-type block -- so an overshoot takes a
    long time to go away. The gains err on the slow side for that reason.

    The gains are STARTING VALUES from a lumped model of the block (SimBlock
    below), not from measurements on your hardware. Check them on the first
    real heat-up (see TUNING at the end of this docstring).

FEED-FORWARD
    Every dose pushed through the target line replaces V of hot liquid in the
    coil with V of cold liquid from the syringe. The cold liquid starts
    taking heat out of the block the moment it enters, long before the dip
    reaches the probe, so the heater has to start replacing that heat
    without waiting for the probe.

    jog_ui.py reports the liquid AS IT MOVES with add_dose(volume_ul): many
    small pieces during a push, and negative pieces for liquid drawn back
    out of the block (the pull-back after each dose). The energy to bring
    that liquid to the setpoint,
        E = FF_GAIN * V * LIQUID_J_PER_ML_K * (setpoint - INLET_TEMP_C),
    is queued per control window and delivered on top of the PI output, at
    a constant rate over FF_SPREAD_S from the window it came in. So for a
    7.5 ml dose at 60 C the heater gets about 1050 J extra -- about 35 W
    for 30 s -- starting within 2 s of the plunger starting to move (the
    push itself takes about 7 s). A pull-back
    takes its share off what is still queued. Energy that does not fit
    under HEATER_MAX_W is carried over, not lost.
    FF_GAIN is below 1 on purpose: under-compensating costs a small dip that
    the PI removes; over-compensating costs an overshoot that only the losses
    remove.

    In the block model (SimBlock), ten 7.5 ml doses a minute apart take the
    probe down about 2.3 K without feed-forward. With it they stay within
    about -0.25/+0.10 K at FF_GAIN 0.9 (the missing 10 % adds up from dose
    to dose until the PI catches up) and within about +-0.08 K at 1.0. So
    when it is right the probe moves only a sensor step or two: watch the
    heater power on the temperature screen, the "FF ... J" note there and
    in the status line, or the log (below), rather than the tile.

LOG
    One CSV line per control window (2 s) in logs/heater_YYYY-MM-DD.csv next
    to this file: time, probe temperature, setpoint, state, fault, PI power,
    feed-forward power, total power, feed-forward still queued, and the
    volume reported in that window. About 2 MB a day; files older than
    LOG_KEEP_DAYS are deleted at start. LOG_ENABLED = False switches it off.

READY
    |T - setpoint| <= STABLE_BAND_K without a break for STABLE_TIME_S.

FAULTS -- the heater is off while any of these is active
    no sensor   no valid reading for SENSOR_TIMEOUT_S. Clears by itself when
                valid readings come back.
    over temp   above setpoint + OVERTEMP_MARGIN_K, or above ABS_MAX_C.
    no heat     the PI has asked for >= NO_HEAT_POWER_FRAC of full power for
                NO_HEAT_S with less than NO_HEAT_MIN_RISE_K of rise: heater
                open circuit, KSD301 cutout tripped, or the 24 V supply off.
    ui silent   heartbeat() not called for HEARTBEAT_TIMEOUT_S: the UI loop
                has hung, so nobody would see the tile.
    controller  the control thread itself crashed (a bug). Needs a restart.
    All but "no sensor" are LATCHED: they stay until set_enabled(False) or a
    new set_setpoint(), i.e. until someone has looked at the screen.

WHAT SOFTWARE CANNOT COVER
    If the whole Python process hangs with the gate high, or the Pi locks up
    without losing power, nothing here can switch the heater off. The KSD301
    manual-reset cutout and the thermal fuse in series with the heater are
    the real protection. Never run the heater without them.

WIRING
    DS18B20   VDD -> 3.3 V (header pin 1), DQ -> GPIO4 (pin 7), GND -> pin 6,
              4.7 kOhm from DQ to 3.3 V. Never 5 V: DQ goes straight into a
              3.3 V GPIO input.
    /boot/firmware/config.txt:   dtoverlay=w1-gpio     (GPIO4 is its default)
    Reboot, then check:          ls /sys/bus/w1/devices/
                                 -> one 28-xxxxxxxxxxxx entry
    Heater    GPIO22 (pin 15) -> MOSFET gate, with a gate pull-down resistor
              on the MOSFET board. GPIO22 is pulled down while the Pi boots.

SETTINGS
    With the read-only overlay file system on (operation), changes are kept
    only until power-off, and the UI says so; set the setpoint and heater
    state you want at every power-on with the overlay off.
    settings.json next to this file holds setpoint_c, heater_on and
    sensor_offset_k. It is written (atomically: temp file, fsync, rename)
    only when the setpoint or on/off changes, so a power cut cannot leave it
    half-written and the SD card sees a handful of writes a year.
    sensor_offset_k is edited by hand: compare the probe with a reference
    thermometer at the working temperature and put the difference here
    (reference minus probe). Restart to apply it.

ACCURACY
    The DS18B20 reads in 0.0625 K steps and is specified to +-0.5 K. That is
    good enough to see that the block has settled, and with sensor_offset_k
    to put the setpoint within about 0.1-0.2 K of the reference thermometer.
    It cannot by itself verify a 0.1 K requirement at the nozzle tip.

TUNING
    First real heat-up -- watch the trend screen or the log from cold:
      * Overshoot more than ~0.5 K   -> lower KP_W_PER_K (e.g. 8 -> 5).
      * Slow oscillation, period of a few minutes, that does not die out
                                     -> lower KP_W_PER_K and raise TI_S.
      * Settles below setpoint for a long time -> lower TI_S a little.
    Feed-forward -- with the tile READY for at least 5 minutes, run 10 or
    more INJECT cycles of the volume you normally use, about a minute
    apart (larger volumes give a clearer signal), leave it 2 more minutes,
    then on the Pi:
        python3 heater.py --report
    It prints each dose's probe excursion, the response averaged over the
    doses, and an FF_GAIN estimate from an energy balance over the whole
    session: feed-forward energy + the PI's extra energy compared with the
    5 minutes before - block heat capacity x temperature change. In a
    steady series the PI ends up supplying whatever the feed-forward
    misses, so its extra energy is the feed-forward's error. Set FF_GAIN
    to a little below the estimate, then, in the averaged curve:
      * Dip that the PI fills in slowly        -> raise FF_GAIN.
      * Bump above the setpoint                -> lower FF_GAIN.
      * Dip first, then a bump (net about zero) -> the heat comes too late:
                                                  shorten FF_SPREAD_S.
      * Bump first, then a dip                 -> lengthen FF_SPREAD_S.
    Change one thing at a time; FF_GAIN first. The DS18B20 reads in 0.0625 K
    steps, so a single dose says little -- trust the average.

BENCH TEST
    python3 heater.py
prints the sensor reading and the controller state once a second (heater
off unless you pass --heat).
    python3 heater.py --report [logs/heater_YYYY-MM-DD.csv]
analyses the doses in a log (default: the newest).
"""

import atexit
import bisect
import glob
import json
import os
import threading
import time
import traceback
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timedelta
from pathlib import Path

import pins


# ===========================================================================
# CONFIGURATION
# ===========================================================================
# True: no GPIO and no 1-Wire; a thermal model of the block stands in for
# both. For running the UI on a desk.
SIMULATE = False
SIM_START_C = 21.0              # simulated block temperature at start

# -- setpoint ---------------------------------------------------------------
SETPOINT_MIN_C = 20
# The KSD301 cutout is 75 C nominal, and these parts are commonly specified
# to +-5 C. A setpoint above 65 C risks tripping one at the low end of its
# tolerance.
SETPOINT_MAX_C = 65
DEFAULT_SETPOINT_C = 60         # used only when there is no settings.json

# -- heater -----------------------------------------------------------------
# What the cartridge really delivers with the gate on: rated power at 24 V
# (power goes with V^2). Every watt the controller asks for is converted to
# on-time through this number, so it must match the heater fitted -- with
# 50 W here and a 60 W cartridge, everything (P, I and feed-forward) came
# out 20 % stronger than intended.
HEATER_MAX_W = 60.0
WINDOW_S = 2.0                  # time-proportioning window
MIN_SWITCH_S = 0.05             # shorter on/off slivers are not switched

# -- PI ---------------------------------------------------------------------
KP_W_PER_K = 8.0
TI_S = 1200.0
INTEGRAL_BAND_K = 1.5
# Estimated steady loss per kelvin above the room, used to preload the
# integrator. HX1 design note: about 2.3 W at 60 C in a 20 C room.
# Measured 26 Nov 2026: about 3.6 W held the block at ~59.9 C (estimate
# was 0.06 W/K).
LOSS_W_PER_K = 0.09

# -- feed-forward -----------------------------------------------------------
INLET_TEMP_C = 21.0             # syringe, valve and pouch are at room temp
# Volumetric heat capacity of the dosed liquid. About 4.0 J/(ml K) for 1 M
# KOH (density ~1.05 g/ml, cp ~3.8-3.9 J/(g K)); water is 4.18.
LIQUID_J_PER_ML_K = 4.0
# Starting value, on the safe side. The block model says about 1.0; the
# real number comes from `python3 heater.py --report` after a dosing run.
FF_GAIN = 0.9
# The heat each window's liquid needs is delivered over this long. About the
# time the cold liquid takes to come to block temperature plus the time heat
# takes to spread from the cartridge to the turns (~15 s through 30 mm of
# aluminium).
FF_SPREAD_S = 30.0
# Heat capacity of the block with its tube and liquid, J/K. Used only by
# --report to turn a temperature change into energy. Estimate for the
# as-built block: ~0.7 kg Al x 900 + ~0.3 kg steel x 500 + ~25 ml liquid x 4.
BLOCK_J_PER_K = 880.0

# -- log --------------------------------------------------------------------
LOG_ENABLED = True
LOG_KEEP_DAYS = 60

# -- ready ------------------------------------------------------------------
STABLE_BAND_K = 0.15            # about +-2 sensor steps
STABLE_TIME_S = 300.0

# -- faults -----------------------------------------------------------------
SENSOR_TIMEOUT_S = 5.0
OVERTEMP_MARGIN_K = 3.0
ABS_MAX_C = 70.0
NO_HEAT_POWER_FRAC = 0.8
NO_HEAT_S = 300.0
NO_HEAT_MIN_RISE_K = 1.0
HEARTBEAT_TIMEOUT_S = 5.0
# A reading this far from the previous one is accepted only when the next
# reading confirms it. The block cannot change by more than ~0.03 K/s.
JUMP_REJECT_K = 2.0
PLAUSIBLE_C = (-10.0, 100.0)

# None: exactly one DS18B20 is expected on the bus. With several on the bus,
# put the block probe's id here, e.g. "28-3c01d607d4a1".
SENSOR_ID = None

# -- trend ------------------------------------------------------------------
HISTORY_S = 1800.0
HISTORY_STEP_S = 10.0

SETTINGS_FILE = Path(__file__).resolve().with_name("settings.json")
LOG_DIR = Path(__file__).resolve().with_name("logs")
W1_DEVICES = "/sys/bus/w1/devices"
# ===========================================================================


# States
OFF = "off"
HEATING = "heating"
COOLING = "cooling"
SETTLING = "settling"
READY = "ready"
FAULT = "fault"

# Faults
F_NO_SENSOR = "no sensor"
F_OVER_TEMP = "over temp"
F_NO_HEAT = "no heat"
F_UI_SILENT = "ui silent"
F_CONTROLLER = "controller"


def _clamp(value, lo, hi):
    return lo if value < lo else hi if value > hi else value


# ---------------------------------------------------------------------------
# Settings file
# ---------------------------------------------------------------------------

def root_is_read_only():
    """True when the Pi runs with the read-only overlay file system
    (raspi-config -> Performance Options -> Overlay File System). Everything
    written -- settings, logs, code -- then lives in RAM and is gone at the
    next power-off; the card itself is never written, so pulling the plug
    is safe."""
    try:
        with open("/proc/mounts", "r", encoding="utf-8") as f:
            return any(len(p) > 2 and p[1] == "/" and p[2] == "overlay"
                       for p in (line.split() for line in f))
    except OSError:
        return False


ROOT_READ_ONLY = root_is_read_only()


def load_settings(path=None):
    """Return the saved settings as a dict, or {} if there are none. A
    corrupt file is reported and ignored (it is overwritten on the next
    change)."""
    path = SETTINGS_FILE if path is None else path
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return data if isinstance(data, dict) else {}
    except FileNotFoundError:
        return {}
    except (OSError, ValueError) as exc:
        print(f"  [heater] ignoring unreadable {path.name}: {exc}")
        return {}


def save_settings(updates, path=None):
    """Merge `updates` into the settings file. Atomic: a power cut leaves
    either the old file or the new one, never half of one. Keys this module
    does not know are kept, so other settings can share the file later."""
    path = SETTINGS_FILE if path is None else path
    data = load_settings(path)
    data.update(updates)
    tmp = path.with_name(path.name + ".tmp")
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, sort_keys=True)
        f.write("\n")
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)
    try:
        # Make the rename itself durable.
        fd = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
    except OSError:
        pass


# ---------------------------------------------------------------------------
# The controller -- pure arithmetic, no threads and no hardware
# ---------------------------------------------------------------------------

class Controller:
    """PI + feed-forward + ready detection + fault logic. update() is called
    once per window with the latest valid temperature (or None) and returns
    the heater power in W. Kept free of I/O so it can be simulated faster
    than real time."""

    def __init__(self, setpoint_c, enabled):
        self.setpoint_c = setpoint_c
        self.enabled = enabled
        self.latched = None
        self._reset_loop()

    def _reset_loop(self):
        self._integral_w = None         # preloaded on the first update
        self._clear_ff()
        self._dose_ul = 0.0             # volume reported since the last update
        self._band_since = None
        self._noheat = None             # (start time, start temperature)
        self.state = OFF if not self.enabled else HEATING
        self.fault = None
        self.power_w = 0.0
        self.pi_w = 0.0
        self.ff_w = 0.0
        self.window_dose_ul = 0.0       # volume reported in the last window
        self.in_band_s = 0.0

    def _clear_ff(self):
        self._ff_new_j = 0.0            # reported since the last update
        self._ff_packets = []           # [[remaining J, rate W], ...]

    # -- commands -----------------------------------------------------------

    def set_setpoint(self, setpoint_c):
        self.setpoint_c = setpoint_c
        self.enabled = True
        self.latched = None
        self._band_since = None
        self._noheat = None
        self._integral_w = None         # new setpoint, new loss estimate

    def set_enabled(self, enabled):
        self.enabled = enabled
        self.latched = None
        if not enabled:
            self._reset_loop()

    def latch(self, fault):
        if self.latched is None:
            self.latched = fault

    def add_dose(self, volume_ul):
        """Report liquid moved through the target line, in ul: positive for
        liquid pushed into the block, negative for liquid drawn back out of
        it. May be called many times per stroke with small pieces. Returns
        the energy queued in J (negative: taken off the queue; 0 if the
        heater is off or faulted)."""
        self._dose_ul += volume_ul
        if not self.enabled or self.latched or self.state == FAULT:
            return 0.0
        dt_k = max(0.0, self.setpoint_c - INLET_TEMP_C)
        energy = FF_GAIN * (volume_ul / 1000.0) * LIQUID_J_PER_ML_K * dt_k
        if energy >= 0.0:
            self._ff_new_j += energy
            return energy
        # Liquid drawn back out: take its energy off what is still queued,
        # newest first. Never below zero -- heat already delivered stays.
        take = -energy
        removed = min(take, self._ff_new_j)
        self._ff_new_j -= removed
        take -= removed
        for packet in reversed(self._ff_packets):
            if take <= 0.0:
                break
            r = min(take, packet[0])
            packet[0] -= r
            take -= r
            removed += r
        self._ff_packets = [p for p in self._ff_packets if p[0] > 1e-6]
        return -removed

    @property
    def ff_pending_j(self):
        return self._ff_new_j + sum(p[0] for p in self._ff_packets)

    # -- the update ---------------------------------------------------------

    def update(self, now, temp_c, dt):
        self.window_dose_ul, self._dose_ul = self._dose_ul, 0.0
        self.pi_w = self.ff_w = 0.0
        if not self.enabled:
            self.state, self.fault = OFF, None
            self.power_w = 0.0
            self._clear_ff()
            self.in_band_s = 0.0
            return 0.0

        fault = self.latched
        if fault is None and temp_c is None:
            fault = F_NO_SENSOR
        if fault is None and temp_c > min(self.setpoint_c + OVERTEMP_MARGIN_K,
                                          ABS_MAX_C):
            self.latch(F_OVER_TEMP)
            fault = F_OVER_TEMP
        if fault is not None:
            return self._faulted(fault)

        err = self.setpoint_c - temp_c

        if self._integral_w is None:
            self._integral_w = _clamp(
                LOSS_W_PER_K * (self.setpoint_c - INLET_TEMP_C),
                0.0, HEATER_MAX_W)
        if abs(err) < INTEGRAL_BAND_K:
            self._integral_w = _clamp(
                self._integral_w + KP_W_PER_K * err * dt / TI_S,
                0.0, HEATER_MAX_W)
        pi_w = _clamp(KP_W_PER_K * err + self._integral_w, 0.0, HEATER_MAX_W)

        # Feed-forward: what came in since the last window becomes one
        # packet, delivered at a constant rate over FF_SPREAD_S. Packets
        # overlap when doses follow closely; the rates then add up.
        if self._ff_new_j > 0.0:
            self._ff_packets.append([self._ff_new_j,
                                     self._ff_new_j / FF_SPREAD_S])
            self._ff_new_j = 0.0
        ff_w = 0.0
        if self._ff_packets:
            want = [min(rate, left / dt) for left, rate in self._ff_packets]
            total = sum(want)
            ff_w = max(0.0, min(total, HEATER_MAX_W - pi_w))
            scale = ff_w / total if total > 0.0 else 0.0
            for packet, w in zip(self._ff_packets, want):
                packet[0] -= w * scale * dt
            self._ff_packets = [p for p in self._ff_packets if p[0] > 1e-6]

        # No-heat check: the PI has been asking for most of full power for a
        # long time and the block has hardly warmed. Only while no dose is
        # being compensated, so a cold dose cannot be mistaken for it.
        if (pi_w >= NO_HEAT_POWER_FRAC * HEATER_MAX_W
                and self.ff_pending_j == 0.0):
            if self._noheat is None:
                self._noheat = (now, temp_c)
            elif now - self._noheat[0] >= NO_HEAT_S:
                if temp_c - self._noheat[1] < NO_HEAT_MIN_RISE_K:
                    self.latch(F_NO_HEAT)
                    return self._faulted(F_NO_HEAT)
                self._noheat = (now, temp_c)
        else:
            self._noheat = None

        if abs(err) <= STABLE_BAND_K:
            if self._band_since is None:
                self._band_since = now
            self.in_band_s = now - self._band_since
        else:
            self._band_since = None
            self.in_band_s = 0.0

        if self._band_since is not None:
            self.state = READY if self.in_band_s >= STABLE_TIME_S else SETTLING
        else:
            self.state = HEATING if err > 0 else COOLING
        self.fault = None
        self.pi_w, self.ff_w = pi_w, ff_w
        self.power_w = pi_w + ff_w
        return self.power_w

    def _faulted(self, fault):
        self.state, self.fault = FAULT, fault
        self.power_w = 0.0
        self._clear_ff()
        self._band_since = None
        self._noheat = None
        self.in_band_s = 0.0
        # The integrator is kept: after a sensor dropout the loop picks up
        # where it was.
        return 0.0


# ---------------------------------------------------------------------------
# Simulated block, for SIMULATE and for checking the tuning
# ---------------------------------------------------------------------------

class SimBlock:
    """Two-node lumped model of an insulated aluminium heater block.

    H: the top (inlet) half with the heater; O: the bottom (outlet) half
    with the probe. A dose's cold liquid lands in the inlet turns, so most of
    its heat is drawn from H. The probe has its own first-order lag and reads
    in 0.0625 K steps. The numbers are estimates for the as-built block
    (Al cylinder ~60 x 80 mm with 10 turns of 6 x 1 steel tube, ~1 kg),
    not measurements."""

    C_H = 440.0             # J/K
    C_O = 440.0             # J/K
    G_HO = 12.0             # W/K: 6082 through a 60 mm disc over 40 mm
    G_AMB = 0.045           # W/K from each half to the room (measured total 0.09)
    TAU_PROBE_S = 20.0
    DOSE_FRACTION_H = 0.9
    DOSE_SPREAD_S = 30.0    # how fast a cold dose takes up its heat
    STEP_S = 0.5

    def __init__(self, start_c=None, ambient_c=None):
        start_c = SIM_START_C if start_c is None else start_c
        ambient_c = INLET_TEMP_C if ambient_c is None else ambient_c
        self.ambient = ambient_c
        self.t_h = self.t_o = self.t_probe = start_c
        self._sinks = []        # [remaining J, rate W]
        self._incoming_j = 0.0  # reported since the last step
        self.heater_ok = True   # False simulates a tripped cutout

    def add_dose(self, volume_ul):
        """Liquid entering (or, negative, leaving) the block. Called in
        small pieces as the plunger moves, like the controller's."""
        self._incoming_j += ((volume_ul / 1000.0) * LIQUID_J_PER_ML_K
                             * (self.t_h - INLET_TEMP_C))

    def step(self, dt, heater_w):
        heater_w = heater_w if self.heater_ok else 0.0
        if self._incoming_j > 0:
            self._sinks.append([self._incoming_j,
                                self._incoming_j / self.DOSE_SPREAD_S])
        elif self._incoming_j < 0 and self._sinks:
            # Drawn back before it had taken up its heat.
            self._sinks[-1][0] = max(0.0, self._sinks[-1][0] + self._incoming_j)
        self._incoming_j = 0.0
        t = 0.0
        while t < dt - 1e-9:
            h = min(self.STEP_S, dt - t)
            sink = 0.0
            for s in self._sinks:
                take = min(s[1], s[0] / h)
                s[0] -= take * h
                sink += take
            self._sinks = [s for s in self._sinks if s[0] > 1e-6]
            q = self.G_HO * (self.t_h - self.t_o)
            self.t_h += (heater_w - self.DOSE_FRACTION_H * sink - q
                         - self.G_AMB * (self.t_h - self.ambient)) * h / self.C_H
            self.t_o += (q - (1 - self.DOSE_FRACTION_H) * sink
                         - self.G_AMB * (self.t_o - self.ambient)) * h / self.C_O
            self.t_probe += (self.t_o - self.t_probe) * h / self.TAU_PROBE_S
            t += h

    def read(self):
        return round(self.t_probe / 0.0625) * 0.0625


# ---------------------------------------------------------------------------
# Log, for tuning
# ---------------------------------------------------------------------------

LOG_COLUMNS = ("time", "temp_c", "setpoint_c", "state", "fault", "pi_w",
               "ff_w", "power_w", "ff_pending_j", "dose_ul")


def _log_params():
    return (f"# FF_GAIN={FF_GAIN:g} FF_SPREAD_S={FF_SPREAD_S:g} "
            f"HEATER_MAX_W={HEATER_MAX_W:g} KP_W_PER_K={KP_W_PER_K:g} "
            f"TI_S={TI_S:g} INLET_TEMP_C={INLET_TEMP_C:g} "
            f"LIQUID_J_PER_ML_K={LIQUID_J_PER_ML_K:g}")


class CsvLog:
    """One CSV file per day in LOG_DIR. Written from the control thread, a
    line every window, flushed to the OS each time (no fsync: losing the
    last half-minute in a power cut does not matter, and fsync every 2 s
    would wear the SD card). Survives a cut: a half-written last line is
    closed off on the next start and skipped by --report. Any I/O error
    switches the log off with one message; it never stops the control
    loop."""

    def __init__(self, folder):
        self.folder = Path(folder)
        self._f = None
        self._day = None
        self.failed = False
        try:
            self.folder.mkdir(exist_ok=True)
            self._prune()
        except OSError as exc:
            self._fail(exc)

    def _prune(self):
        cutoff = (datetime.now() - timedelta(days=LOG_KEEP_DAYS)).strftime("%Y-%m-%d")
        for p in self.folder.glob("heater_????-??-??.csv"):
            if p.stem[len("heater_"):] < cutoff:
                p.unlink()

    def _open(self, day):
        self.close()
        path = self.folder / f"heater_{day}.csv"
        new = not path.exists()
        # A power cut can leave the last line half written. Start on a
        # fresh line so the next one is not glued to it (--report skips
        # the broken line).
        broken_tail = False
        if not new:
            with open(path, "rb") as f:
                f.seek(0, os.SEEK_END)
                if f.tell() > 0:
                    f.seek(-1, os.SEEK_END)
                    broken_tail = f.read(1) != b"\n"
        self._f = open(path, "a", encoding="utf-8")
        if new:
            self._f.write(",".join(LOG_COLUMNS) + "\n")
        elif broken_tail:
            self._f.write("\n")
        # Parameters at every (re)start, so --report knows which gain a
        # stretch of log was recorded with.
        self._f.write(_log_params() + "\n")
        self._day = day

    @staticmethod
    def row(now, temp_c, ctl):
        """(day, line) for one window. Cheap, no I/O: called under the
        heater's lock."""
        temp = "" if temp_c is None else f"{temp_c:.4f}"
        line = (f"{now:%Y-%m-%d %H:%M:%S}.{now.microsecond // 100000},"
                f"{temp},{ctl.setpoint_c},{ctl.state},{ctl.fault or ''},"
                f"{ctl.pi_w:.2f},{ctl.ff_w:.2f},{ctl.power_w:.2f},"
                f"{ctl.ff_pending_j:.1f},{ctl.window_dose_ul:.1f}\n")
        return now.strftime("%Y-%m-%d"), line

    def write(self, row):
        if self.failed:
            return
        day, line = row
        try:
            if day != self._day:
                self._open(day)
            self._f.write(line)
            self._f.flush()
        except OSError as exc:
            self._fail(exc)

    def _fail(self, exc):
        print(f"  [heater] log switched off: {exc}")
        self.failed = True
        self.close()

    def close(self):
        if self._f is not None:
            try:
                self._f.close()
            except OSError:
                pass
            self._f = None
            self._day = None


# ---------------------------------------------------------------------------
# The heater
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Snapshot:
    temp_c: object          # float, or None without a valid reading
    setpoint_c: int
    enabled: bool
    state: str
    fault: object           # str, or None
    power_frac: float
    in_band_s: float
    ff_pending_j: float     # feed-forward energy still to deliver, J
    history_version: int
    ff_w: float = 0.0       # feed-forward part of the present power, W


class Heater:
    def __init__(self, verbose=True):
        self.verbose = verbose
        settings = load_settings()
        try:
            setpoint = int(settings.get("setpoint_c", DEFAULT_SETPOINT_C))
        except (TypeError, ValueError):
            setpoint = DEFAULT_SETPOINT_C
        setpoint = int(_clamp(setpoint, SETPOINT_MIN_C, SETPOINT_MAX_C))
        try:
            self._offset = float(settings.get("sensor_offset_k", 0.0))
        except (TypeError, ValueError):
            self._offset = 0.0
        self._ctl = Controller(setpoint, bool(settings.get("heater_on", False)))

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._threads = []
        self._crashed = False

        self._temp = None
        self._temp_time = float("-inf")
        self._last_raw = None
        self._jump_pending = None
        self.sensor_note = ""           # why the last read failed, if it did
        self._resolution_warned = False

        self._last_heartbeat = None
        self._history = deque(maxlen=int(HISTORY_S / HISTORY_STEP_S))
        self._history_version = 0
        self._next_history = 0.0

        self._log = CsvLog(LOG_DIR) if LOG_ENABLED else None

        self._gate = None
        self._gate_on = False
        self._sim = SimBlock() if SIMULATE else None
        if not SIMULATE:
            # Imported here so SIMULATE works on a machine without Blinka.
            import board
            import digitalio
            gate = digitalio.DigitalInOut(getattr(board, f"D{pins.HEATER_GATE}"))
            gate.switch_to_output(value=False)
            self._gate = gate

        # Backstop: whatever path the program leaves by, the gate goes low.
        atexit.register(self.close)

    # -- lifecycle ------------------------------------------------------------

    def start(self):
        if self._threads:
            return
        self.heartbeat()
        for target, name in ((self._sensor_loop, "heater-sensor"),
                             (self._control_loop, "heater-control")):
            th = threading.Thread(target=target, name=name, daemon=True)
            th.start()
            self._threads.append(th)

    def close(self):
        """Stop both threads and leave the gate low. Safe to call twice."""
        self._stop.set()
        for th in self._threads:
            th.join(timeout=WINDOW_S + 1.5)
        self._threads = []
        self._set_gate(False)
        if self._log is not None:
            self._log.close()
        if self._gate is not None:
            try:
                self._gate.deinit()
            except Exception:
                pass
            self._gate = None

    def heartbeat(self):
        """Call every UI loop iteration."""
        self._last_heartbeat = time.monotonic()

    # -- commands from the UI -------------------------------------------------

    @property
    def enabled(self):
        return self._ctl.enabled

    def set_setpoint(self, setpoint_c):
        """Set, save and switch heating on. Returns (ok, message)."""
        if not SETPOINT_MIN_C <= setpoint_c <= SETPOINT_MAX_C:
            return False, f"range {SETPOINT_MIN_C}-{SETPOINT_MAX_C} \u00b0C"
        with self._lock:
            self._ctl.set_setpoint(int(setpoint_c))
        return True, self._save(f"heating to {setpoint_c} \u00b0C")

    def set_enabled(self, enabled):
        """Switch heating on or off and save. Returns a message."""
        with self._lock:
            self._ctl.set_enabled(bool(enabled))
            sp = self._ctl.setpoint_c
        return self._save(f"heating to {sp} \u00b0C" if enabled else "heater off")

    def add_dose(self, volume_ul):
        """Report liquid moved through the target line as it moves: positive
        into the block, negative back out of it. Called many times per
        stroke, so it prints nothing; jog_ui.py prints a summary per cycle.
        Returns the feed-forward energy queued (negative: removed), in J."""
        with self._lock:
            energy = self._ctl.add_dose(volume_ul)
            if self._sim is not None:
                self._sim.add_dose(volume_ul)
        return energy

    def _save(self, message):
        with self._lock:
            data = {"setpoint_c": self._ctl.setpoint_c,
                    "heater_on": self._ctl.enabled}
        try:
            save_settings(data)
            # Saved to RAM only: back to the card's settings at power-off.
            return message + (" (until power-off)" if ROOT_READ_ONLY else "")
        except OSError as exc:
            print(f"  [heater] could not save settings: {exc}")
            return message + " (NOT saved)"

    # -- reading out ----------------------------------------------------------

    def snapshot(self):
        with self._lock:
            ctl = self._ctl
            state, fault = ctl.state, ctl.fault
            if self._crashed:
                state, fault = FAULT, F_CONTROLLER
            return Snapshot(
                temp_c=self._fresh_temp(time.monotonic()),
                setpoint_c=ctl.setpoint_c,
                enabled=ctl.enabled,
                state=state,
                fault=fault,
                power_frac=ctl.power_w / HEATER_MAX_W,
                in_band_s=ctl.in_band_s,
                ff_pending_j=ctl.ff_pending_j,
                history_version=self._history_version,
                ff_w=ctl.ff_w)

    def history(self):
        """[(monotonic time, temperature C), ...], oldest first, one point
        per HISTORY_STEP_S over the last HISTORY_S."""
        with self._lock:
            return list(self._history)

    def _fresh_temp(self, now):
        if now - self._temp_time <= SENSOR_TIMEOUT_S:
            return self._temp
        return None

    # -- the gate -------------------------------------------------------------

    def _set_gate(self, on):
        self._gate_on = on
        if self._gate is not None:
            try:
                self._gate.value = on
            except Exception:
                pass

    # -- sensor thread ----------------------------------------------------------

    def _sensor_loop(self):
        while not self._stop.is_set():
            if self._sim is not None:
                self._stop.wait(0.75)
                with self._lock:
                    raw = self._sim.read()
            else:
                raw = self._read_ds18b20()      # blocks ~0.75 s
            value = self._screen(raw)
            with self._lock:
                if value is not None:
                    self._temp = value + self._offset
                    self._temp_time = time.monotonic()
            if raw is None:
                self._stop.wait(0.5)            # e.g. no device: do not spin

    def _screen(self, raw):
        """Reject implausible values and single-reading jumps."""
        if raw is None:
            return None
        if not PLAUSIBLE_C[0] <= raw <= PLAUSIBLE_C[1]:
            self.sensor_note = f"implausible {raw:.2f} C"
            return None
        if self._last_raw is not None and abs(raw - self._last_raw) > JUMP_REJECT_K:
            if self._jump_pending is None or abs(raw - self._jump_pending) > 0.5:
                self._jump_pending = raw
                self.sensor_note = f"jump to {raw:.2f} C, waiting for confirmation"
                return None
        self._jump_pending = None
        self._last_raw = raw
        self.sensor_note = ""
        return raw

    def _read_ds18b20(self):
        """One reading in C through the kernel w1_therm driver, or None.
        Reading w1_slave starts a conversion and blocks until it is done."""
        paths = sorted(glob.glob(f"{W1_DEVICES}/28-*/w1_slave"))
        if SENSOR_ID:
            paths = [p for p in paths if f"/{SENSOR_ID}/" in p]
        if not paths:
            self.sensor_note = "no DS18B20 on the 1-Wire bus"
            return None
        if len(paths) > 1:
            self.sensor_note = "several DS18B20s -- set SENSOR_ID in heater.py"
            return None
        try:
            with open(paths[0], "r") as f:
                lines = f.read().strip().splitlines()
        except OSError as exc:
            self.sensor_note = f"read failed: {exc}"
            return None
        # Line 1: "72 01 4b 46 7f ff 0e 10 57 : crc=57 YES"
        # Line 2: "72 01 4b 46 7f ff 0e 10 57 t=23125"
        if len(lines) < 2 or not lines[0].endswith("YES"):
            self.sensor_note = "CRC error"
            return None
        data = lines[0].split(":")[0].split()
        if all(b == "00" for b in data):
            # A data line held low reads as all zeros, whose CRC is also zero.
            self.sensor_note = "all-zero data (DQ shorted to GND?)"
            return None
        # Byte 4 is the configuration register: 0x7f = 12-bit (0.0625 K).
        # The block's first sensor came set to 10-bit (0.25 K steps), which
        # made the reading flip 59.75 <-> 60.00 and kept it from ever
        # reaching READY (STABLE_BAND_K is 0.15 K). Warn once, with the fix.
        if len(data) > 4 and not self._resolution_warned:
            bits = 9 + ((int(data[4], 16) >> 5) & 3)
            if bits < 12:
                self._resolution_warned = True
                print(f"  [heater] WARNING: DS18B20 set to {bits}-bit "
                      f"({0.0625 * 2 ** (12 - bits):g} K steps). Fix once, it "
                      f"is saved in the sensor:\n"
                      f"    echo 12   | sudo tee {W1_DEVICES}/28-*/resolution\n"
                      f"    echo save | sudo tee {W1_DEVICES}/28-*/eeprom_cmd")
        pos = lines[1].find("t=")
        if pos < 0:
            self.sensor_note = "no t= in w1_slave"
            return None
        milli = int(lines[1][pos + 2:])
        if milli == 85000:
            # The DS18B20's power-on value: a reset during the conversion.
            self.sensor_note = "85.000 C power-on value"
            return None
        return milli / 1000.0

    # -- control thread ---------------------------------------------------------

    def _control_loop(self):
        try:
            next_start = time.monotonic()
            last = next_start - WINDOW_S
            while not self._stop.is_set():
                now = time.monotonic()
                dt = _clamp(now - last, 0.1, 3 * WINDOW_S)
                last = now

                with self._lock:
                    if (self._last_heartbeat is not None
                            and now - self._last_heartbeat > HEARTBEAT_TIMEOUT_S
                            and self._ctl.enabled):
                        self._ctl.latch(F_UI_SILENT)
                    temp = self._fresh_temp(now)
                    power = self._ctl.update(now, temp, dt)
                    if temp is not None and now >= self._next_history:
                        self._next_history = now + HISTORY_STEP_S
                        self._history.append((now, temp))
                        self._history_version += 1
                    # Captured under the lock, written outside it: a slow SD
                    # card must never hold up snapshot() in the UI loop,
                    # which keeps the Tic's command timeout fed.
                    log_row = (None if self._log is None
                               else self._log.row(datetime.now(), temp, self._ctl))
                if log_row is not None:
                    self._log.write(log_row)

                on_s = power / HEATER_MAX_W * WINDOW_S
                if on_s < MIN_SWITCH_S:
                    on_s = 0.0
                elif on_s > WINDOW_S - MIN_SWITCH_S:
                    on_s = WINDOW_S

                if on_s > 0.0:
                    self._set_gate(True)
                    if on_s < WINDOW_S:
                        if self._stop.wait(on_s):
                            break
                        self._set_gate(False)
                else:
                    self._set_gate(False)

                if self._sim is not None:
                    with self._lock:
                        self._sim.step(WINDOW_S, on_s / WINDOW_S * HEATER_MAX_W)

                next_start += WINDOW_S
                delay = next_start - time.monotonic()
                if delay < -WINDOW_S:           # fell far behind: resync
                    next_start = time.monotonic()
                    delay = 0.0
                if self._stop.wait(max(delay, 0.0)):
                    break
        except Exception:
            traceback.print_exc()
            with self._lock:
                self._crashed = True
        finally:
            self._set_gate(False)


# ---------------------------------------------------------------------------
# Bench test
# ---------------------------------------------------------------------------

def bench_test(heat=False):
    print(f"Heater bench test ({'SIMULATED' if SIMULATE else 'DS18B20 + GPIO'}).")
    h = Heater()
    if not heat and h.enabled:
        # Off for the bench test unless asked. Not saved, so settings.json
        # keeps whatever the UI last stored.
        with h._lock:
            h._ctl.enabled = False
    h.start()
    try:
        while True:
            h.heartbeat()
            s = h.snapshot()
            t = "  --.---" if s.temp_c is None else f"{s.temp_c:8.3f}"
            note = f"  ({h.sensor_note})" if h.sensor_note else ""
            print(f"{t} C  set {s.setpoint_c}  {s.state:8s} "
                  f"{s.fault or '':10s} power {s.power_frac * 100:5.1f} %{note}")
            time.sleep(1.0)
    except KeyboardInterrupt:
        print()
    finally:
        h.close()


# ---------------------------------------------------------------------------
# Feed-forward report from a log
# ---------------------------------------------------------------------------

def _read_log(path):
    """[(datetime, temp or None, state, pi_w, ff_w, dose_ul, params,
    setpoint)]."""
    rows, params = [], {}
    # errors="replace": a power cut can leave garbage at the end of a log.
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("time,"):
                continue
            if line.startswith("#"):
                params = dict(kv.split("=", 1) for kv in line[1:].split() if "=" in kv)
                continue
            parts = line.split(",")
            if len(parts) != len(LOG_COLUMNS):
                continue
            try:
                ts = datetime.strptime(parts[0], "%Y-%m-%d %H:%M:%S.%f")
                temp = float(parts[1]) if parts[1] else None
                rows.append((ts, temp, parts[3], float(parts[5]),
                             float(parts[6]), float(parts[9]), params,
                             float(parts[2])))
            except ValueError:
                continue
    return rows


def _ff_queued(row, vol):
    """(FF_GAIN, feed-forward energy queued for vol ul) with the parameters
    the log line was recorded with."""
    p = row[6]
    gain = float(p.get("FF_GAIN", FF_GAIN))
    cp = float(p.get("LIQUID_J_PER_ML_K", LIQUID_J_PER_ML_K))
    t_in = float(p.get("INLET_TEMP_C", INLET_TEMP_C))
    return gain, gain * vol / 1000.0 * cp * max(0.0, row[7] - t_in)


def report(path=None, idle_s=300.0, tail_s=120.0, base_s=30.0, gap_s=300.0):
    """Analyse the doses in a log.

    Per dose: the probe's lowest and highest point relative to the base_s
    before the dose, and that response averaged over all doses -- its shape
    tells gain from timing (see TUNING).

    Per session (doses at most gap_s apart, at least 3 of them): an energy
    balance against the idle_s of steady regulation before the first dose,
    up to tail_s after the last one:
        heat the doses took = feed-forward energy delivered
                            + (PI power - PI power before) x time
                            - BLOCK_J_PER_K x (temperature after - before)
    In a steady series of doses the PI ends up supplying whatever the
    feed-forward misses, so its extra energy over the session is the
    feed-forward's error. FF_GAIN estimate = FF_GAIN x heat taken / FF
    energy queued.
    """
    if path is None:
        files = sorted(LOG_DIR.glob("heater_????-??-??.csv"))
        if not files:
            print(f"no logs in {LOG_DIR}")
            return
        path = files[-1]
    rows = _read_log(path)
    if not rows:
        print(f"{path}: no data")
        return
    t0 = rows[0][0]
    ts = [(r[0] - t0).total_seconds() for r in rows]

    def span(a, b):
        """Row indices with a <= t < b."""
        return range(bisect.bisect_left(ts, a), bisect.bisect_left(ts, b))

    # Doses: runs of windows with reported volume, gaps of at most 6 s.
    # [first row, last row, net volume]; the volume came in during the
    # window before each row, so a dose starts WINDOW_S before its first row.
    events, cur = [], None
    for i, r in enumerate(rows):
        if r[5] != 0.0:
            if cur is not None and ts[i] - ts[cur[1]] <= 6.0:
                cur[1] = i
                cur[2] += r[5]
            else:
                cur = [i, i, r[5]]
                events.append(cur)
    events = [e for e in events if e[2] > 0.0]
    print(f"{path}: {len(events)} doses")
    if not events:
        return
    starts = [ts[e[0]] - WINDOW_S for e in events]

    # -- per dose -----------------------------------------------------------
    bins = list(range(0, 130, 10))
    curves = []
    print(" time      volume   FF J    lowest       highest    "
          "(probe against the 30 s before)")
    for k, (i0, i1, vol) in enumerate(events):
        start = starts[k]
        end = min(start + bins[-1] + 10, starts[k + 1] if k + 1 < len(events)
                  else float("inf"))
        base, win = span(start - base_s, start), span(start, end)
        if (len(base) < 0.8 * base_s / WINDOW_S
                or any(rows[j][1] is None for j in list(base) + list(win))):
            continue
        b_t = sum(rows[j][1] for j in base) / len(base)
        devs = [(ts[j] - start, rows[j][1] - b_t) for j in win]
        lo = min(devs, key=lambda d: d[1])
        hi = max(devs, key=lambda d: d[1])
        print(f" {rows[i0][0]:%H:%M:%S} {vol:6.0f} ul {_ff_queued(rows[i0], vol)[1]:5.0f}"
              f"   {lo[1]:+.3f} {lo[0]:3.0f}s   {hi[1]:+.3f} {hi[0]:3.0f}s")
        curve = {}
        for d_t, dev in devs:
            curve.setdefault(int(d_t // 10) * 10, []).append(dev)
        curves.append({b: sum(v) / len(v) for b, v in curve.items()})
    if curves:
        print(f"averaged probe deviation after the dose ({len(curves)} doses), K:")
        cells = []
        for b in bins:
            # Only as far as most doses reach before the next one starts.
            vals = [c[b] for c in curves if b in c]
            if len(vals) >= max(1, len(curves) // 2):
                cells.append(f"{b:3d}s {sum(vals) / len(vals):+.3f}")
        for n in range(0, len(cells), 5):
            print("   " + "   ".join(cells[n:n + 5]))

    # -- per session --------------------------------------------------------
    sessions, cur = [], None
    for k in range(len(events)):
        if cur is not None and starts[k] - starts[cur[-1]] <= gap_s:
            cur.append(k)
        else:
            cur = [k]
            sessions.append(cur)
    for sess in sessions:
        s0, s1 = starts[sess[0]], starts[sess[-1]] + tail_s
        head = f"session {rows[events[sess[0]][0]][0]:%H:%M}, {len(sess)} doses:"
        if len(sess) < 3:
            print(f"{head} too few for an energy balance (3 or more)")
            continue
        idle, run = span(s0 - idle_s, s0), span(s0, s1)
        if (len(idle) < 0.8 * idle_s / WINDOW_S
                or any(rows[j][1] is None or rows[j][5] != 0.0
                       or rows[j][2] not in (READY, SETTLING) for j in idle)):
            print(f"{head} needs {idle_s / 60:.0f} min of steady regulation "
                  f"(READY/SETTLING, no doses) before the first dose")
            continue
        if ts[-1] < s1 - WINDOW_S or any(rows[j][1] is None or rows[j][2] == FAULT
                                         for j in run):
            print(f"{head} needs {tail_s:.0f} s of log after the last dose, "
                  f"without faults or sensor gaps")
            continue
        p_idle = sum(rows[j][3] for j in idle) / len(idle)
        t_before = sum(rows[j][1] for j in span(s0 - 60.0, s0)) / len(span(s0 - 60.0, s0))
        t_after = sum(rows[j][1] for j in span(s1 - 30.0, s1)) / len(span(s1 - 30.0, s1))
        e_pi = sum((rows[j][3] - p_idle) * WINDOW_S for j in run)
        e_ff = sum(rows[j][4] * WINDOW_S for j in run)
        queued = [_ff_queued(rows[events[k][0]], events[k][2]) for k in sess]
        gain = queued[-1][0]
        e_q = sum(q[1] for q in queued)
        e_block = BLOCK_J_PER_K * (t_after - t_before)
        e_took = e_ff + e_pi - e_block
        vol = sum(events[k][2] for k in sess)
        dt_k = rows[events[sess[0]][0]][7] - float(rows[events[sess[0]][0]][6].get(
            "INLET_TEMP_C", INLET_TEMP_C))
        print(head)
        print(f"   feed-forward {e_ff:.0f} J delivered ({e_q:.0f} J queued)")
        print(f"   PI {p_idle:.2f} W before, {e_pi / (s1 - s0):+.2f} W while dosing "
              f"-> {e_pi:+.0f} J; block {t_after - t_before:+.3f} K -> {e_block:+.0f} J")
        if e_q > 1.0 and dt_k > 0:
            print(f"   heat the doses took {e_took:.0f} J = "
                  f"{e_took / (vol / 1000.0) / dt_k:.2f} J/(ml K) of liquid "
                  f"(LIQUID_J_PER_ML_K = {LIQUID_J_PER_ML_K:g})")
            print(f"   FF_GAIN estimate {gain * e_took / e_q:.2f} (recorded with "
                  f"{gain:g}). Rests on BLOCK_J_PER_K = {BLOCK_J_PER_K:g} and "
                  f"HEATER_MAX_W matching the cartridge.")
            if abs(e_ff - e_q) > 0.05 * e_q:
                print("   NOTE: delivered and queued differ by more than 5 % "
                      "(heater at full power, or the heater was switched off)")
        else:
            print("   no feed-forward queued (heater off?)")


if __name__ == "__main__":
    import sys
    if "--report" in sys.argv:
        rest = sys.argv[sys.argv.index("--report") + 1:]
        report(rest[0] if rest else None)
    else:
        bench_test(heat="--heat" in sys.argv)
