#!/usr/bin/env python3
"""
Temperature control of the heater block: one DS18B20 in the block next to the
outlet turns, and a 24 V 50 W cartridge heater in the centre of the block on
the inlet side, switched by a logic-level MOSFET on GPIO22.

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
    coil with V of cold liquid from the syringe. jog_ui.py reports each such
    stroke with add_dose(volume_ul), and the energy to bring that liquid to
    the setpoint,
        E = FF_GAIN * V * LIQUID_J_PER_ML_K * (setpoint - INLET_TEMP_C),
    is delivered on top of the PI output over the next FF_SPREAD_S. Energy
    that does not fit under HEATER_MAX_W is carried over, not lost.
    FF_GAIN is below 1 on purpose: under-compensating costs a small dip that
    the PI removes; over-compensating costs an overshoot that only the losses
    remove.

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

TUNING (first real heat-up)
    Log or watch the trend screen from cold to setpoint:
      * Overshoot more than ~0.5 K   -> lower KP_W_PER_K (e.g. 8 -> 5).
      * Slow oscillation, period of a few minutes, that does not die out
                                     -> lower KP_W_PER_K and raise TI_S.
      * Settles below setpoint for a long time -> lower TI_S a little.
    Then dose and watch the dip: a dip that the PI has to fill in means
    FF_GAIN can go up; a bump above the setpoint means it should go down.

BENCH TEST
    python3 heater.py
prints the sensor reading and the controller state once a second (heater
off unless you pass --heat).
"""

import atexit
import glob
import json
import os
import threading
import time
import traceback
from collections import deque
from dataclasses import dataclass
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
HEATER_MAX_W = 50.0             # rated power at 24 V (power goes with V^2)
WINDOW_S = 2.0                  # time-proportioning window
MIN_SWITCH_S = 0.05             # shorter on/off slivers are not switched

# -- PI ---------------------------------------------------------------------
KP_W_PER_K = 8.0
TI_S = 1200.0
INTEGRAL_BAND_K = 1.5
# Estimated steady loss per kelvin above the room, used to preload the
# integrator. HX1 design note: about 2.3 W at 60 C in a 20 C room.
LOSS_W_PER_K = 0.06

# -- feed-forward -----------------------------------------------------------
INLET_TEMP_C = 21.0             # syringe, valve and pouch are at room temp
# Volumetric heat capacity of the dosed liquid. About 4.0 J/(ml K) for 1 M
# KOH (density ~1.05 g/ml, cp ~3.8-3.9 J/(g K)); water is 4.18.
LIQUID_J_PER_ML_K = 4.0
FF_GAIN = 0.9
FF_SPREAD_S = 30.0

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
        self._ff_j = 0.0
        self._ff_rate_w = 0.0
        self._band_since = None
        self._noheat = None             # (start time, start temperature)
        self.state = OFF if not self.enabled else HEATING
        self.fault = None
        self.power_w = 0.0
        self.in_band_s = 0.0

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
        """Queue the energy for one dose. Returns the energy in J (0 if the
        heater is off or faulted)."""
        if not self.enabled or self.latched or self.state == FAULT:
            return 0.0
        dt_k = max(0.0, self.setpoint_c - INLET_TEMP_C)
        energy = FF_GAIN * (volume_ul / 1000.0) * LIQUID_J_PER_ML_K * dt_k
        self._ff_j += energy
        # Everything still queued is spread over the next FF_SPREAD_S.
        self._ff_rate_w = self._ff_j / FF_SPREAD_S
        return energy

    @property
    def ff_pending_j(self):
        return self._ff_j

    # -- the update ---------------------------------------------------------

    def update(self, now, temp_c, dt):
        if not self.enabled:
            self.state, self.fault = OFF, None
            self.power_w = 0.0
            self._ff_j = 0.0
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

        ff_w = 0.0
        if self._ff_j > 0.0:
            ff_w = max(0.0, min(self._ff_rate_w, HEATER_MAX_W - pi_w,
                                self._ff_j / dt))
            self._ff_j -= ff_w * dt
            if self._ff_j < 1e-6:
                self._ff_j = 0.0

        # No-heat check: the PI has been asking for most of full power for a
        # long time and the block has hardly warmed. Only while no dose is
        # being compensated, so a cold dose cannot be mistaken for it.
        if pi_w >= NO_HEAT_POWER_FRAC * HEATER_MAX_W and self._ff_j == 0.0:
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
        self.power_w = pi_w + ff_w
        return self.power_w

    def _faulted(self, fault):
        self.state, self.fault = FAULT, fault
        self.power_w = 0.0
        self._ff_j = 0.0
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

    H: the inlet half with the heater; O: the outlet half with the probe.
    A dose's cold liquid lands in the inlet turns, so most of its heat is
    drawn from H. The probe has its own first-order lag and reads in
    0.0625 K steps. The numbers are estimates for the HX1 block (1.4 kg Al,
    3.1 m of 6x1 steel tube, PIR insulation), not measurements."""

    C_H = 800.0             # J/K
    C_O = 830.0             # J/K
    G_HO = 25.0             # W/K between the halves
    G_AMB = 0.03            # W/K from each half to the room
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
        self.heater_ok = True   # False simulates a tripped cutout

    def add_dose(self, volume_ul):
        energy = (volume_ul / 1000.0) * LIQUID_J_PER_ML_K * (self.t_h - INLET_TEMP_C)
        if energy > 0:
            self._sinks.append([energy, energy / self.DOSE_SPREAD_S])

    def step(self, dt, heater_w):
        heater_w = heater_w if self.heater_ok else 0.0
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
    ff_pending_j: float
    history_version: int


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

        self._last_heartbeat = None
        self._history = deque(maxlen=int(HISTORY_S / HISTORY_STEP_S))
        self._history_version = 0
        self._next_history = 0.0

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
        """Report liquid pushed through the target line. Returns the
        feed-forward energy queued, in J."""
        with self._lock:
            energy = self._ctl.add_dose(volume_ul)
            if self._sim is not None:
                self._sim.add_dose(volume_ul)
        if self.verbose and energy:
            print(f"  [heater] dose {volume_ul:.0f} ul -> feed-forward {energy:.0f} J")
        return energy

    def _save(self, message):
        with self._lock:
            data = {"setpoint_c": self._ctl.setpoint_c,
                    "heater_on": self._ctl.enabled}
        try:
            save_settings(data)
            return message
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
                history_version=self._history_version)

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
        if all(b == "00" for b in lines[0].split(":")[0].split()):
            # A data line held low reads as all zeros, whose CRC is also zero.
            self.sensor_note = "all-zero data (DQ shorted to GND?)"
            return None
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


if __name__ == "__main__":
    import sys
    bench_test(heat="--heat" in sys.argv)
