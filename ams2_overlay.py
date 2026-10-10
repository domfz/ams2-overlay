#!/usr/bin/env python3
"""Minimal racing gauge overlay for Automobilista 2.

Reads AMS2's UDP telemetry (Project CARS 2 protocol) and draws rpm, gear,
speed, fuel, shift lights with flags, pedal and engine arcs, tyres, brakes,
damage and a proximity radar in a transparent overlay. On Wayland it uses
gtk4-layer-shell (if installed) so it sits above the fullscreen game.
Press and hold the panel to drag it; right click closes it.

In AMS2: Options > System > UDP Protocol Version = "Project CARS 2",
UDP Frequency = 1 (fastest). Then run:  python3 ams2_overlay.py
Preview without the game:            python3 ams2_overlay.py --demo
"""

import argparse
import ctypes.util
import json
import math
import os
import signal
import socket
import struct
import sys
import threading
import time

# gtk4-layer-shell must be loaded before libwayland-client, so re-exec once
# with LD_PRELOAD when the library is available.
_LS_LIB = ctypes.util.find_library("gtk4-layer-shell")
if _LS_LIB and "libgtk4-layer-shell" not in os.environ.get("LD_PRELOAD", ""):
    os.environ["LD_PRELOAD"] = (
        _LS_LIB if _LS_LIB.startswith("/") else f"/usr/lib/{_LS_LIB}"
    ) + (":" + os.environ["LD_PRELOAD"] if os.environ.get("LD_PRELOAD") else "")
    os.execv(sys.executable, [sys.executable] + sys.argv)

import cairo  # noqa: E402
import gi  # noqa: E402

gi.require_version("Gtk", "4.0")
from gi.repository import GLib, Gtk  # noqa: E402

try:
    gi.require_version("Gtk4LayerShell", "1.0")
    from gi.repository import Gtk4LayerShell as LayerShell  # noqa: E402
except (ValueError, ImportError):
    LayerShell = None


# ---------------------------------------------------------------- telemetry

class Telemetry:
    def __init__(self):
        self.rpm = 0
        self.max_rpm = 8000
        self.gear = 0
        self.speed = 0.0  # m/s
        self.throttle = 0.0  # 0..1
        self.brake = 0.0
        self.clutch = 0.0
        self.fuel = 0.0  # 0..1
        self.fuel_capacity = 0  # litres, 0 = unknown
        # geared top speed (m/s) learned in top gear, 0 = not yet known
        self.top_speed = 0.0
        self.car_key = None  # (max rpm, gear count) to notice a car change
        # per wheel FL, FR, RL, RR: surface temps (left, centre, right) in °C
        self.tyre_temp = [(0, 0, 0)] * 4
        self.tyre_hud_temp = [0] * 4  # the single per tyre value AMS2's HUD shows
        self.tyre_wear = [0.0] * 4  # 0 = new, 1 = gone
        self.brake_temp = [0] * 4  # disc temps in °C
        # damage, 0 = fine, 1 = destroyed
        self.brake_damage = [0.0] * 4
        self.susp_damage = [0.0] * 4
        self.aero_damage = 0.0
        self.engine_damage = 0.0
        self.water_temp = 0  # °C
        self.oil_temp = 0  # °C
        self.oil_pressure = 0  # kPa
        # radar: heading (yaw, rad) and world positions (x, y, z) in metres
        self.yaw = 0.0
        self.my_pos = None
        self.cars = []  # every other active car
        self.position = 0  # 0 = unknown
        self.num_cars = 0
        self.lap = 0
        self.total_laps = 0
        self.timed = False
        self.time_left = -1.0  # seconds, timed sessions only
        self.flag = 0  # F_* colour of the viewed car's highest flag
        self.flag_reason = 0
        self.race_state = 0  # 3 = finished
        self.last_packet = 0.0
        self.lock = threading.Lock()


# pCars2 UDP sTelemetryData (packet type 0), little endian. Offsets:
#  10 mPacketType  12 sViewedParticipantIndex(b)  18 sOilTempCelsius(h)
#  20 sOilPressureKPa(H)  22 sWaterTempCelsius(h)  28 sFuelCapacity(B, litres)
#  29 sBrake  30 sThrottle  31 sClutch  32 sFuelLevel(f)  36 sSpeed(f, m/s)
#  40 sRpm(H)  42 sMaxRpm(H)  45 sGearNumGears(B, low nibble gear, high count)
#  52 sOrientation[3](f)
_TELEM = struct.Struct("<BBBffHHbB")  # unpacked from offset 29
# Wheel arrays (FL, FR, RL, RR): 176 sTyreTemp[4](B, °C)  196 sTyreWear[4](B)
#  200 sBrakeDamage[4](B)  204 sSuspensionDamage[4](B)  208 sBrakeTempCelsius[4](h)
#  256/264/272 sTyreTempLeft/Center/Right[4](H, °C)  371 sAeroDamage(B)
#  372 sEngineDamage(B)
_TYRE_TEMP = struct.Struct("<4B")
_TYRE_WEAR = struct.Struct("<4B")
_BRAKE_TEMP = struct.Struct("<4h")
_TYRE_SURF = struct.Struct("<12H")

# sRaceData (type 1): 304 sLapsTimeInEvent(H), top bit set = timed session
# sTimingsData (type 3): 12 sNumParticipants(b)  17 sEventTimeRemaining(f)
#  33 sParticipants[32] of 32 bytes: +14 sRacePosition(B, top bit = active)
#  +0 sWorldPosition[3](h, metres)  +15 sSector(B, bits 6-7 / 4-5 = extra
#  quarter metres of x / z)
#  +16 sHighestFlag(B, low nibble colour, high nibble reason)
#  +20 sRaceState(B, low 3 bits)  +21 sCurrentLap(B)
#  1057 sLocalParticipantIndex(H)
_PART_BASE, _PART_SIZE = 33, 32


def _parse_race_def(tel, data):
    if len(data) < 306:
        return
    v = struct.unpack_from("<H", data, 304)[0]
    tel.timed = bool(v & 0x8000)
    tel.total_laps = 0 if tel.timed else v & 0x7FFF


def _parse_timings(tel, data, viewed):
    if len(data) < 1059:
        return
    n = data[12]
    tel.time_left = struct.unpack_from("<f", data, 17)[0]
    me = viewed if 0 <= viewed < 32 else struct.unpack_from("<H", data, 1057)[0]
    active = 0
    tel.cars, tel.my_pos = [], None
    for i in range(min(n, 32)):
        off = _PART_BASE + i * _PART_SIZE
        pos = data[off + 14]
        if pos & 0x80:
            active += 1
            x, y, z = struct.unpack_from("<3h", data, off)
            sec = data[off + 15]
            world = (x + (sec >> 6) / 4, y, z + ((sec >> 4) & 3) / 4)
            if i == me:
                tel.my_pos = world
            else:
                tel.cars.append(world)
        if i == me:
            tel.position = pos & 0x7F
            tel.lap = data[off + 21]
            tel.flag = data[off + 16] & 0x0F
            tel.flag_reason = data[off + 16] >> 4
            tel.race_state = data[off + 20] & 0x07
    tel.num_cars = active or n


def learn_top_speed(tel, speed, rpm, max_rpm, num_gears, clutch, brake):
    """Speed at the limiter in top gear: in a fixed gear speed / rpm is
    constant, so scale the current ratio up to max rpm. Call with tel locked."""
    key = (max_rpm, num_gears)
    if key != tel.car_key:
        tel.car_key, tel.top_speed = key, 0.0
    if not (max_rpm and num_gears and tel.gear == num_gears):
        return
    # skip clutch slip, brake lock-ups and low revs where the reading is noisy
    if clutch > 12 or brake > 12 or rpm < max_rpm * 0.5 or speed < 15:
        return
    est = speed / rpm * max_rpm
    if tel.top_speed <= 0:
        tel.top_speed = est
    else:
        tel.top_speed += (est - tel.top_speed) * 0.02  # smooth over a few seconds


def udp_listener(tel: Telemetry, port: int):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))
    viewed = -1
    while True:
        data, _ = sock.recvfrom(2048)
        if len(data) < 12:
            continue
        kind = data[10]
        if kind in (1, 3):
            with tel.lock:
                if kind == 1:
                    _parse_race_def(tel, data)
                else:
                    _parse_timings(tel, data, viewed)
            continue
        if kind != 0 or len(data) < 46:
            continue
        viewed = struct.unpack_from("<b", data, 12)[0]
        brake, throttle, clutch, fuel, speed, rpm, max_rpm, _steer, gears = (
            _TELEM.unpack_from(data, 29)
        )
        yaw = struct.unpack_from("<f", data, 56)[0]  # sOrientation[1] at 52 + 4
        gear = gears & 0x0F
        with tel.lock:
            tel.brake = brake / 255
            tel.throttle = throttle / 255
            tel.clutch = clutch / 255
            tel.fuel = max(0.0, min(1.0, fuel))
            tel.fuel_capacity = data[28]
            tel.oil_temp, tel.oil_pressure, tel.water_temp = struct.unpack_from(
                "<hHh", data, 18)
            tel.speed = speed
            tel.yaw = yaw
            tel.rpm = rpm
            tel.max_rpm = max_rpm or tel.max_rpm
            tel.gear = -1 if gear == 15 else gear
            learn_top_speed(tel, speed, rpm, max_rpm, gears >> 4, clutch, brake)
            if len(data) >= 280:
                tt = _TYRE_TEMP.unpack_from(data, 176)
                surf = _TYRE_SURF.unpack_from(data, 256)
                tel.tyre_temp = [
                    (surf[i], surf[4 + i], surf[8 + i]) if surf[4 + i] else (tt[i],) * 3
                    for i in range(4)
                ]
                tel.tyre_hud_temp = list(tt)
                tel.tyre_wear = [w / 255 for w in _TYRE_WEAR.unpack_from(data, 196)]
                tel.brake_temp = list(_BRAKE_TEMP.unpack_from(data, 208))
                tel.brake_damage = [v / 255 for v in data[200:204]]
                tel.susp_damage = [v / 255 for v in data[204:208]]
            if len(data) >= 373:
                tel.aero_damage, tel.engine_damage = data[371] / 255, data[372] / 255
            tel.last_packet = time.monotonic()


def demo_feeder(tel: Telemetry):
    gear, rpm, speed = 1, 1000.0, 0.0
    max_rpm = 8500
    brakes = [90.0] * 4
    shifted = 0.0
    while True:
        t = time.monotonic()
        braking = int(t / 6) % 3 == 2
        with tel.lock:
            if braking:
                tel.throttle, tel.brake = 0.0, 0.85
                rpm -= 120
                speed = max(0.0, speed - 0.6)
                if rpm < 4500 and gear > 1:
                    gear -= 1
                    rpm += 2200
            else:
                tel.throttle, tel.brake = 1.0, 0.0
                rpm += 95 / gear
                speed += 0.35 / gear
                if rpm > max_rpm * 0.97 and gear < 6:
                    gear += 1
                    rpm *= 0.68
                    shifted = t
            tel.clutch = max(0.0, 1 - (t - shifted) / 0.25)
            rpm = max(1000.0, min(rpm, max_rpm))
            tel.rpm, tel.max_rpm, tel.gear = int(rpm), max_rpm, gear
            tel.speed, tel.fuel, tel.fuel_capacity = speed, 0.62, 100
            heat = min(1.0, (t % 120) / 40)  # warm up over the first laps
            tel.tyre_temp = [
                tuple(
                    45 + heat * (38 + 6 * math.sin(t * 0.4 + w))
                    + 7 * (1 - abs(c - z))  # camber: inner edge runs hotter
                    + (14 * tel.brake if w < 2 else 0)
                    for c in (0, 1, 2)
                )
                for w, z in enumerate((2, 0, 2, 0))
            ]
            tel.tyre_hud_temp = [sum(tt) / 3 for tt in tel.tyre_temp]
            tel.tyre_wear = [0.18, 0.22, 0.10, 0.12]
            # discs heat under braking (fronts more) and cool in the airflow
            for w in range(4):
                gain = (11 if w < 2 else 7) * tel.brake * speed / 40
                brakes[w] += gain - (brakes[w] - 70) * 0.003 * (1 + speed / 30)
            tel.brake_temp = [int(b) for b in brakes]
            # alternate between a lap race and a timed race every 15 s
            tel.position, tel.num_cars = 3 + int(t / 9) % 3, 18
            tel.timed = int(t / 15) % 2 == 1
            tel.lap, tel.total_laps = 4 + int(t / 30) % 10, 0 if tel.timed else 15
            tel.time_left = 1500 - t % 1500 if tel.timed else -1
            # cycle through every flag
            ft = t % 70
            tel.flag, tel.flag_reason, tel.race_state = F_NONE, 0, 2
            for start, end, flag, reason in DEMO_FLAGS:
                if start <= ft < end:
                    tel.flag, tel.flag_reason = flag, reason
            if 52 <= ft < 60:  # final lap
                tel.timed, tel.lap, tel.total_laps = False, 15, 15
            if ft >= 60:
                tel.race_state = 3
            # a knock halfway through the cycle
            hit = 30 <= ft < 50
            tel.susp_damage = [0.35 if hit else 0.0, 0.0, 0.0, 0.0]
            tel.brake_damage = [0.6 if hit else 0.0, 0.0, 0.0, 0.0]
            tel.aero_damage = 0.25 if hit else 0.0
            tel.engine_damage = 0.9 if 40 <= ft < 50 else 0.0
            # engine warms up with the tyres and overheats after the knock
            tel.water_temp = 30 + heat * 62 + (22 if 40 <= ft < 50 else 0)
            tel.oil_temp = 40 + heat * 65
            tel.oil_pressure = 150 + 450 * rpm / max_rpm
            # a car overtaking on the left, then one on the right
            tel.yaw, tel.my_pos = 0.0, (0.0, 0.0, 0.0)
            rt = t % 20
            if rt < 12:
                side = 2.6 if rt < 6 else -2.2  # +x is left at yaw 0
                tel.cars = [(side, 0.0, 9 - (rt % 6) * 3)]
            else:
                tel.cars = []
            tel.last_packet = t
        time.sleep(1 / 60)


# ---------------------------------------------------------------- flags

(F_NONE, F_GREEN, F_BLUE, F_WHITE_SLOW, F_WHITE_FINAL, F_RED, F_YELLOW,
 F_DYELLOW, F_BW, F_MEATBALL, F_BLACK, F_CHEQUERED) = range(12)

FLAG_PRIORITY = {
    F_NONE: 0, F_GREEN: 1, F_WHITE_SLOW: 2, F_WHITE_FINAL: 2, F_BLUE: 3,
    F_BW: 4, F_YELLOW: 5, F_DYELLOW: 6, F_MEATBALL: 7, F_BLACK: 8, F_RED: 9,
    F_CHEQUERED: 10,
}
# shown for as long as they are active; others only for FLAG_TIME
STICKY_FLAGS = {F_BLUE, F_YELLOW, F_DYELLOW, F_WHITE_SLOW, F_BW, F_MEATBALL,
                F_BLACK, F_RED}
MIN_HOLD = 1.0  # s a flag stays shown before a lower-priority one replaces it
FLAG_TIME = 4.0
GREEN_TIME = 2.5  # "track clear" after a yellow ends

DEMO_FLAGS = (  # (start, end, flag, reason) within a 70 s cycle
    (5, 11, F_YELLOW, 2), (16, 22, F_BLUE, 0), (25, 30, F_DYELLOW, 1),
    (33, 37, F_WHITE_SLOW, 3), (39, 44, F_BW, 0), (46, 51, F_MEATBALL, 0),
)


class FlagTracker:
    """Debounces raw flags: priority order, minimum hold, derived green and
    final-lap flags."""

    def __init__(self):
        self.flag, self.reason, self.since = F_NONE, 0, 0.0
        self.green_until = 0.0
        self.final_lap_at = None

    def update(self, raw, reason, now, final_lap, finished):
        if final_lap and self.final_lap_at is None:
            self.final_lap_at = now
        elif not final_lap:
            self.final_lap_at = None

        target = raw
        if finished:
            target = F_CHEQUERED
        elif target == F_NONE and now < self.green_until:
            target = F_GREEN
        elif (target == F_NONE and self.final_lap_at is not None
              and now - self.final_lap_at < FLAG_TIME):
            target = F_WHITE_FINAL  # AMS2 may not send it, so derive it

        if target != self.flag and (
            FLAG_PRIORITY[target] > FLAG_PRIORITY[self.flag]
            or now - self.since >= MIN_HOLD
        ):
            if self.flag in (F_YELLOW, F_DYELLOW) and target == F_NONE:
                target, self.green_until = F_GREEN, now + GREEN_TIME
            self.flag, self.reason, self.since = target, reason, now
        return self.flag

    def active(self, now):
        return self.flag != F_NONE and (
            self.flag in STICKY_FLAGS or now - self.since < FLAG_TIME
        )


# ---------------------------------------------------------------- drawing

BG = (0.07, 0.08, 0.10, 0.78)
TRACK = (1, 1, 1, 0.08)
TEXT = (0.95, 0.96, 0.98, 1)
DIM = (0.62, 0.65, 0.72, 1)
GREEN = (0.20, 0.85, 0.45)
YELLOW = (1.00, 0.78, 0.15)
RED = (1.00, 0.24, 0.27)
BLUE = (0.25, 0.55, 1.00)
CYAN = (0.20, 0.80, 0.95)
SPEED_COL = (0.45, 0.78, 1.00)
WHITE = (0.96, 0.97, 1.00)
ORANGE = (1.00, 0.55, 0.10)

# tyre heat map, as offsets from the optimal temperature: a flat green
# window of +-8 °C (70-90 °C is the usual AMS2 target, past ~90 it overheats)
TYRE_STOPS = ((-30, BLUE), (-14, CYAN), (-8, GREEN), (8, GREEN), (14, YELLOW),
              (24, RED))
# brake discs in °C: blue when too cold to bite (AMS2's HUD turns them green
# at about 200 °C), plain steel through the working range, then glowing iron.
# The HUD wants the discs to just touch its yellow (~600 °C) in the heaviest
# zones; its orange (~750 °C) is fade.
BRAKE_STOPS = ((140, BLUE), (230, (0.36, 0.37, 0.40)), (450, (0.36, 0.37, 0.40)),
               (580, (0.62, 0.10, 0.07)), (680, (1.00, 0.30, 0.05)),
               (780, (1.00, 0.62, 0.15)), (950, (1.00, 0.93, 0.75)))

ARC_START = math.radians(135)
ARC_SWEEP = math.radians(270)


def rounded_rect(cr, x, y, w, h, r):
    cr.new_sub_path()
    cr.arc(x + w - r, y + r, r, -math.pi / 2, 0)
    cr.arc(x + w - r, y + h - r, r, 0, math.pi / 2)
    cr.arc(x + r, y + h - r, r, math.pi / 2, math.pi)
    cr.arc(x + r, y + r, r, math.pi, 3 * math.pi / 2)
    cr.close_path()


def text(cr, s, x, y, size, color, weight=cairo.FONT_WEIGHT_BOLD, align="center"):
    cr.select_font_face("Inter", cairo.FONT_SLANT_NORMAL, weight)
    cr.set_font_size(size)
    ext = cr.text_extents(s)
    if align == "center":
        x -= ext.x_advance / 2
    elif align == "right":
        x -= ext.x_advance
    cr.move_to(x, y)
    cr.set_source_rgba(*color)
    cr.show_text(s)


def ramp(stops, v):
    if v <= stops[0][0]:
        return stops[0][1]
    for (v0, c0), (v1, c1) in zip(stops, stops[1:]):
        if v <= v1:
            k = (v - v0) / (v1 - v0)
            return tuple(a + (b - a) * k for a, b in zip(c0, c1))
    return stops[-1][1]


def tyre_color(temp, optimal):
    return ramp(TYRE_STOPS, temp - optimal)


DAMAGE_MIN = 0.04  # ignore the small values AMS2 reports for an undamaged car


# engine arcs: (arc start, arc end, cold below, warm from, too hot from) in °C
WATER_RANGE = (40, 130, 60, 100, 110)
OIL_RANGE = (40, 150, 70, 125, 135)
OIL_PRESSURE_MAX = 700  # kPa at the end of the arc
OIL_PRESSURE_LOW = 100  # kPa, alarming once the engine is revving


def engine_color(v, rng):
    """(colour, alpha): faint while normal, blue while cold, orange to red
    when running hot."""
    _, _, cold, warm, hot = rng
    if v < cold:
        return BLUE, 0.8
    if v < warm:
        return DIM, 0.55
    return ramp(((warm, ORANGE), (hot, RED)), v), 1.0


def damage_color(v):
    return ramp(((DAMAGE_MIN, YELLOW), (0.4, ORANGE), (0.8, RED)), v)


# radar: cars whose side overlaps yours, in metres
RADAR_LONG = 5.5  # max front / rear offset (about a car length plus a bit)
RADAR_LAT = (1.4, 4.5)  # beside you: closer than 1.4 m is in line behind or ahead
RADAR_MIN_SPEED = 3.0  # m/s, quiet on the grid and in the pit box
# a car this close is side by side with you: red border, don't turn in
RADAR_OVERLAP = (4.5, 3.5)  # max front / rear and sideways offset


def radar_contacts(tel):
    """(side, longitudinal, lateral) of cars alongside: side -1 left / +1
    right, longitudinal + ahead. Call with tel locked."""
    if tel.my_pos is None or tel.speed < RADAR_MIN_SPEED:
        return ()
    mx, my, mz = tel.my_pos
    sy, cy = math.sin(tel.yaw), math.cos(tel.yaw)
    out = []
    for x, y, z in tel.cars:
        dx, dz = x - mx, z - mz
        if abs(y - my) > 3 or abs(dx) > 10 or abs(dz) > 10:
            continue  # other level (bridge) or far away
        lng = -dx * sy - dz * cy  # forward is (-sin, -cos) yaw
        lat = -dx * cy + dz * sy  # right is (-cos, sin) yaw
        if abs(lng) < RADAR_LONG and RADAR_LAT[0] < abs(lat) < RADAR_LAT[1]:
            out.append((1 if lat > 0 else -1, round(lng, 1), round(abs(lat), 1)))
    return tuple(out)


def draw_brake(cr, x, y, temp, r=8, caliper=0.0, damage=0.0):
    """Brake disc icon seen face on, glowing like hot iron as it heats up.
    caliper is the angle (radians) where the caliper sits on the rim; it
    takes the damage colour when the brake is damaged."""
    col = ramp(BRAKE_STOPS, temp)
    heat = max(0.0, min(1.0, (temp - 540) / 360))
    if heat > 0:  # heat haze, growing as the disc starts to glow
        hr = r + 4 + 8 * heat
        haze = cairo.RadialGradient(x, y, r * 0.6, x, y, hr)
        haze.add_color_stop_rgba(0, *col, 0.55 * heat)
        haze.add_color_stop_rgba(1, *col, 0)
        cr.arc(x, y, hr, 0, 2 * math.pi)
        cr.set_source(haze)
        cr.fill()
    # disc face, hottest at the rim where the pads bite
    face = cairo.RadialGradient(x, y, 0, x, y, r)
    face.add_color_stop_rgba(0, *(c * 0.55 for c in col), 1)
    face.add_color_stop_rgba(1, *col, 1)
    cr.arc(x, y, r, 0, 2 * math.pi)
    cr.set_source(face)
    cr.fill()
    # hub and cross-drilled holes
    cr.set_source_rgba(0.07, 0.08, 0.10, 0.85)
    cr.arc(x, y, r * 0.32, 0, 2 * math.pi)
    cr.fill()
    for k in range(6):
        a = k * math.pi / 3 + math.pi / 6
        cr.arc(x + math.cos(a) * r * 0.66, y + math.sin(a) * r * 0.66, 0.9, 0, 2 * math.pi)
        cr.fill()
    # caliper clamping the rim
    cr.set_line_cap(cairo.LINE_CAP_ROUND)
    cr.set_line_width(3.5)
    if damage > DAMAGE_MIN:
        cr.set_source_rgba(*damage_color(damage), 1)
    else:
        cr.set_source_rgba(0.16, 0.17, 0.20, 1)
    cr.arc(x, y, r - 1, caliper - 0.55, caliper + 0.55)
    cr.stroke()
    cr.set_line_width(1)
    cr.set_source_rgba(1, 1, 1, 0.3)
    cr.arc(x, y, r + 0.9, caliper - 0.6, caliper + 0.6)
    cr.stroke()


# AMS2's HUD shows a tyre as finished at about half the raw wear value
WEAR_LIMIT = 0.5


def draw_tyre(cr, x, y, w, h, temps, label_temp, wear, optimal):
    cols = [tyre_color(t, optimal) for t in temps]
    glow = tyre_color(sum(temps) / 3, optimal)

    # empty carcass outline at full size
    rounded_rect(cr, x + 0.5, y + 0.5, w - 1, h - 1, 8)
    cr.set_source_rgba(1, 1, 1, 0.05)
    cr.fill_preserve()
    cr.set_line_width(1)
    cr.set_source_rgba(1, 1, 1, 0.16)
    cr.stroke()

    # remaining rubber: full width, the level drops from the top as it wears
    ih = max(2.0, h * max(0.0, 1 - wear / WEAR_LIMIT))
    iw, ix, iy = w, x, y + h - ih
    ir = min(8.0, ih / 2)

    # soft glow
    for i in range(4, 0, -1):
        rounded_rect(cr, ix - i * 2, iy - i * 2, iw + i * 4, ih + i * 4, ir + i * 2)
        cr.set_source_rgba(*glow, 0.05)
        cr.fill()

    # heat map across the tread: left / centre / right blended
    cr.save()
    rounded_rect(cr, ix, iy, iw, ih, ir)
    cr.clip()
    grad = cairo.LinearGradient(ix, 0, ix + iw, 0)
    for stop, c in zip((0.0, 0.5, 1.0), cols):
        grad.add_color_stop_rgba(stop, *c, 0.92)
    cr.set_source(grad)
    cr.paint()
    # tread grooves
    cr.set_line_width(1.2)
    cr.set_source_rgba(0, 0, 0, 0.22)
    for gy in range(int(y) + 6, int(y + h), 7):
        cr.move_to(ix, gy)
        cr.line_to(ix + iw, gy)
    cr.stroke()
    # gloss
    gloss = cairo.LinearGradient(0, iy, 0, iy + ih)
    gloss.add_color_stop_rgba(0, 1, 1, 1, 0.28)
    gloss.add_color_stop_rgba(0.45, 1, 1, 1, 0.0)
    gloss.add_color_stop_rgba(1, 0, 0, 0, 0.25)
    cr.set_source(gloss)
    cr.paint()
    cr.restore()

    rounded_rect(cr, ix + 0.5, iy + 0.5, iw - 1, ih - 1, ir)
    cr.set_line_width(1)
    cr.set_source_rgba(1, 1, 1, 0.22)
    cr.stroke()

    text(cr, f"{label_temp:.0f}°", x + w / 2, y + h + 15, 13, TEXT)


def fmt_time(sec):
    if sec < 0:
        return "--:--"
    sec = int(sec)
    h, m, s = sec // 3600, sec // 60 % 60, sec % 60
    return f"{h}:{m:02}:{s:02}" if h else f"{m:02}:{s:02}"


def pill(cr, x, y, label, value, align, color=TEXT):
    """Small label/value capsule; x is the left or right edge depending on align."""
    cr.select_font_face("Inter", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_NORMAL)
    cr.set_font_size(9)
    lw_ = cr.text_extents(label).x_advance
    cr.select_font_face("Inter", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_BOLD)
    cr.set_font_size(15)
    vw = cr.text_extents(value).x_advance
    w, h = lw_ + vw + 26, 22
    if align == "right":
        x -= w
    rounded_rect(cr, x, y, w, h, h / 2)
    cr.set_source_rgba(1, 1, 1, 0.07)
    cr.fill()
    text(cr, label, x + 10, y + 15, 9, DIM, cairo.FONT_WEIGHT_NORMAL, "left")
    text(cr, value, x + 16 + lw_, y + 16.5, 15, color, align="left")


FLAG_LABEL = {  # accent colour, short label under the flag icon
    F_GREEN: (GREEN, "GREEN"),
    F_BLUE: (BLUE, "BLUE"),
    F_WHITE_SLOW: (WHITE, "SLOW CAR"),
    F_WHITE_FINAL: (WHITE, "FINAL LAP"),
    F_RED: (RED, "RED"),
    F_YELLOW: (YELLOW, "YELLOW"),
    F_DYELLOW: (YELLOW, "2× YELLOW"),
    F_BW: (WHITE, "WARNING"),
    F_MEATBALL: (ORANGE, "DAMAGE"),
    F_BLACK: (WHITE, "BLACK"),
    F_CHEQUERED: (WHITE, "FINISH"),
}


def paint_flag_cloth(cr, flag, w, h):
    """Flat flag design at the origin, w x h."""
    solid = {F_GREEN: GREEN, F_WHITE_SLOW: WHITE, F_WHITE_FINAL: WHITE,
             F_RED: RED, F_YELLOW: YELLOW, F_DYELLOW: YELLOW}
    if flag in solid:
        cr.set_source_rgb(*solid[flag])
        cr.paint()
    elif flag == F_BLUE:
        cr.set_source_rgb(*BLUE)
        cr.paint()
        cr.set_source_rgb(*YELLOW)
        cr.set_line_width(h * 0.22)
        cr.move_to(0, h)
        cr.line_to(w, 0)
        cr.stroke()
    elif flag == F_BW:
        cr.set_source_rgb(*WHITE)
        cr.paint()
        cr.set_source_rgb(0.05, 0.05, 0.06)
        cr.move_to(0, 0)
        cr.line_to(w, 0)
        cr.line_to(0, h)
        cr.close_path()
        cr.fill()
    elif flag in (F_MEATBALL, F_BLACK):
        cr.set_source_rgb(0.05, 0.05, 0.06)
        cr.paint()
        if flag == F_MEATBALL:
            cr.set_source_rgb(*ORANGE)
            cr.arc(w / 2, h / 2, h * 0.3, 0, 2 * math.pi)
            cr.fill()
        cr.set_source_rgba(1, 1, 1, 0.6)
        cr.set_line_width(1.5)
        cr.rectangle(0.75, 0.75, w - 1.5, h - 1.5)
        cr.stroke()
    elif flag == F_CHEQUERED:
        cols, rows = 5, 4
        for r in range(rows):
            for c in range(cols):
                cr.set_source_rgb(*(WHITE if (r + c) % 2 else (0.05, 0.05, 0.06)))
                cr.rectangle(c * w / cols, r * h / rows, w / cols + 0.5, h / rows + 0.5)
                cr.fill()


FUEL_LOW, FUEL_WARN = 0.07, 0.15  # red / yellow fuel thresholds


def draw_fuel_pump(cr, x, y, col):
    """Fuel pump symbol centred on x, standing on baseline y (gear sized)."""
    bx, top = x - 15, y - 40
    cr.set_source_rgba(*col, 1)
    rounded_rect(cr, bx, top, 22, 38, 4)  # pump body
    cr.fill()
    rounded_rect(cr, bx - 3, y - 4, 28, 4, 2)  # base plate
    cr.fill()
    cr.set_source_rgba(*BG[:3], 1)
    rounded_rect(cr, bx + 4, top + 5, 14, 10, 2)  # display window
    cr.fill()
    # hose out of the side, down to the nozzle
    cr.set_source_rgba(*col, 1)
    cr.set_line_width(3)
    cr.set_line_cap(cairo.LINE_CAP_ROUND)
    cr.set_line_join(cairo.LINE_JOIN_ROUND)
    cr.move_to(bx + 22, top + 9)
    cr.line_to(bx + 27, top + 14)
    cr.line_to(bx + 27, y - 12)
    cr.curve_to(bx + 27, y - 7, bx + 32, y - 7, bx + 32, y - 12)
    cr.line_to(bx + 32, top + 8)
    cr.line_to(bx + 28, top + 3)
    cr.stroke()


_CLOTH_CACHE = {}


def draw_waving_flag(cr, flag, x, y, w, h, now, res=3):
    """Flag on a pole, rippling in the wind."""
    key = (flag, w, h, res)
    cloth = _CLOTH_CACHE.get(key)
    if cloth is None:
        cloth = cairo.ImageSurface(cairo.FORMAT_ARGB32, int(w * res), int(h * res))
        c2 = cairo.Context(cloth)
        c2.scale(res, res)
        paint_flag_cloth(c2, flag, w, h)
        _CLOTH_CACHE[key] = cloth

    cr.set_source_rgba(*DIM)
    cr.set_line_width(1.6)
    cr.move_to(x - 1, y - 3)
    cr.line_to(x - 1, y + h + 2)
    cr.stroke()

    step = 1.0
    sx = 0.0
    while sx < w:
        k = sx / w  # pinned at the pole, free at the tip
        phase = now * 7 - sx * 0.32
        dy = math.sin(phase) * 2.2 * k
        cr.save()
        cr.rectangle(x + sx, y + dy - 1, step + 0.35, h + 2)
        cr.clip()
        cr.translate(x, y + dy)
        cr.scale(1 / res, 1 / res)
        cr.set_source_surface(cloth, 0, 0)
        cr.paint()
        cr.restore()
        # fold shading
        shade = (math.cos(phase) * 0.5 + 0.5) * 0.28 * k
        cr.rectangle(x + sx, y + dy, step + 0.35, h)
        cr.set_source_rgba(0, 0, 0, shade)
        cr.fill()
        sx += step


def flag_leds(flag, now, n):
    """Per-LED (colour, alpha) for the shift lights while a flag is shown."""
    tick = lambda hz: int(now * hz) % 2 == 0  # noqa: E731
    if flag == F_YELLOW:
        return [(YELLOW, 1.0)] * n
    if flag == F_DYELLOW:
        a = tick(4)
        return [(YELLOW, 1.0 if (i < n // 2) == a else 0.12) for i in range(n)]
    if flag == F_BLUE:  # comet sweeping side to side
        head = (math.sin(now * 4) + 1) / 2 * (n - 1)
        return [(BLUE, max(0.12, 1 - abs(i - head) / 2.5)) for i in range(n)]
    if flag in (F_WHITE_SLOW, F_WHITE_FINAL):
        return [(WHITE, 1.0)] * n
    if flag == F_RED:
        return [(RED, 1.0 if tick(2) else 0.15)] * n
    if flag == F_GREEN:
        return [(GREEN, 1.0 if tick(3) else 0.25)] * n
    if flag == F_BW:
        a = tick(2)
        return [(WHITE, 1.0 if (i < n // 2) == a else 0.08) for i in range(n)]
    if flag == F_MEATBALL:
        pulse = 0.55 + 0.45 * math.sin(now * 5)
        return [(ORANGE, pulse if 3 <= i < n - 3 else 0.08) for i in range(n)]
    if flag == F_BLACK:
        a = tick(1.5)
        return [(WHITE, 0.9 if (i % 2 == 0) == a else 0.08) for i in range(n)]
    if flag == F_CHEQUERED:
        shift = int(now * 6)
        return [(WHITE, 1.0 if (i + shift) % 2 else 0.08) for i in range(n)]
    return None


def rpm_color(frac):
    if frac < 0.80:
        return GREEN
    if frac < 0.92:
        return YELLOW
    return RED


class Gauge(Gtk.DrawingArea):
    W, PANEL_H = 388, 230
    H = PANEL_H
    ARC = (120, 132, 82)  # rpm arc centre x, y and radius
    SPEED_R = 69  # radius of the inner speed arc
    TYRE_X = (256, 332)  # left and right tyre columns
    # clutch, brake and throttle arcs outside the rpm arc: radii, and they fill
    # anticlockwise from the bottom right end of the rpm arc up to the top
    PEDAL_R = (92, 96.5, 101)
    PEDAL_START = ARC_START + ARC_SWEEP
    PEDAL_SWEEP = math.radians(135)
    # water temp, oil temp and oil pressure mirror them on the left, filling
    # clockwise from the bottom left end up to the top
    ENGINE_START = ARC_START

    def __init__(self, tel: Telemetry, scale: float, units: str, tyre_opt: float,
                 fps: int = 30, vmax: float = 0):
        super().__init__()
        self.tel, self.scale, self.units = tel, scale, units
        self.tyre_opt = tyre_opt
        # end of the speed arc until a top speed has been learned in top gear
        self.vmax_fallback = vmax or (320 if units == "kmh" else 200)
        self.shown_vmax = self.vmax_fallback
        self.shown_rpm = 0.0
        self.last_draw = time.monotonic()
        self.static = None  # (key, surface) of the parts that never change
        self.last_sig = None
        self.animating = True
        self.flags = FlagTracker()
        self.drag_armed = False  # outline while held for moving
        self.drag_offset = None  # panel position inside the monitor while dragging
        self.set_content_width(int(self.W * scale))
        self.set_content_height(int(self.H * scale))
        self.set_draw_func(self.draw)
        # Redraw at a capped rate and only when something visible changed, so
        # the compositor isn't forced to repaint over the game every refresh.
        GLib.timeout_add(max(1, round(1000 / fps)), self.on_timer)

    def on_timer(self):
        with self.tel.lock:
            t = self.tel
            sig = (
                t.rpm // 20, t.max_rpm, t.gear, int(t.speed * 10),
                int(t.throttle * 50), int(t.brake * 50), int(t.clutch * 50),
                int(t.fuel * 200),
                tuple(tuple(int(x) for x in w) for w in t.tyre_temp),
                tuple(int(x) for x in t.tyre_hud_temp),
                tuple(int(w * 100) for w in t.tyre_wear),
                tuple(b // 5 for b in t.brake_temp),
                tuple(int(v * 50) for v in t.brake_damage + t.susp_damage),
                int(t.aero_damage * 50), int(t.engine_damage * 50),
                int(t.water_temp), int(t.oil_temp), int(t.oil_pressure) // 10,
                radar_contacts(t),
                t.position, t.num_cars, t.lap, t.total_laps, t.timed,
                int(t.time_left), t.flag, t.flag_reason, t.race_state,
                int(t.top_speed),
                time.monotonic() - t.last_packet < 2.0,
            )
        if sig != self.last_sig or self.animating:
            self.last_sig = sig
            self.queue_draw()
        return True

    def static_layer(self, max_rpm):
        """Panel background, scale, labels and bar tracks, cached per car."""
        key = (max_rpm, self.units, self.scale)
        if self.static and self.static[0] == key:
            return self.static[1]
        surf = cairo.ImageSurface(cairo.FORMAT_ARGB32, math.ceil(self.W * self.scale),
                                  math.ceil(self.PANEL_H * self.scale))
        cr = cairo.Context(surf)
        cr.scale(self.scale, self.scale)

        rounded_rect(cr, 0, 0, self.W, self.PANEL_H, 22)
        cr.set_source_rgba(*BG)
        cr.fill()

        cx, cy, r = self.ARC
        cr.set_line_cap(cairo.LINE_CAP_ROUND)
        cr.set_line_width(12)
        cr.set_source_rgba(*TRACK)
        cr.arc(cx, cy, r, ARC_START, ARC_START + ARC_SWEEP)
        cr.stroke()
        # pedal arc tracks
        cr.set_line_width(3)
        cr.set_source_rgba(*TRACK)
        for pr in self.PEDAL_R:
            cr.new_path()
            cr.arc_negative(cx, cy, pr, self.PEDAL_START,
                            self.PEDAL_START - self.PEDAL_SWEEP)
            cr.stroke()
            cr.new_path()
            cr.arc(cx, cy, pr, self.ENGINE_START, self.ENGINE_START + self.PEDAL_SWEEP)
            cr.stroke()
        # tick labels (thousands)
        for k in range(max_rpm // 1000 + 1):
            a = ARC_START + ARC_SWEEP * (k * 1000 / max_rpm)
            tx, ty = cx + math.cos(a) * (r - 24), cy + math.sin(a) * (r - 24)
            text(cr, str(k), tx, ty + 4, 10, DIM, cairo.FONT_WEIGHT_NORMAL)
        # inner speed arc track, speed unit under the speed readout
        cr.new_path()
        cr.set_line_cap(cairo.LINE_CAP_ROUND)
        cr.set_line_width(4)
        cr.set_source_rgba(*TRACK)
        cr.arc(cx, cy, self.SPEED_R, ARC_START, ARC_START + ARC_SWEEP)
        cr.stroke()
        text(cr, "KM/H" if self.units == "kmh" else "MPH", cx, cy + 51, 8, DIM,
             cairo.FONT_WEIGHT_NORMAL)

        # divider and faint chassis between the wheels
        lx, rx = self.TYRE_X
        mid = (lx + 34 + rx) / 2
        cr.set_source_rgba(*TRACK)
        cr.set_line_width(1)
        cr.move_to(lx - 21.5, 40)
        cr.line_to(lx - 21.5, 205)
        cr.stroke()
        rounded_rect(cr, mid - 13, 60, 26, 120, 10)
        cr.set_source_rgba(1, 1, 1, 0.05)
        cr.fill()
        cr.set_line_width(2)
        cr.set_source_rgba(1, 1, 1, 0.07)
        for ay in (78, 160):
            cr.move_to(lx + 34, ay)
            cr.line_to(rx, ay)
        cr.stroke()

        self.static = (key, surf)
        return surf

    RADAR_Y = (40, 205)  # top and bottom of the side glow strips

    def draw_radar(self, cr, contacts):
        """Glow on the panel edge for each car alongside, higher for one by
        your front, lower for one by your rear, redder the closer it is."""
        if any(abs(lng) < RADAR_OVERLAP[0] and lat < RADAR_OVERLAP[1]
               for _, lng, lat in contacts):
            rounded_rect(cr, 1.5, 1.5, self.W - 3, self.PANEL_H - 3, 20.5)
            cr.set_line_width(3)
            cr.set_source_rgba(*RED, 0.9)
            cr.stroke()
        top, bottom = self.RADAR_Y
        mid, half = (top + bottom) / 2, (bottom - top) / 2
        for side, lng, lat in contacts:
            near = max(0.0, min(1.0, 1 - (lat - 2.0) / (RADAR_LAT[1] - 2.0)))
            along = max(0.0, min(1.0, 1 - (abs(lng) - 2.5) / (RADAR_LONG - 2.5)))
            k = near * along
            if k <= 0.02:
                continue
            col = ramp(((0.0, ORANGE), (1.0, RED)), near)
            gy = mid - lng / RADAR_LONG * half  # ahead = higher up
            ex = 0 if side < 0 else self.W
            inward = 1 if side < 0 else -1
            # soft wash into the panel
            cr.save()
            rounded_rect(cr, 0, top - 10, self.W, bottom - top + 20, 0)
            cr.clip()
            cr.scale(1, 1.6)  # stretch the glow along the car's length
            wash = cairo.RadialGradient(ex, gy / 1.6, 0, ex, gy / 1.6, 40)
            wash.add_color_stop_rgba(0, *col, 0.6 * k)
            wash.add_color_stop_rgba(1, *col, 0)
            cr.set_source(wash)
            cr.paint()
            cr.restore()
            # bright strip on the edge, about a car length tall
            h = 34
            y0 = max(top, min(bottom - h, gy - h / 2))
            rounded_rect(cr, ex + inward * 5 - (4 if side > 0 else 0), y0, 4, h, 2)
            cr.set_source_rgba(*col, 0.35 + 0.65 * k)
            cr.fill()

    def draw_damage(self, cr, inner_l, inner_r, susp, aero, engine):
        """Damage on the chassis between the wheels, drawn only once there is
        some: suspension struts, front and rear wings, engine block."""
        mid = (inner_l + inner_r) / 2
        cr.set_line_cap(cairo.LINE_CAP_ROUND)
        cr.set_line_width(3)
        for i, ay in enumerate((78, 78, 160, 160)):
            if susp[i] > DAMAGE_MIN:
                edge = inner_l + 8 if i % 2 == 0 else inner_r - 8  # disc rim
                end = mid - 4 if i % 2 == 0 else mid + 4
                cr.set_source_rgba(*damage_color(susp[i]), 1)
                cr.move_to(edge, ay)
                cr.line_to(end, ay)
                cr.stroke()
        if aero > DAMAGE_MIN:
            cr.set_source_rgba(*damage_color(aero), 1)
            for wy in (55, 185):
                rounded_rect(cr, mid - 16, wy - 2, 32, 4, 2)
                cr.fill()
        if engine > DAMAGE_MIN:
            col = damage_color(engine)
            rounded_rect(cr, mid - 9, 111, 18, 13, 3)  # glow
            cr.set_source_rgba(*col, 0.25)
            cr.set_line_width(4)
            cr.stroke()
            rounded_rect(cr, mid - 7, 113, 14, 9, 2)  # block
            cr.set_source_rgba(*col, 1)
            cr.fill()
            for k in (-4, 0, 4):  # cylinders on top
                rounded_rect(cr, mid + k - 1.5, 109, 3, 4, 1)
                cr.fill()

    def draw(self, _area, cr, w, _h):
        cr.set_operator(cairo.OPERATOR_SOURCE)
        cr.set_source_rgba(0, 0, 0, 0)
        cr.paint()
        cr.set_operator(cairo.OPERATOR_OVER)
        if self.drag_offset and w > self.W * self.scale * 1.5:
            cr.translate(*self.drag_offset)  # inside the monitor sized drag surface
        cr.scale(self.scale, self.scale)

        with self.tel.lock:
            t = self.tel
            rpm, max_rpm, gear = t.rpm, max(t.max_rpm, 1), t.gear
            speed, thr, brk, clu, fuel = t.speed, t.throttle, t.brake, t.clutch, t.fuel
            fuel_cap = t.fuel_capacity
            top_speed = t.top_speed
            tyre_temp, tyre_wear = list(t.tyre_temp), list(t.tyre_wear)
            tyre_hud_temp = list(t.tyre_hud_temp)
            brake_temp = list(t.brake_temp)
            brake_dmg, susp_dmg = list(t.brake_damage), list(t.susp_damage)
            aero_dmg, engine_dmg = t.aero_damage, t.engine_damage
            water, oil, oil_p = t.water_temp, t.oil_temp, t.oil_pressure
            radar = radar_contacts(t)
            pos, cars, lap, laps = t.position, t.num_cars, t.lap, t.total_laps
            timed, time_left = t.timed or (laps == 0 and t.time_left > 0), t.time_left
            raw_flag, flag_reason = t.flag, t.flag_reason
            finished = t.race_state == 3
            live = time.monotonic() - t.last_packet < 2.0

        # smooth the needle a little (frame-rate independent)
        now = time.monotonic()
        dt, self.last_draw = now - self.last_draw, now
        self.shown_rpm += (rpm - self.shown_rpm) * (1 - math.exp(-dt / 0.035))
        frac = max(0.0, min(1.0, self.shown_rpm / max_rpm))

        final_lap = not timed and laps > 0 and lap == laps
        flag = self.flags.update(raw_flag, flag_reason, now, final_lap, finished)

        # panel
        cr.save()
        cr.scale(1 / self.scale, 1 / self.scale)
        cr.set_source_surface(self.static_layer(max_rpm), 0, 0)
        cr.paint()
        cr.restore()

        # shift lights
        n, lit = 10, 0
        if frac > 0.75:
            lit = math.ceil((frac - 0.75) / 0.22 * n)
        flash = frac > 0.97 and int(now * 12) % 2 == 0
        # flags take over the lights unless you are about to hit the limiter
        leds = None
        flag_on = flag == F_CHEQUERED or self.flags.active(now)
        if frac < 0.9 and flag_on:
            leds = flag_leds(flag, now, n)
        lw, gap = 14, 3
        x0 = (self.W - (n * lw + (n - 1) * gap)) / 2
        for i in range(n):
            if leds:
                col, alpha = leds[i]
            else:
                col = BLUE if flash else GREEN if i < 4 else YELLOW if i < 7 else RED
                alpha = 1 if flash or i < lit else 0.14
            lx = x0 + i * (lw + gap)
            if alpha > 0.5:  # bloom
                rounded_rect(cr, lx - 2, 12, lw + 4, 11, 5.5)
                cr.set_source_rgba(*col, 0.18 * alpha)
                cr.fill()
            rounded_rect(cr, lx, 14, lw, 7, 3.5)
            cr.set_source_rgba(*col, alpha)
            cr.fill()

        # position and lap / remaining time pills either side of the lights
        if pos:
            pill(cr, 14, 8, "POS", f"{pos}/{cars}" if cars else str(pos), "left",
                 GREEN if pos == 1 else TEXT)
        if timed:
            pill(cr, self.W - 14, 8, "TIME", fmt_time(time_left), "right",
                 RED if 0 <= time_left < 60 else TEXT)
        elif laps or lap:
            lap_txt = f"{min(lap, laps)}/{laps}" if laps else str(lap)
            pill(cr, self.W - 14, 8, "LAP", lap_txt, "right",
                 YELLOW if laps and lap >= laps else TEXT)

        # rpm arc
        cx, cy, r = self.ARC
        if frac > 0.005:
            cr.new_path()
            cr.set_line_cap(cairo.LINE_CAP_ROUND)
            cr.set_line_width(12)
            cr.set_source_rgba(*rpm_color(frac), 1)
            cr.arc(cx, cy, r, ARC_START, ARC_START + ARC_SWEEP * frac)
            cr.stroke()

        # gear, with fuel underneath
        g = "R" if gear < 0 else "N" if gear == 0 else str(gear)
        # low fuel: the gear blinks with a fuel pump
        fuel_low = live and fuel < FUEL_LOW
        if fuel_low and int(now * 2) % 2:
            draw_fuel_pump(cr, cx, cy + 12, RED)
        else:
            text(cr, g, cx, cy + 12, 58, RED if frac > 0.97 else TEXT)

        # speed: inner arc plus readout under the gear
        conv = 3.6 if self.units == "kmh" else 2.23694
        v = speed * conv
        vmax = top_speed * conv if top_speed > 0 else self.vmax_fallback
        self.shown_vmax += (vmax - self.shown_vmax) * (1 - math.exp(-dt / 0.4))
        vfrac = max(0.0, min(1.0, v / self.shown_vmax))
        if vfrac > 0.005:
            cr.new_path()
            cr.set_line_cap(cairo.LINE_CAP_ROUND)
            cr.set_line_width(4)
            cr.set_source_rgba(*SPEED_COL, 1)
            cr.arc(cx, cy, self.SPEED_R, ARC_START, ARC_START + ARC_SWEEP * vfrac)
            cr.stroke()
        text(cr, f"{v:.0f}", cx, cy + 41, 18, TEXT)
        # fuel in the gap at the bottom of the arcs; a flag takes its place
        if flag_on:
            accent, label = FLAG_LABEL[flag]
            icons = 2 if flag == F_DYELLOW else 1
            fw, fh, gap = 24, 16, 6
            fx = cx - (icons * fw + (icons - 1) * gap) / 2
            for k in range(icons):
                draw_waving_flag(cr, flag, fx + k * (fw + gap), cy + 56, fw, fh,
                                 now + k * 0.6)
            text(cr, label, cx, cy + 84, 9, accent)
        else:
            rounded_rect(cr, cx - 28, cy + 61, 56, 5, 2.5)
            cr.set_source_rgba(*TRACK)
            cr.fill()
            fuel_col = RED if fuel < FUEL_LOW else YELLOW if fuel < FUEL_WARN else BLUE
            if fuel > 0.005:
                rounded_rect(cr, cx - 28, cy + 61, max(5, 56 * fuel), 5, 2.5)
                cr.set_source_rgba(*fuel_col, 1)
                cr.fill()
            fuel_txt = f"{fuel * fuel_cap:.1f} L" if fuel_cap else f"{fuel * 100:.0f}%"
            text(cr, fuel_txt, cx, cy + 79, 10, fuel_col if fuel < FUEL_WARN else DIM,
                 cairo.FONT_WEIGHT_NORMAL)

        # clutch / brake / throttle fill their arcs anticlockwise
        cr.set_line_cap(cairo.LINE_CAP_ROUND)
        for pr, val, col in zip(self.PEDAL_R, (clu, brk, thr), (CYAN, RED, GREEN)):
            if val > 0.01:
                cr.new_path()
                cr.set_line_width(3)
                cr.set_source_rgba(*col, 1)
                cr.arc_negative(cx, cy, pr, self.PEDAL_START,
                                self.PEDAL_START - self.PEDAL_SWEEP * val)
                cr.stroke()

        # water temp / oil temp / oil pressure fill theirs clockwise
        revving = rpm > max_rpm * 0.3
        p_low = revving and oil_p < OIL_PRESSURE_LOW
        engine = (
            ((water - WATER_RANGE[0]) / (WATER_RANGE[1] - WATER_RANGE[0]),
             engine_color(water, WATER_RANGE)),
            ((oil - OIL_RANGE[0]) / (OIL_RANGE[1] - OIL_RANGE[0]),
             engine_color(oil, OIL_RANGE)),
            (oil_p / OIL_PRESSURE_MAX, (RED, 1.0) if p_low else (DIM, 0.55)),
        )
        for pr, (val, (col, alpha)) in zip(self.PEDAL_R, engine):
            val = max(0.0, min(1.0, val))
            if p_low and pr == self.PEDAL_R[2]:
                val = max(val, 0.04)  # keep a visible red stub at zero pressure
            if val > 0.01:
                cr.new_path()
                cr.set_line_width(3)
                cr.set_source_rgba(*col[:3], alpha)
                cr.arc(cx, cy, pr, self.ENGINE_START,
                       self.ENGINE_START + self.PEDAL_SWEEP * val)
                cr.stroke()

        # tyres, with the brake discs inboard on the axle lines
        tw, th = 34, 54
        lx, rx = self.TYRE_X
        self.draw_damage(cr, lx + tw, rx, susp_dmg, aero_dmg, engine_dmg)
        for i, (tx, ty) in enumerate(((lx, 51), (rx, 51), (lx, 133), (rx, 133))):
            draw_tyre(cr, tx, ty, tw, th, tyre_temp[i], tyre_hud_temp[i], tyre_wear[i],
                      self.tyre_opt)
        for i, (bx, by) in enumerate(((lx + tw, 78), (rx, 78),
                                      (lx + tw, 160), (rx, 160))):
            # caliper at the back of the front discs, the front of the rears,
            # tilted out towards the wheel
            base = math.pi / 2 if i < 2 else -math.pi / 2
            tilt = 0.5 if (i % 2 == 0) == (i < 2) else -0.5
            draw_brake(cr, bx, by, brake_temp[i], caliper=base + tilt,
                       damage=brake_dmg[i])

        self.draw_radar(cr, radar if live else ())

        if not live:
            text(cr, "waiting for AMS2 UDP…", self.W / 2, self.PANEL_H - 10, 10, DIM,
                 cairo.FONT_WEIGHT_NORMAL)

        if self.drag_armed:
            rounded_rect(cr, 1, 1, self.W - 2, self.PANEL_H - 2, 21)
            cr.set_line_width(2)
            cr.set_source_rgba(1, 1, 1, 0.6)
            cr.stroke()

        # keep redrawing only while something is moving on its own
        fl = self.flags
        self.animating = (
            abs(rpm - self.shown_rpm) > 10
            or abs(vmax - self.shown_vmax) > 0.5  # speed scale easing to a new top speed
            or frac > 0.97  # limiter flash
            or flag_on  # LEDs and the waving flag
            or fuel_low  # gear / fuel pump blink
            or raw_flag != fl.flag  # a flag change waiting out its hold time
            or now < fl.green_until
            or fl.final_lap_at is not None and now - fl.final_lap_at < FLAG_TIME
        )


# ---------------------------------------------------------------- window

CSS = b"window, .background { background: transparent; }"


POS_FILE = os.path.join(
    os.environ.get("XDG_CONFIG_HOME", os.path.expanduser("~/.config")),
    "ams2-overlay", "position.json")
HOLD_MS = 350  # press and hold this long before the panel follows the pointer
DEBUG = bool(os.environ.get("AMS2_OVERLAY_DEBUG"))


def debug(*a):
    if DEBUG:
        print("[drag]", *a, file=sys.stderr, flush=True)


class Mover:
    """Press and hold the panel, then drag it. The spot is saved per monitor
    and the panel is anchored top left with margins from then on.

    While dragging, the overlay grows to a transparent surface covering the
    monitor and draws the panel at the pointer: moving a layer surface under
    the pointer would make the drag offsets lag a frame behind and overshoot."""

    def __init__(self, win, args):
        self.win, self.args = win, args
        self.gauge = win.get_child()
        self.held = None  # GLib timeout while waiting for the hold
        self.pos = None  # (left, top) margins once anchored top left
        self.grab = (0, 0)  # pointer position inside the panel when grabbed
        saved = self.load().get(self.key())
        if saved:
            self.anchor(*saved)
        drag = Gtk.GestureDrag()
        drag.connect("drag-begin", self.on_begin)
        drag.connect("drag-update", self.on_update)
        drag.connect("drag-end", self.on_end)
        self.gauge.add_controller(drag)

    def key(self):
        return self.args.output or "default"

    def load(self):
        try:
            with open(POS_FILE) as f:
                return json.load(f)
        except (OSError, ValueError):
            return {}

    def save(self):
        data = self.load()
        data[self.key()] = list(self.pos)
        try:
            os.makedirs(os.path.dirname(POS_FILE), exist_ok=True)
            with open(POS_FILE, "w") as f:
                json.dump(data, f)
        except OSError as e:
            print(f"could not save position: {e}", file=sys.stderr)

    def set_anchors(self, top, left, bottom, right):
        E = LayerShell.Edge
        for edge, on in ((E.TOP, top), (E.LEFT, left), (E.BOTTOM, bottom),
                         (E.RIGHT, right)):
            LayerShell.set_anchor(self.win, edge, on)

    def anchor(self, left, top):
        self.set_anchors(True, True, False, False)
        LayerShell.set_margin(self.win, LayerShell.Edge.LEFT, int(left))
        LayerShell.set_margin(self.win, LayerShell.Edge.TOP, int(top))
        LayerShell.set_margin(self.win, LayerShell.Edge.BOTTOM, 0)
        self.pos = (int(left), int(top))

    def monitor_size(self):
        surface = self.win.get_surface()
        mon = LayerShell.get_monitor(self.win) or (
            self.win.get_display().get_monitor_at_surface(surface))
        geo = mon.get_geometry()
        return geo.width, geo.height

    def current_pos(self):
        """Where the panel sits now, also before it was ever moved."""
        if self.pos:
            return self.pos
        mw, mh = self.monitor_size()
        w, h = self.win.get_width(), self.win.get_height()
        top = (self.args.margin if self.args.anchor == "top"
               else mh - h - self.args.margin)
        return (mw - w) // 2, top

    def on_begin(self, _gesture, x, y):
        self.grab = (x, y)
        self.held = GLib.timeout_add(HOLD_MS, self.on_hold)

    def on_hold(self):
        self.held = None
        left, top = self.current_pos()
        self.pos = (left, top)
        self.panel = (self.win.get_width(), self.win.get_height())
        # cover the monitor; the panel keeps its place inside it. A
        # non-resizable GTK window would ignore the bigger configure size.
        self.win.set_resizable(True)
        self.set_anchors(True, True, True, True)
        for edge in LayerShell.Edge.TOP, LayerShell.Edge.LEFT, LayerShell.Edge.BOTTOM:
            LayerShell.set_margin(self.win, edge, 0)
        self.gauge.drag_offset = (left, top)
        self.gauge.drag_armed = True
        self.gauge.queue_draw()
        debug("hold at", self.pos, "panel", self.panel, "grab", self.grab)
        return False

    def on_update(self, gesture, dx, dy):
        if self.held and math.hypot(dx, dy) > 6:
            GLib.source_remove(self.held)  # moved before the hold: not a drag
            self.held = None
        if not self.gauge.drag_armed:
            return
        if self.gauge.get_width() <= self.panel[0] * 1.5:
            debug("waiting for the big surface, gauge is",
                  self.gauge.get_width(), "x", self.gauge.get_height())
            return  # the monitor sized surface hasn't arrived yet
        # offsets are measured from the grab point in the small surface, so in
        # the monitor sized one the pointer is at grab + offset
        _, sx, sy = gesture.get_start_point()
        px, py = sx + dx, sy + dy
        mw, mh = self.gauge.get_width(), self.gauge.get_height()
        left = max(0, min(mw - self.panel[0], px - self.grab[0]))
        top = max(0, min(mh - self.panel[1], py - self.grab[1]))
        self.pos = (int(left), int(top))
        self.gauge.drag_offset = self.pos
        self.gauge.queue_draw()

    def on_end(self, _gesture, _dx, _dy):
        if self.held:
            GLib.source_remove(self.held)
            self.held = None
        if self.gauge.drag_armed:
            self.gauge.drag_armed = False
            self.gauge.drag_offset = None
            self.anchor(*self.pos)
            self.win.set_resizable(False)
            self.win.set_default_size(*self.panel)
            self.gauge.queue_draw()
            self.save()
            debug("dropped at", self.pos)


def on_activate(app, args, tel):
    # launching again while running activates this instance: keep one panel
    if app.get_windows():
        app.get_windows()[0].present()
        return
    win = Gtk.ApplicationWindow(application=app, title="AMS2 Overlay")
    win.set_decorated(False)
    win.set_resizable(False)

    provider = Gtk.CssProvider()
    provider.load_from_data(CSS)
    Gtk.StyleContext.add_provider_for_display(
        win.get_display(), provider, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION
    )

    if LayerShell and LayerShell.is_supported():
        LayerShell.init_for_window(win)
        LayerShell.set_namespace(win, "ams2-overlay")
        LayerShell.set_layer(win, LayerShell.Layer.OVERLAY)
        LayerShell.set_keyboard_mode(win, LayerShell.KeyboardMode.NONE)
        LayerShell.set_exclusive_zone(win, -1)
        edge = LayerShell.Edge.TOP if args.anchor == "top" else LayerShell.Edge.BOTTOM
        LayerShell.set_anchor(win, edge, True)
        LayerShell.set_margin(win, edge, args.margin)
        if args.output:
            monitors = win.get_display().get_monitors()
            names = [monitors.get_item(i).get_connector()
                     for i in range(monitors.get_n_items())]
            if args.output in names:
                LayerShell.set_monitor(win, monitors.get_item(names.index(args.output)))
            else:
                print(f"output {args.output!r} not found, have: {', '.join(names)}",
                      file=sys.stderr)
    else:
        print("gtk4-layer-shell not available: running as a normal window "
              "(install 'gtk4-layer-shell' for a real overlay).", file=sys.stderr)

    win.set_child(Gauge(tel, args.scale, args.units, args.tyre_opt, args.fps,
                        args.vmax))

    if LayerShell and LayerShell.is_supported():
        Mover(win, args)
    # right click closes the overlay
    close = Gtk.GestureClick(button=3)
    close.connect("released", lambda *_: app.quit())
    win.get_child().add_controller(close)
    win.present()


def main():
    p = argparse.ArgumentParser(description="Automobilista 2 gauge overlay")
    p.add_argument("--port", type=int, default=5606, help="UDP port (default 5606)")
    p.add_argument("--demo", action="store_true", help="fake telemetry for preview")
    p.add_argument("--anchor", choices=("top", "bottom"), default="bottom")
    p.add_argument("--margin", type=int, default=40, help="distance from edge in px")
    p.add_argument("--scale", type=float, default=1.0)
    p.add_argument("--output", help="monitor connector to show on, e.g. DP-1")
    p.add_argument("--fps", type=int, default=30,
                   help="max overlay redraw rate (default 30)")
    p.add_argument("--units", choices=("kmh", "mph"), default="kmh")
    p.add_argument("--vmax", type=float, default=0,
                   help="top of the speed arc until the car's top speed is learned "
                        "in top gear (default 320 km/h or 200 mph)")
    p.add_argument("--tyre-opt", type=float, default=80,
                   help="optimal tyre temperature in °C, green on the heat map")
    args = p.parse_args()

    tel = Telemetry()
    target = demo_feeder if args.demo else udp_listener
    targs = (tel,) if args.demo else (tel, args.port)
    threading.Thread(target=target, args=targs, daemon=True).start()

    app = Gtk.Application(application_id="dev.local.ams2overlay")
    app.connect("activate", on_activate, args, tel)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app.run([])


if __name__ == "__main__":
    main()
