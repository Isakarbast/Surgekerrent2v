#!/usr/bin/env python3
"""Parametric geometry / kinematics / load calculator for the SK2V scooter chassis.

Reads chassis_params.json, prints a report, and writes a side-view layout SVG.
These are first-order hand calculations to size the layout and seed CAD/FEA;
they are not a substitute for FEA, rig testing or road testing.

Coordinates (side view, rider aboard at sag): x forward from the REAR axle,
z up from the ground, y to the left. Units mm / N / kg / deg.

    python3 tools/chassis/chassis_calc.py                  # report + SVG
    python3 tools/chassis/chassis_calc.py --optimise       # re-tune rear linkage
    python3 tools/chassis/chassis_calc.py --optimise --write   # ... and save it
"""
import argparse
import json
import math
import os
import random
import re

import numpy as np

G = 9.81
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))


def v2(a):
    return np.array(a, dtype=float)


def cross2(a, b):
    return a[0] * b[1] - a[1] * b[0]


def rot2(p, ang):
    c, s = math.cos(ang), math.sin(ang)
    return np.array([c * p[0] - s * p[1], s * p[0] + c * p[1]])


def seg_point_dist(p, a, b):
    ab = b - a
    t = np.clip(np.dot(p - a, ab) / np.dot(ab, ab), 0.0, 1.0)
    return float(np.linalg.norm(p - (a + t * ab)))


# --------------------------------------------------------------------------
# Model
# --------------------------------------------------------------------------
class Chassis:
    def __init__(self, P):
        self.P = P
        t = P["tires"]["list"][P["tires"]["design_tire"]]
        self.R = t["od"] / 2.0
        self.W = t["width"]
        self.Rg = self.R * (1 + P["tires"]["speed_growth_pct_dia"] / 100.0)
        self.Wg = self.W + P["tires"]["speed_growth_width"]
        self.WB = P["vehicle"]["wheelbase_sag"]
        c = P["clearance"]
        self.gc = c["ground_clearance_sag"]
        self.deck_top = self.gc + c["deck_structure_height"]
        self._front()
        self._mass()
        self._rear()
        self.link = {k: v2(v) for k, v in P["linkage_seed"].items() if k != "_comment"}

    # ---- front ----------------------------------------------------------
    def _front(self):
        f = self.P["front"]
        rk = math.radians(f["rake_deg"])
        self.rk = rk
        self.u = v2([-math.sin(rk), math.cos(rk)])   # steering axis, up
        self.n = v2([math.cos(rk), math.sin(rk)])    # perpendicular, forward
        self.Af = v2([self.WB, self.R])
        self.Q0 = self.Af - f["fork_offset"] * self.n  # axis point level with axle
        self.x_g = self.Q0[0] + self.Q0[1] * math.tan(rk)
        self.trail = self.x_g - self.WB
        self.fsag = f["fork_travel_axial"] * f["fork_sag_pct"] / 100.0
        a2c = f["fork_a2c_extended"] - self.fsag
        self.crown_lo = self.Q0 + a2c * self.u
        self.ht_bot = self.crown_lo + f["lower_crown_thickness"] * self.u
        self.ht_top = self.ht_bot + f["head_tube_length"] * self.u
        self.crown_up = self.ht_top + f["upper_crown_thickness"] * self.u
        s_bar = (f["bar_height"] - self.Q0[1]) / math.cos(rk)
        self.bar = self.Q0 + s_bar * self.u

    def trail_at(self, rake_deg, R=None, offset=None):
        R = self.R if R is None else R
        off = self.P["front"]["fork_offset"] if offset is None else offset
        r = math.radians(rake_deg)
        return (R * math.sin(r) - off) / math.cos(r)

    # ---- mass -------------------------------------------------------------
    def _mass(self):
        v = self.P["vehicle"]
        mr = v["rider_mass_design"] + v["rider_gear_mass"]
        mv = v["vehicle_mass"]
        zr = self.deck_top + v["rider_cg_above_deck"]
        self.m = mr + mv
        self.x_cg = (mr * v["rider_cg_x_from_rear_axle"] + mv * v["vehicle_cg_x_from_rear_axle"]) / self.m
        self.h_cg = (mr * zr + mv * v["vehicle_cg_z"]) / self.m
        self.Wt = self.m * G
        self.front_frac = self.x_cg / self.WB

    # ---- rear -------------------------------------------------------------
    def _rear(self):
        r = self.P["rear"]
        L = r["swingarm_length"]
        th0 = math.radians(r["swingarm_angle_sag_deg"])
        self.L = L
        self.th0 = th0
        self.Pv = v2([L * math.cos(th0), self.R + L * math.sin(th0)])
        self.rsag = r["wheel_travel_vertical"] * r["sag_pct"] / 100.0
        self.z_top = self.R - self.rsag
        self.z_bot = self.R + r["wheel_travel_vertical"] - self.rsag

    def theta_for_axle_z(self, z):
        return math.asin((self.Pv[1] - z) / self.L)

    def axle(self, th):
        return v2([self.Pv[0] - self.L * math.cos(th), self.Pv[1] - self.L * math.sin(th)])

    def swing(self, p0, th):
        return self.Pv + rot2(p0 - self.Pv, th - self.th0)

    def anti_squat(self, th):
        a = self.axle(th)
        cp = v2([a[0], a[1] - self.R])
        slope = (self.Pv[1] - cp[1]) / (self.Pv[0] - cp[0])
        return 100.0 * slope / (self.h_cg / self.WB)

    # ---- linkage ------------------------------------------------------------
    def link_sweep(self, link=None, n=41):
        k = self.link if link is None else link
        A0, R, B0, C0, S = k["A"], k["R"], k["B"], k["C"], k["S"]
        lp = np.linalg.norm(B0 - A0)
        rb = np.linalg.norm(B0 - R)
        zs = np.linspace(self.z_top, self.z_bot, n)
        out = []
        Bprev = None
        for z in zs:
            th = self.theta_for_axle_z(z)
            A = self.swing(A0, th)
            d = np.linalg.norm(A - R)
            if d > lp + rb or d < abs(lp - rb):
                return None
            a = (rb * rb - lp * lp + d * d) / (2 * d)
            h = math.sqrt(max(rb * rb - a * a, 0.0))
            e = (A - R) / d
            mid = R + a * e
            perp = v2([-e[1], e[0]])
            cands = [mid + h * perp, mid - h * perp]
            ref = B0 if Bprev is None else Bprev
            B = min(cands, key=lambda c: np.linalg.norm(c - ref))
            psi = math.atan2(*(B - R)[::-1]) - math.atan2(*(B0 - R)[::-1])
            C = R + rot2(C0 - R, psi)
            out.append(dict(z=z, th=th, A=A, B=B, C=C, psi=psi,
                            Ls=float(np.linalg.norm(C - S))))
            Bprev = B
        wt = np.array([o["z"] - self.z_top for o in out])
        Ls = np.array([o["Ls"] for o in out])
        dL = np.gradient(Ls, wt)
        with np.errstate(divide="ignore"):
            lr = -1.0 / dL
        return dict(pts=out, wt=wt, Ls=Ls, lr=lr)


# --------------------------------------------------------------------------
# Linkage optimisation
# --------------------------------------------------------------------------
def link_cost(c, link):
    P = c.P
    r = P["rear"]
    sw = c.link_sweep(link)
    if sw is None:
        return 1e9, None
    lr = sw["lr"]
    if not np.all(np.isfinite(lr)) or np.any(lr <= 0):
        return 1e9, None
    stroke = sw["Ls"][0] - sw["Ls"][-1]
    cost = 0.0
    cost += 40 * (lr[1] - r["lr_target_start"]) ** 2
    cost += 40 * (lr[-2] - r["lr_target_end"]) ** 2
    cost += 2.0 * (stroke - (r["shock_stroke"] - 1.0)) ** 2
    cost += 5 * (sw["Ls"][0] - r["shock_eye_to_eye"]) ** 2
    rising = np.diff(lr[1:-1])
    cost += 500 * float(np.sum(np.clip(rising, 0, None)))
    curv = np.diff(lr[1:-1], 2)
    cost += 2000 * float(np.sum(curv ** 2))
    pts = sw["pts"]
    psi_range = abs(pts[-1]["psi"] - pts[0]["psi"])
    cost += 10 * max(0.0, math.degrees(psi_range) - 50) ** 2
    Rg = c.Rg
    for o in pts:
        ax = c.axle(o["th"])
        # transmission angle push-rod vs rocker arm
        rod = o["B"] - o["A"]
        arm = o["B"] - link["R"]
        ang = math.degrees(math.acos(np.clip(np.dot(rod, arm) /
                                             (np.linalg.norm(rod) * np.linalg.norm(arm)), -1, 1)))
        cost += 2 * max(0.0, 40 - ang) ** 2 + 2 * max(0.0, ang - 140) ** 2
        need_s = Rg + 35 + 12
        ds = seg_point_dist(ax, o["C"], link["S"])
        cost += 5 * max(0.0, need_s - ds) ** 2
        for p, need in ((o["B"], Rg + 12 + 15), (o["C"], Rg + 12 + 20),
                        (link["R"], Rg + 12 + 20)):
            cost += 5 * max(0.0, need - float(np.linalg.norm(p - ax))) ** 2
        dpr = seg_point_dist(ax, o["A"], o["B"])
        cost += 5 * max(0.0, Rg + 12 + 12 - dpr) ** 2
    A0 = link["A"]
    ax0 = c.axle(c.th0)
    cost += 5 * max(0.0, Rg + 12 + 20 - float(np.linalg.norm(A0 - ax0))) ** 2
    # packaging box: tail tower behind x=385, no hardpoint above z=470
    for key in ("R", "B", "C", "S"):
        p = link[key]
        cost += 2 * max(0.0, p[0] - 385) ** 2 + 2 * max(0.0, p[1] - 470) ** 2
        cost += 2 * max(0.0, c.deck_top + 60 - p[1]) ** 2
    cost += 2 * max(0.0, -40 - link["S"][0]) ** 2
    # push-rod pick-up must sit on the swingarm between pivot and tyre
    cost += 2 * max(0.0, A0[0] - (c.Pv[0] - 40)) ** 2
    # A sits on top of the swingarm cross-brace: 0..45 mm above the pivot-axle line
    off = -cross2(c.axle(c.th0) - c.Pv, A0 - c.Pv) / c.L   # + = above the line
    cost += 5 * max(0.0, 0.0 - off) ** 2 + 5 * max(0.0, off - 45) ** 2
    cost += 5 * max(0.0, 70 - (c.Pv[0] - A0[0])) ** 2
    return cost, sw


def optimise(c, iters=40000, seed=1):
    rnd = random.Random(seed)
    best = {k: v.copy() for k, v in c.link.items()}
    bc, _ = link_cost(c, best)
    step = 12.0
    for i in range(iters):
        cand = {k: v.copy() for k, v in best.items()}
        for key in rnd.sample(list(cand), rnd.randint(1, 3)):
            cand[key] = cand[key] + np.array([rnd.gauss(0, step), rnd.gauss(0, step)])
        cc, _ = link_cost(c, cand)
        if cc < bc:
            best, bc = cand, cc
        if i % 4000 == 3999:
            step = max(1.0, step * 0.7)
    for k in best:
        best[k] = np.round(best[k], 1)
    return best, link_cost(c, best)[0]


# --------------------------------------------------------------------------
# Front tyre swept-volume clearance (steer x travel)
# --------------------------------------------------------------------------
def tire_points(c, n_phi=96, n_y=9):
    Rg, Wg = c.Rg, c.Wg
    pts = []
    for y in np.linspace(-Wg / 2, Wg / 2, n_y):
        r = Rg - 0.18 * Wg * (2 * y / Wg) ** 2
        for ph in np.linspace(0, 2 * math.pi, n_phi, endpoint=False):
            pts.append([r * math.cos(ph), y, r * math.sin(ph)])
    for side in (-1, 1):     # sidewall
        for rr in np.linspace(0.62 * Rg, 0.95 * Rg, 5):
            for ph in np.linspace(0, 2 * math.pi, n_phi, endpoint=False):
                pts.append([rr * math.cos(ph), side * Wg / 2, rr * math.sin(ph)])
    return np.array(pts)


def front_sweep_clearance(c, frame_pts):
    f = c.P["front"]
    U = np.array([-math.sin(c.rk), 0.0, math.cos(c.rk)])
    tp = tire_points(c)
    lock = c.P["clearance"]["steer_lock_deg"]
    best = (1e9, None)
    for travel in np.linspace(0, f["fork_travel_axial"], 9):
        ax2 = c.Af + (travel - c.fsag) * c.u
        q2 = ax2 - f["fork_offset"] * c.n
        ax = np.array([ax2[0], 0.0, ax2[1]])
        q = np.array([q2[0], 0.0, q2[1]])
        for d in np.linspace(-lock, lock, 17):
            a = math.radians(d)
            K = np.array([[0, -U[2], U[1]], [U[2], 0, -U[0]], [-U[1], U[0], 0]])
            Rm = np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K
            world = ((tp + ax - q) @ Rm.T) + q
            dmat = np.linalg.norm(world[:, None, :] - frame_pts[None, :, :], axis=2)
            i = np.unravel_index(np.argmin(dmat), dmat.shape)
            if dmat[i] < best[0]:
                best = (float(dmat[i]), dict(travel=float(travel), steer=float(d),
                                              frame_pt=frame_pts[i[1]].round(1).tolist()))
    return best


def neck_frame_points(c):
    """Sampled outer surface of the deck front + neck (fixed structure)."""
    d = c.P["deck"]
    xf = d["battery_bay_front_x"]
    w_deck = c.P["clearance"]["deck_width"] / 2
    pts = []
    # deck front wall (full width)
    for z in np.linspace(c.gc, c.deck_top, 7):
        for y in np.linspace(-w_deck, w_deck, 9):
            pts.append([xf, y, z])
    # neck: lower face from deck front-top to head-tube bottom-rear, 110 wide
    hb = c.ht_bot - (c.P["front"]["head_tube_od"] / 2 + 2) * c.n
    p0 = v2([xf, c.deck_top - 40])
    for t in np.linspace(0, 1, 25):
        p = p0 + t * (hb - p0)
        for y in np.linspace(-55, 55, 7):
            pts.append([p[0], y, p[1]])
    # head tube outer surface below mid-height
    for s in np.linspace(0, 40, 5):
        p = c.ht_bot + s * c.u
        for ang in np.linspace(0, 2 * math.pi, 16, endpoint=False):
            r = c.P["front"]["head_tube_od"] / 2
            off = r * math.cos(ang) * c.n
            pts.append([p[0] + off[0], r * math.sin(ang), p[1] + off[1]])
    return np.array(pts), p0, hb


# --------------------------------------------------------------------------
# Loads
# --------------------------------------------------------------------------
def load_cases(c):
    W = c.Wt
    ff = c.front_frac
    Wf, Wr = W * ff, W * (1 - ff)
    hb = c.h_cg / c.WB
    cases = []

    def add(name, Ff, Fr, lat=0.0, note="", at_axle=False):
        cases.append(dict(name=name, Ff=v2(Ff), Fr=v2(Fr), lat=lat, note=note, at_axle=at_axle))
    add("1 static standing (1 g)", [0, Wf], [0, Wr])
    a = 0.7
    add("2 hard acceleration 0.7 g (rear drive)", [0, W * (ff - a * hb)], [W * a, W * (1 - ff + a * hb)])
    add("3 max braking: stoppie limit, mu=1.0 front", [-W, W], [0, 0], note="all weight on front")
    add("4 landing 4 g (both wheels)", [0, 4 * Wf], [0, 4 * Wr])
    add("5a front one-wheel impact 4 g + 0.5 rearward", [-0.5 * 4 * Wf, 4 * Wf], [0, Wr])
    add("5b rear one-wheel impact 4 g", [0, Wf], [0, 4 * Wr])
    add("6 cornering 1.0 g (45 deg lean)", [0, 1.414 * Wf], [0, 1.414 * Wr], lat=0.25,
        note="in-plane x1.41, out-of-plane 0.25 Fz")
    ab = 0.7
    Fzf = W * 1.22 * min(1.0, ff + ab * hb)
    add("7 braking 0.7 g + cornering 0.7 g", [-W * ab * 0.9, Fzf], [-W * ab * 0.1, W * 1.22 - Fzf],
        lat=0.25)
    add("10 frontal crash: 3.5 W rearward at front axle", [-3.5 * W, Wf], [0, Wr],
        note="applied at axle; design to bend, not fracture", at_axle=True)
    return cases


def head_bearing_reactions(c, F, at_axle=False):
    """Fork as rigid body on two head bearings. F from ground on tyre (Fx, Fz)."""
    s_lo = 10.0
    s_up = c.P["front"]["head_tube_length"] - 10.0
    Lb = c.ht_bot + s_lo * c.u
    span = s_up - s_lo
    Cp = c.Af.copy() if at_axle else v2([c.WB, 0.0])
    M = cross2(Cp - Lb, F)
    Fu = M / span
    Fl_perp = -float(np.dot(F, c.n)) - Fu
    axial = float(np.dot(F, c.u))
    return Fu, Fl_perp, axial


def rear_forces(c, F, sw0):
    k = c.link
    A, B, C, S, R = k["A"], k["B"], k["C"], k["S"], k["R"]
    ax = c.axle(c.th0)
    cp = v2([ax[0], 0.0])
    Mc = cross2(cp - c.Pv, F)
    d = (B - A) / np.linalg.norm(B - A)
    T = Mc / cross2(A - c.Pv, d)
    e = (C - S) / np.linalg.norm(C - S)
    Fs = -cross2(B - R, T * d) / cross2(C - R, e)
    pivot = -F + T * d
    rocker = -(T * d + Fs * e)
    return dict(rod=T, shock=Fs, pivot=pivot, rocker=rocker)


def deck_section(c):
    """Deck spine section (top cover excluded: non-structural)."""
    d = c.P["deck"]
    H = c.P["clearance"]["deck_structure_height"] - d["cover_thickness"]
    bw = d["side_rail_width"]
    t = 3.0
    wf = c.P["clearance"]["deck_width"] - 2 * bw
    tf = d["floor_thickness"]
    parts = []   # (area, centroid z, own I)
    for _ in range(2):
        a_out = bw * H
        a_in = (bw - 2 * t) * (H - 2 * t)
        i_out = bw * H ** 3 / 12
        i_in = (bw - 2 * t) * (H - 2 * t) ** 3 / 12
        parts.append((a_out - a_in, H / 2, i_out - i_in))
    parts.append((wf * tf, tf / 2, wf * tf ** 3 / 12))
    A = sum(p[0] for p in parts)
    zc = sum(p[0] * p[1] for p in parts) / A
    I = sum(p[2] + p[0] * (p[1] - zc) ** 2 for p in parts)
    return dict(A=A, zc=zc, I=I, c=max(zc, H - zc), H=H)


# --------------------------------------------------------------------------
# Report
# --------------------------------------------------------------------------
def report(c, args):
    P = c.P
    f, r, cl = P["front"], P["rear"], P["clearance"]
    out = []
    pr = out.append

    pr("== TYRES ==")
    for name, t in P["tires"]["list"].items():
        R = t["od"] / 2
        g = P["tires"]["speed_growth_pct_dia"] / 100
        rpm = (P["vehicle"]["v_max_kmh"] / 3.6) / (math.pi * t["od"] / 1000) * 60
        pr(f"  {name:8s} OD {t['od']} W {t['width']}  grown OD {t['od']*(1+g):.0f}  "
           f"rpm@{P['vehicle']['v_max_kmh']} {rpm:.0f}  trail {c.trail_at(f['rake_deg'], R):.1f}  "
           f"lateral gap in {r['dropout_spacing']} dropout {(r['dropout_spacing']-t['width']-P['tires']['speed_growth_width'])/2:.1f}/side")
    pr(f"  design envelope: {P['tires']['design_tire']} R={c.R:.1f} grown R={c.Rg:.1f} W={c.Wg:.0f}")

    pr("\n== FRONT GEOMETRY (at sag) ==")
    pr(f"  rake {f['rake_deg']:.1f} deg (head angle {90-f['rake_deg']:.1f}), offset {f['fork_offset']} mm")
    pr(f"  ground trail {c.trail:.1f} mm, normal trail {c.trail*math.cos(c.rk):.1f} mm")
    pr(f"  axis meets ground x={c.x_g:.1f}")
    pr(f"  fork sag {c.fsag:.0f} mm axial ({c.fsag*math.cos(c.rk):.0f} vertical); "
       f"vertical travel {f['fork_travel_axial']*math.cos(c.rk):.0f} mm")
    for nm, p in (("lower crown", c.crown_lo), ("head tube bottom", c.ht_bot),
                  ("head tube top", c.ht_top), ("upper crown top", c.crown_up), ("bar centre", c.bar)):
        pr(f"  {nm:18s} x={p[0]:7.1f} z={p[1]:6.1f}")
    pr("  trail vs rake (11in / 10in):")
    for rd in (22, 23, 24, 25, 26, 27):
        pr(f"    rake {rd}: {c.trail_at(rd):5.1f} / {c.trail_at(rd, P['tires']['list']['10x2.75']['od']/2):5.1f}")
    pr("  dynamic trail (pitch from suspension):")
    vt_f = f["fork_travel_axial"] * math.cos(c.rk)
    for nm, df, dr in (("hard braking: fork 90%, rear top-out", 0.9 * vt_f - c.fsag * math.cos(c.rk), -c.rsag),
                       ("hard accel: fork top-out, rear 80%", -c.fsag * math.cos(c.rk), 0.8 * r["wheel_travel_vertical"] - c.rsag),
                       ("both bottomed", vt_f - c.fsag * math.cos(c.rk), r["wheel_travel_vertical"] - c.rsag)):
        pitch = math.degrees(math.atan((df - dr) / c.WB))
        pr(f"    {nm:38s} pitch {pitch:+5.2f} deg -> rake {f['rake_deg']-pitch:5.2f}, trail {c.trail_at(f['rake_deg']-pitch):5.1f}")

    pr("\n== MASS / CG ==")
    pr(f"  GVW {c.m:.0f} kg  CG x={c.x_cg:.0f} (from rear axle) h={c.h_cg:.0f}")
    pr(f"  static split front/rear {100*c.front_frac:.1f} / {100*(1-c.front_frac):.1f} %")
    pr(f"  pitch-over (stoppie) decel limit {(c.WB - c.x_cg)/c.h_cg:.2f} g; wheelie limit {c.x_cg/c.h_cg:.2f} g")
    pr("  stoppie limit vs wheelbase:")
    for wb in (1150, 1200, 1250, 1300, 1350, 1400):
        xr = c.x_cg * wb / c.WB
        pr(f"    WB {wb}: {(wb - xr)/c.h_cg:.2f} g")
    v = P["vehicle"]
    vms = v["v_max_kmh"] / 3.6
    paero = 0.5 * 1.2 * v["cda_m2"] * vms ** 3
    proll = v["crr"] * c.Wt * vms
    pr(f"  road-load power @ {v['v_max_kmh']} km/h: aero {paero/1e3:.1f} kW + roll {proll/1e3:.1f} kW -> "
       f"battery {(paero+proll)/v['drivetrain_eff']/1e3:.1f} kW")

    pr("\n== REAR GEOMETRY ==")
    pr(f"  swingarm {c.L} mm, pivot x={c.Pv[0]:.1f} z={c.Pv[1]:.1f}")
    th_t = c.theta_for_axle_z(c.z_top)
    th_b = c.theta_for_axle_z(c.z_bot)
    pr(f"  swingarm angle top-out {math.degrees(th_t):.1f}, sag {math.degrees(c.th0):.1f}, bottom {math.degrees(th_b):.1f} deg")
    pr(f"  sag {c.rsag:.1f} mm; axle x top/sag/bottom {c.axle(th_t)[0]:.1f}/{c.axle(c.th0)[0]:.1f}/{c.axle(th_b)[0]:.1f}"
       f" (wheelbase change {c.axle(th_t)[0]-c.axle(th_b)[0]:+.1f} mm over travel)")
    pr("  anti-squat / anti-rise (hub motor & caliper on swingarm -> same line):")
    for zz in np.linspace(c.z_top, c.z_bot, 5):
        th = c.theta_for_axle_z(zz)
        pr(f"    travel {zz-c.z_top:5.1f}  {c.anti_squat(th):5.1f} %")

    if args.optimise:
        best, cost = optimise(c)
        pr(f"\n  [optimiser] cost {cost:.3f}")
        c.link = best
        if args.write:
            P["linkage_seed"].update({k: v.tolist() for k, v in best.items()})
            txt = json.dumps(P, indent=2)
            txt = re.sub(r"\[\s+([-\d.]+),\s+([-\d.]+)\s+\]", r"[\1, \2]", txt)
            with open(args.params, "w") as fh:
                fh.write(txt + "\n")
            pr("  [optimiser] hardpoints written back to params")

    sw = c.link_sweep()
    pr("\n== REAR LINKAGE (rocker on frame, push-rod from swingarm) ==")
    for k_, p_ in c.link.items():
        pr(f"  {k_}: x={p_[0]:7.1f} z={p_[1]:6.1f}")
    if sw is None:
        pr("  LINKAGE DOES NOT ASSEMBLE")
        return out, None
    lp = np.linalg.norm(c.link["B"] - c.link["A"])
    pr(f"  push-rod {lp:.1f} mm, rocker R-B {np.linalg.norm(c.link['B']-c.link['R']):.1f}, "
       f"R-C {np.linalg.norm(c.link['C']-c.link['R']):.1f}")
    pr(f"  shock length top-out {sw['Ls'][0]:.1f}, bottom {sw['Ls'][-1]:.1f}, stroke used {sw['Ls'][0]-sw['Ls'][-1]:.1f} of {r['shock_stroke']}")
    lr = sw["lr"]
    pr(f"  leverage ratio start {lr[1]:.2f} -> end {lr[-2]:.2f}, mean {r['wheel_travel_vertical']/(sw['Ls'][0]-sw['Ls'][-1]):.2f}, "
       f"progression {100*(lr[1]-lr[-2])/lr[1]:.1f} %")
    pr(f"  rocker rotation {math.degrees(sw['pts'][-1]['psi']-sw['pts'][0]['psi']):.1f} deg")
    for i in range(0, len(sw["wt"]), 5):
        pr(f"    wheel {sw['wt'][i]:6.1f}  shock {sw['Ls'][i]:6.1f}  LR {lr[i]:4.2f}")
    isag = int(np.argmin(abs(sw["wt"] - c.rsag)))
    shock_sag = sw["Ls"][0] - sw["Ls"][isag]
    pr(f"  shock sag {shock_sag:.1f} mm = {100*shock_sag/r['shock_stroke']:.0f} % of stroke")
    # min clearances of rear linkage to tyre
    mins = dict(shock=1e9, rocker=1e9, pushrod=1e9, pickup=1e9)
    for o in sw["pts"]:
        ax = c.axle(o["th"])
        mins["shock"] = min(mins["shock"], seg_point_dist(ax, o["C"], c.link["S"]) - c.Rg - 35)
        mins["rocker"] = min(mins["rocker"], min(np.linalg.norm(o["B"] - ax), np.linalg.norm(o["C"] - ax),
                                                 np.linalg.norm(c.link["R"] - ax)) - c.Rg - 20)
        mins["pushrod"] = min(mins["pushrod"], seg_point_dist(ax, o["A"], o["B"]) - c.Rg - 12)
    mins["pickup"] = float(np.linalg.norm(c.link["A"] - c.axle(c.th0))) - c.Rg - 20
    pr("  min radial clearance grown tyre -> (shock body r35 / rocker r20 / push-rod r12 / pick-up r20): "
       + ", ".join(f"{k} {v:.1f}" for k, v in mins.items()))

    # springs
    pr("\n== SPRINGS / SAG ==")
    unsprung_r = 13.0
    Fw = c.Wt * (1 - c.front_frac) - unsprung_r * G
    LRs = lr[isag]
    pre = 2.0
    k_sh = Fw * LRs / (shock_sag + pre)
    k_w = k_sh / LRs ** 2
    m_spr = Fw / G
    kt = 180.0
    k_ser = 1 / (1 / k_w + 1 / kt)
    pr(f"  rear sprung load at wheel {Fw:.0f} N; LR at sag {LRs:.2f}")
    pr(f"  rear shock spring ~{k_sh:.0f} N/mm ({k_sh*5.71:.0f} lb/in) with {pre:.0f} mm preload; wheel rate {k_w:.1f} N/mm")
    pr(f"  rear ride frequency {math.sqrt(k_ser*1000/m_spr)/(2*math.pi):.2f} Hz (incl. tyre {kt:.0f} N/mm)")
    full_sh = k_sh * (r['shock_stroke'] + pre)
    pr(f"  spring force at full stroke {full_sh/1e3:.1f} kN; design shock-mount load (x1.6 bumper) {1.6*full_sh/1e3:.1f} kN")
    unsprung_f = f["unsprung_mass_hubmotor"] + 3.0
    Ff = c.Wt * c.front_frac - unsprung_f * G
    k_f = Ff * math.cos(c.rk) / c.fsag
    pr(f"  front sprung load {Ff:.0f} N -> total fork spring ~{k_f:.1f} N/mm axial ({k_f/2:.1f} per leg) for {c.fsag:.0f} mm sag")
    pr(f"  front ride frequency {math.sqrt(1/(1/(k_f/math.cos(c.rk)**2)+1/kt)*1000/(Ff/G))/(2*math.pi):.2f} Hz")
    pr(f"  front unsprung/sprung ratio {unsprung_f/(Ff/G):.2f} (hub motor: {f['unsprung_mass_hubmotor']} kg)")

    # clearances
    pr("\n== CLEARANCE ==")
    pts, p0, hb = neck_frame_points(c)
    d, info = front_sweep_clearance(c, pts)
    pr(f"  front grown tyre -> deck front / neck / head tube, all travel x +/-{cl['steer_lock_deg']} deg steer: "
       f"min {d:.1f} mm at {info}")
    pr(f"  neck lower face from x={p0[0]:.0f} z={p0[1]:.0f} to x={hb[0]:.0f} z={hb[1]:.0f}")
    ax_b = c.axle(th_b)
    xb = P["deck"]["battery_bay_rear_x"]
    gap_bulk = math.hypot(max(0, xb - 90 - ax_b[0]), 0) - c.Rg
    pr(f"  rear tyre (bottomed) front face to rear-subframe front zone x>={xb-90}: {gap_bulk:.1f} mm")
    drop_f = (f["fork_travel_axial"] - c.fsag) * math.cos(c.rk)
    drop_r = r["wheel_travel_vertical"] - c.rsag
    pr(f"  ground clearance at sag {c.gc} mm; both ends bottomed: ~{c.gc - (drop_f+drop_r)/2:.0f} mm "
       f"(before tyre deflection)")
    hw = cl["deck_width"] / 2
    for comp in (0, 30):
        h = c.gc - comp
        lean = math.degrees(math.atan2(h, hw - 0.0))
        pr(f"  deck-corner lean limit (suspension {comp} mm deeper than sag): ~{lean:.0f} deg")
    xt = tail_front_x(c)
    pr(f"  deck top {c.deck_top} mm, usable standing deck x {xt:.0f}..{P['deck']['battery_bay_front_x']} "
       f"= {P['deck']['battery_bay_front_x'] - xt:.0f} mm (tail front face to deck front)")

    # battery
    pr("\n== BATTERY BAY ==")
    b, dk = P["battery"], P["deck"]
    Lin = dk["battery_bay_front_x"] - dk["battery_bay_rear_x"] - 2 * dk["bulkhead_wall"]
    Win = cl["deck_width"] - 2 * dk["side_rail_width"]
    Hin = cl["deck_structure_height"] - dk["floor_thickness"] - dk["cover_thickness"] - 4
    pr(f"  internal bay L {Lin} x W {Win} x H {Hin} mm = {Lin*Win*Hin/1e6:.1f} L")
    chan = b.get("wiring_channel_each_side", 0)
    ncol = int((Win - 2 * chan - 2 * b["liner_each_side"] - 2 * b["holder_edge"]) // b["holder_pitch"])
    nrow = int((Lin - b["end_zone_len"] - 2 * b["liner_each_side"] - 2 * b["holder_edge"]) // b["holder_pitch"])
    n = ncol * nrow
    pr(f"  vertical cells, square grid: {ncol} across x {nrow} along = {n} cells "
       f"(height use {b['stack_height_allow']} of {Hin}; {chan} mm wiring channel each side)")
    pr(f"  cell block {nrow*b['holder_pitch']+2*b['holder_edge']:.0f} x {ncol*b['holder_pitch']+2*b['holder_edge']:.0f} mm; "
       f"end zone {b['end_zone_len']} mm")
    for s, p in ((20, n // 20), (22, n // 22)):
        pr(f"    {s}S{p}P: {s*p} cells, {s*b['cell_v_nom']:.0f} V nom / {s*4.2:.0f} V full, "
           f"{p*b['cell_ah']:.1f} Ah, {s*p*b['cell_v_nom']*b['cell_ah']/1e3:.2f} kWh, cells {s*p*b['cell_mass_kg']:.1f} kg")

    # brakes
    pr("\n== BRAKES ==")
    br = P["brakes"]
    E = 0.5 * c.m * vms ** 2
    Ef = E * (1 - br["regen_share"]) * br["front_energy_share"] * br["fraction_into_rotor"]
    pr(f"  KE @ {v['v_max_kmh']} km/h {E/1e3:.0f} kJ; front rotor absorbs ~{Ef/1e3:.0f} kJ per stop "
       f"(regen {100*br['regen_share']:.0f} %, front share {100*br['front_energy_share']:.0f} %)")
    for od, t in br["options"]:
        ri = od / 2 - br["rotor_band_width"]
        band = math.pi * ((od / 2) ** 2 - ri ** 2) * t * 7.8e-6
        mass = band * (1 + br["rotor_spider_mass_frac"])
        dT = Ef / (mass * 480)
        rr = (od / 2 - br["rotor_band_width"] / 2) / 1000
        torque = c.Wt * 1.0 * c.R / 1000
        pr(f"    {od}x{t}: rotor ~{mass*1000:.0f} g, single-stop bulk dT ~{dT:.0f} K "
           f"{'(over limit)' if dT > br['rotor_max_temp_rise_ok'] else ''}; "
           f"pad clamp force at 1 g ~{torque/rr/2/0.45/1e3:.1f} kN/side (mu 0.45)")
    pr(f"  front brake torque at 1.0 g stoppie: {c.Wt*c.R/1000:.0f} N.m")

    # loads
    pr("\n== LOAD CASES (quasi-static, contact-patch forces, N) ==")
    sec = deck_section(c)
    pr(f"  deck spine section: A {sec['A']:.0f} mm2, I {sec['I']/1e4:.0f} cm4, c {sec['c']:.0f} mm")
    J = v2([P["deck"]["battery_bay_front_x"], c.gc + cl["deck_structure_height"] / 2])
    nb, nd, nt = 110.0, 130.0, 5.0      # neck box section (width, depth, wall)
    nZ = (nb * nd ** 3 - (nb - 2 * nt) * (nd - 2 * nt) ** 3) / 12 / (nd / 2)
    pr(f"  neck box {nb:.0f}x{nd:.0f}x{nt:.0f}: Z {nZ/1e3:.0f} cm3")
    for lc in load_cases(c):
        Ff_, Fr_ = lc["Ff"], lc["Fr"]
        Fu, Fl, Fa = head_bearing_reactions(c, Ff_, lc["at_axle"])
        Cp = c.Af if lc["at_axle"] else v2([c.WB, 0.0])
        Mj = cross2(Cp - J, Ff_)
        Hm = c.ht_bot + 0.5 * c.P["front"]["head_tube_length"] * c.u
        Mh = cross2(Cp - Hm, Ff_)
        rf = rear_forces(c, Fr_, None)
        # deck bending at rear bulkhead station from rear side
        xs = P["deck"]["battery_bay_rear_x"]
        Mdeck = cross2(v2([0.0, 0.0]) - v2([xs, J[1]]), Fr_)
        pr(f"  {lc['name']}")
        pr(f"     front Fx {Ff_[0]:7.0f} Fz {Ff_[1]:6.0f} | rear Fx {Fr_[0]:6.0f} Fz {Fr_[1]:6.0f}"
           + (f" | lateral {lc['lat']:.2f} Fz" if lc["lat"] else "") + (f"  ({lc['note']})" if lc["note"] else ""))
        pr(f"     head bearings: upper {Fu:7.0f}  lower {Fl:7.0f}  axial {Fa:6.0f}")
        pr(f"     neck root @head tube {Mh/1e3:6.0f} N.m -> neck box stress {abs(Mh)/nZ:5.1f} MPa | "
           f"neck/deck junction {Mj/1e3:6.0f} N.m -> deck stress {abs(Mj)*sec['c']/sec['I']:5.1f} MPa")
        pr(f"     rear: push-rod {rf['rod']:7.0f}  shock {rf['shock']:7.0f}  swingarm pivot |{np.linalg.norm(rf['pivot']):6.0f}|"
           f"  rocker pivot |{np.linalg.norm(rf['rocker']):6.0f}|  deck @rear bulkhead {Mdeck/1e3:6.0f} N.m "
           f"({abs(Mdeck)*sec['c']/sec['I']:.0f} MPa)")
    return out, sw


# --------------------------------------------------------------------------
# SVG
# --------------------------------------------------------------------------
def tail_front_x(c):
    return max(c.link[k][0] for k in ("R", "B", "C")) + 40.0


def svg(c, sw, path):
    P = c.P
    S = 0.6
    X0, Z0 = 190, 720
    Wd, Hd = 1200, 960

    def T(p):
        return X0 + p[0] * S, Z0 - p[1] * S

    el = []
    add = el.append

    def line(a, b, col="#222", w=2, dash=None):
        (x1, y1), (x2, y2) = T(a), T(b)
        d = f' stroke-dasharray="{dash}"' if dash else ""
        add(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="{col}" stroke-width="{w}"{d}/>')

    def circ(p, r, col="#222", w=2, fill="none", dash=None):
        x, y = T(p)
        d = f' stroke-dasharray="{dash}"' if dash else ""
        add(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{r*S:.1f}" stroke="{col}" stroke-width="{w}" fill="{fill}"{d}/>')

    def poly(pts, col="#222", fill="none", w=2, op=1.0):
        s = " ".join(f"{T(p)[0]:.1f},{T(p)[1]:.1f}" for p in pts)
        add(f'<polygon points="{s}" stroke="{col}" fill="{fill}" fill-opacity="{op}" stroke-width="{w}"/>')

    def text(p, s, size=13, col="#222", anchor="start"):
        x, y = T(p)
        add(f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" fill="{col}" text-anchor="{anchor}" '
            f'font-family="sans-serif">{s}</text>')

    f, r, dk = P["front"], P["rear"], P["deck"]
    line([-300, 0], [1650, 0], "#888", 1)
    # rear wheel sag + bottomed ghost
    ax_s = c.axle(c.th0)
    ax_b = c.axle(c.theta_for_axle_z(c.z_bot))
    circ(ax_s, c.R, "#333", 3)
    circ(ax_s, 82.5, "#999", 1)
    circ(ax_s, 80, "#c33", 1.5, dash="4,3")
    circ(ax_b, c.Rg, "#d77", 1.5, dash="6,4")
    circ(ax_b, c.Rg + P["clearance"]["tire_radial_min"], "#f0b0b0", 1, dash="2,4")
    # front wheel sag + bottomed ghost
    circ(c.Af, c.R, "#333", 3)
    circ(c.Af, 82.5, "#999", 1)
    circ(c.Af, 80, "#c33", 1.5, dash="4,3")
    af_b = c.Af + (f["fork_travel_axial"] - c.fsag) * c.u
    circ(af_b, c.Rg, "#d77", 1.5, dash="6,4")
    # steering axis
    line(v2([c.x_g, 0]), c.bar + 60 * c.u, "#36c", 1, "8,4")
    # fork legs + crowns
    for side in (-1, 1):
        o = side * 38 * c.n
        line(c.Af + o * 0 + 15 * c.u, c.crown_up + o, "#555", 7)
    poly([c.crown_lo - 70 * c.n, c.crown_lo + 70 * c.n, c.ht_bot + 70 * c.n, c.ht_bot - 70 * c.n], "#555", "#bbb")
    poly([c.ht_top - 60 * c.n, c.ht_top + 60 * c.n, c.crown_up + 60 * c.n, c.crown_up - 60 * c.n], "#555", "#bbb")
    line(c.crown_up, c.bar, "#444", 8)
    hw = f["head_tube_od"] / 2
    poly([c.ht_bot - hw * c.n, c.ht_bot + hw * c.n, c.ht_top + hw * c.n, c.ht_top - hw * c.n], "#111", "#6a8fbf", 2)
    # deck box & battery
    xr, xf = dk["battery_bay_rear_x"], dk["battery_bay_front_x"]
    gc, dt = c.gc, c.deck_top
    poly([[xr, gc], [xf, gc], [xf, dt], [xr, dt]], "#111", "#d9e4f2")
    b = P["battery"]
    poly([[xr + 14, gc + 8], [xf - 10 - b["end_zone_len"], gc + 8], [xf - 10 - b["end_zone_len"], gc + 8 + b["stack_height_allow"]],
          [xr + 14, gc + 8 + b["stack_height_allow"]]], "#2a7", "#bfe8cf", 1.5)
    text([(xr + xf) / 2 - 60, gc + 50], "21700 cell block (20S7P)", 12, "#185")
    poly([[xf - 8 - b["end_zone_len"], gc + 8], [xf - 12, gc + 8], [xf - 12, dt - 10], [xf - 8 - b["end_zone_len"], dt - 10]],
         "#a60", "#f6deb0", 1.5)
    text([xf - b["end_zone_len"], gc + 60], "BMS/fuse", 10, "#a60")
    text(c.ht_bot + v2([-150, -40]), "controller bay", 10, "#a60")
    # neck
    pts, p0, hb = neck_frame_points(c)
    poly([[xf - 60, dt], p0 if False else [xf, dt - 40], hb, c.ht_top - (hw + 2) * c.n, [xf - 90, dt]], "#111", "#9db4d6")
    # rear bulkhead / pivot block
    poly([[xr - 90, gc + 20], [xr, gc], [xr, dt], [xr - 90, dt]], "#111", "#9db4d6")
    # rear subframe/tail
    L = c.link
    tail = [[xr - 90, dt], [L["R"][0] + 40, L["R"][1] - 40], [L["R"][0] + 30, L["S"][1] + 45],
            [L["S"][0] - 30, L["S"][1] + 45], [L["S"][0] - 30, L["S"][1] - 40], [L["R"][0] - 60, L["R"][1] - 20],
            [xr - 90, dt - 60]]
    poly(tail, "#111", "#9db4d6", 1.5, 0.55)
    text([L["R"][0] - 10, L["S"][1] + 60], "tail / rear subframe", 11)
    # swingarm
    A0 = L["A"]
    poly([c.Pv + v2([0, 22]), ax_s + v2([0, 18]), ax_s - v2([0, 18]), c.Pv - v2([0, 22])], "#111", "#7f99c4")
    circ(c.Pv, 16, "#111", 2, "#fff")
    line(c.Pv, A0, "#111", 6)
    # linkage
    line(A0, L["B"], "#c50", 4)
    poly([L["R"], L["B"], L["C"]], "#c50", "#f2c7a5")
    line(L["C"], L["S"], "#a0a", 14)
    line(L["C"], L["S"], "#f3d4f3", 8)
    for k in ("A", "B", "C", "S", "R"):
        circ(L[k], 6, "#000", 1.5, "#fff")
        text(L[k] + v2([8, 8]), k, 12)
    # CG
    cg = v2([c.x_cg, c.h_cg])
    circ(cg, 14, "#000", 2, "#ffe066")
    text(cg + v2([20, -4]), f"CG GVW {c.m:.0f} kg, h={c.h_cg:.0f}", 12)
    # labels / dims
    text([c.WB / 2, -40], f"wheelbase {c.WB} mm", 14, anchor="middle")
    text([c.x_g + 10, 30], f"trail {c.trail:.0f}", 12, "#36c")
    text(c.ht_top + v2([40, 20]), f"head angle {90-f['rake_deg']:.0f} deg", 12, "#36c")
    text([xr + 20, dt + 18], f"usable deck ~{xf - tail_front_x(c):.0f} mm, deck top z={dt}", 12)
    text([c.Pv[0] - 30, c.Pv[1] - 40], "pivot", 11)
    add('<text x="20" y="940" font-size="13" font-family="sans-serif">Rider aboard at sag. Dashed red: grown tyre at '
        'full compression (dotted = +clearance). Small dashed red: 160 mm rotor. Blue dashed: steering axis.</text>')
    add(f'<text x="20" y="24" font-size="16" font-family="sans-serif" font-weight="bold">SK2V chassis - '
        f'side layout (generated by tools/chassis/chassis_calc.py; 1 px = {1/S:.2f} mm)</text>')
    # LR inset
    if sw is not None:
        gx, gy, gw, gh = 860, 750, 300, 160
        add(f'<rect x="{gx}" y="{gy}" width="{gw}" height="{gh}" fill="#fff" stroke="#999"/>')
        lr = sw["lr"][1:-1]
        wt = sw["wt"][1:-1]
        lo, hi = 1.8, 3.0
        pts_ = " ".join(f"{gx + gw * (w / wt[-1]):.1f},{gy + gh - gh * (l - lo) / (hi - lo):.1f}" for w, l in zip(wt, lr))
        add(f'<polyline points="{pts_}" fill="none" stroke="#a0a" stroke-width="2"/>')
        add(f'<text x="{gx+6}" y="{gy+16}" font-size="12" font-family="sans-serif">rear leverage ratio vs wheel travel '
            f'({lr[0]:.2f} to {lr[-1]:.2f})</text>')
        add(f'<text x="{gx+6}" y="{gy+gh-6}" font-size="11" font-family="sans-serif">0 .. {wt[-1]:.0f} mm; axis {lo}-{hi}</text>')
    body = "\n".join(el)
    doc = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{Wd}" height="{Hd}" viewBox="0 0 {Wd} {Hd}">\n'
           f'<rect width="100%" height="100%" fill="#fbfbf8"/>\n{body}\n</svg>\n')
    with open(path, "w") as fh:
        fh.write(doc)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--params", default=os.path.join(HERE, "chassis_params.json"))
    ap.add_argument("--optimise", action="store_true")
    ap.add_argument("--write", action="store_true")
    ap.add_argument("--svg", default=os.path.join(ROOT, "docs", "chassis", "layout_side.svg"))
    args = ap.parse_args()
    with open(args.params) as fh:
        P = json.load(fh)
    c = Chassis(P)
    lines, sw = report(c, args)
    print("\n".join(lines))
    os.makedirs(os.path.dirname(args.svg), exist_ok=True)
    svg(c, sw, args.svg)
    print(f"\nSVG written: {os.path.relpath(args.svg, ROOT)}")


if __name__ == "__main__":
    main()
