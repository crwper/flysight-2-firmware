#!/usr/bin/env python3
"""Generate a synthetic skydive track in the simulator's simple CSV format.

Columns: t,lat,lon,hMSL,velN,velE,velD,hAcc,vAcc,sAcc,numSV,gpsFix
(seconds; degrees; metres; m/s; velD positive DOWN; SI accuracies)

Profile (all altitudes in metres above dz_elev, i.e. AGL):

    [no-fix]  [ground]  [climb]           [freefall]        [canopy]   [landed]
    gpsFix=0  v=0       climb_rate up     tanh spin-up to   const      v=0
                        to exit_alt       terminal, down    descent
                                          to deploy_alt     to 0 AGL

The generator is fully deterministic (no noise) so tracks regenerate
byte-identically; crossing altitudes for alarms etc. can be computed from
the parameters.

Example:
    python gen_jump.py --out track.csv --exit-alt 4000 --deploy-alt 1000
"""

import argparse
import math
import sys

G = 9.81


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--out", required=True)
    p.add_argument("--dz-elev", type=float, default=0.0, help="ground elevation, m MSL (default 0)")
    p.add_argument("--rate", type=float, default=0.2, help="sample interval, s (default 0.2)")
    p.add_argument("--nofix", type=float, default=0.0, help="seconds of gpsFix=0 rows at start")
    p.add_argument("--ground", type=float, default=10.0, help="seconds stationary on the ground")
    p.add_argument("--climb-rate", type=float, default=5.0, help="m/s climb (default 5)")
    p.add_argument("--exit-alt", type=float, default=4000.0, help="exit altitude AGL, m")
    p.add_argument("--terminal", type=float, default=55.0, help="freefall terminal vertical speed, m/s")
    p.add_argument("--freefall-hspeed", type=float, default=15.0, help="horizontal speed in freefall, m/s")
    p.add_argument("--deploy-alt", type=float, default=1000.0, help="deployment altitude AGL, m")
    p.add_argument("--snivel", type=float, default=4.0, help="seconds to decelerate to canopy speed")
    p.add_argument("--canopy-vspeed", type=float, default=5.0, help="m/s descent under canopy")
    p.add_argument("--canopy-hspeed", type=float, default=10.0, help="m/s forward under canopy")
    p.add_argument("--landed", type=float, default=10.0, help="seconds stationary after landing")
    p.add_argument("--climb-hspeed", type=float, default=40.0, help="aircraft ground speed, m/s")
    p.add_argument("--heading0", type=float, default=0.0, help="initial course, deg (0 = north)")
    p.add_argument("--climb-turn", type=float, default=0.0, help="turn rate during climb, deg/s")
    p.add_argument("--freefall-turn", type=float, default=0.0, help="turn rate in freefall, deg/s")
    p.add_argument("--canopy-turn", type=float, default=0.0, help="turn rate under canopy (and snivel), deg/s")
    p.add_argument("--lat0", type=float, default=51.0442700)
    p.add_argument("--lon0", type=float, default=-114.0620190)
    p.add_argument("--hacc", type=float, default=1.5)
    p.add_argument("--vacc", type=float, default=2.0, help="vertical accuracy, m (<10 sets FLAG_VERTICAL_ACC)")
    p.add_argument("--sacc", type=float, default=0.5)
    p.add_argument("--numsv", type=int, default=14)
    p.add_argument("--vacc-poor", type=float, default=0.0, metavar="S",
                   help="vAcc is 30 m until this many seconds in (then --vacc)")
    p.add_argument("--dropout", default=None, metavar="T0:DUR[:FIX]",
                   help="gpsFix=FIX (default 0) during [T0, T0+DUR) seconds")
    args = p.parse_args()

    dropout = None
    dropout_fix = 0
    if args.dropout:
        parts = args.dropout.split(":")
        t0, dur = float(parts[0]), float(parts[1])
        if len(parts) > 2:
            dropout_fix = int(parts[2])
        dropout = (t0, t0 + dur)

    dt = args.rate
    rows = []

    # Build the jump as (duration, state_fn(t_in_phase) -> (alt, hspeed, velD)).
    # velD positive down; alt in m AGL. The horizontal speed is pointed along
    # the current course (heading0 + integrated per-phase turn rate) when the
    # rows are emitted.

    def phase_ground(t):
        return 0.0, 0.0, 0.0

    climb_dur = args.exit_alt / args.climb_rate

    def phase_climb(t):
        return args.climb_rate * t, args.climb_hspeed, -args.climb_rate

    # freefall: vD(t) = vt*tanh(g*t/vt); alt(t) = exit - (vt^2/g)*ln(cosh(g*t/vt))
    vt = args.terminal

    def ff_alt(t):
        return args.exit_alt - (vt * vt / G) * math.log(math.cosh(G * t / vt))

    def ff_vd(t):
        return vt * math.tanh(G * t / vt)

    # find freefall duration to reach deploy_alt (bisection; alt monotonic)
    lo, hi = 0.0, 600.0
    for _ in range(200):
        mid = (lo + hi) / 2
        if ff_alt(mid) > args.deploy_alt:
            lo = mid
        else:
            hi = mid
    ff_dur = (lo + hi) / 2

    def phase_freefall(t):
        return ff_alt(t), args.freefall_hspeed, ff_vd(t)

    # snivel: linear blend from freefall speeds to canopy speeds
    snivel_v0 = ff_vd(ff_dur)
    snivel_alt0 = ff_alt(ff_dur)

    def snivel_alt(t):
        # integral of linear blend vD(t)
        a = t / args.snivel
        vd = snivel_v0 + (args.canopy_vspeed - snivel_v0) * a
        return snivel_alt0 - t * (snivel_v0 + vd) / 2.0

    def phase_snivel(t):
        a = t / args.snivel
        vd = snivel_v0 + (args.canopy_vspeed - snivel_v0) * a
        h = args.freefall_hspeed + (args.canopy_hspeed - args.freefall_hspeed) * a
        return snivel_alt(t), h, vd

    canopy_alt0 = snivel_alt(args.snivel)
    canopy_dur = canopy_alt0 / args.canopy_vspeed

    def phase_canopy(t):
        return canopy_alt0 - args.canopy_vspeed * t, args.canopy_hspeed, args.canopy_vspeed

    def phase_landed(t):
        return 0.0, 0.0, 0.0

    if args.exit_alt <= 0:
        # ground-only track (e.g. for init / first-fix scenarios)
        climb_dur = ff_dur = canopy_dur = 0.0
        phases = [
            (args.ground, phase_ground, 3, 0.0),
            (args.landed, phase_landed, 3, 0.0),
        ]
    else:
        phases = [
            (args.ground, phase_ground, 3, 0.0),
            (climb_dur, phase_climb, 3, args.climb_turn),
            (ff_dur, phase_freefall, 3, args.freefall_turn),
            (args.snivel, phase_snivel, 3, args.canopy_turn),
            (canopy_dur, phase_canopy, 3, args.canopy_turn),
            (args.landed, phase_landed, 3, 0.0),
        ]
    if args.nofix > 0:
        phases.insert(0, (args.nofix, phase_ground, 0, 0.0))

    # walk the profile at fixed dt, integrating course and lat/lon
    lat, lon = args.lat0, args.lon0
    heading = args.heading0
    t = 0.0
    n = 0
    for dur, fn, fix, turn in phases:
        steps = int(round(dur / dt))
        for i in range(steps):
            alt, h, vd = fn(i * dt)
            psi = math.radians(heading)
            vn = h * math.cos(psi)
            ve = h * math.sin(psi)
            rows.append((t, lat, lon, args.dz_elev + alt, vn, ve, vd, fix))
            # integrate position and course for the next sample
            lat += (vn * dt) / 111320.0
            lon += (ve * dt) / (111320.0 * math.cos(math.radians(lat)))
            heading = (heading + turn * dt) % 360.0
            t = round(t + dt, 6)
            n += 1

    with open(args.out, "w", newline="\n") as f:
        f.write("# generated by gen_jump.py: " + " ".join(sys.argv[1:]) + "\n")
        f.write("t,lat,lon,hMSL,velN,velE,velD,hAcc,vAcc,sAcc,numSV,gpsFix\n")
        for (t, la, lo_, h, vn, ve, vd, fix) in rows:
            vacc = 30.0 if t < args.vacc_poor else args.vacc
            if dropout and dropout[0] <= t < dropout[1]:
                fix = dropout_fix
            f.write(
                f"{t:.1f},{la:.7f},{lo_:.7f},{h:.3f},{vn:.3f},{ve:.3f},{vd:.3f},"
                f"{args.hacc:.3f},{vacc:.3f},{args.sacc:.3f},{args.numsv},{fix}\n"
            )

    total = rows[-1][0] if rows else 0
    print(f"{args.out}: {len(rows)} rows, {total:.1f} s "
          f"(climb {climb_dur:.1f} s, freefall {ff_dur:.1f} s, canopy {canopy_dur:.1f} s)")


if __name__ == "__main__":
    main()
