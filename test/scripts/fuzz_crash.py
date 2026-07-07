#!/usr/bin/env python3
"""Crash-only fuzzer for the audio module (card C1).

History: this started as `fuzz_diff.py`, a *differential* fuzzer that ran the
live simulator (audio_sim) alongside a frozen-oracle simulator (audio_sim_ref)
built from the pre-rewrite audio_control.c and byte-compared their traces. That
served Phase A, where the rewrite had to stay byte-identical to the original.
From B1 onward the metrics moved to SI floats, so the frozen integer oracle
diverges from the new goldens on every rounding boundary BY DESIGN, and the
differential comparison stopped being a useful gate. At C1 the reference and the
audio_sim_ref target were retired, so this tool was converted to crash-only.

Each iteration derives, from a single seeded PRNG, a random skydive track (via
gen_jump.py) and a random-but-parser-valid device config, then runs audio_sim on
them and checks the exit code. A non-zero exit is a crash (e.g. a host integer
divide-by-zero, exit 0xC0000094) -- with the goldens frozen this is the cheapest
way to hunt for a config/track combination that faults the module. The known
mode-12 `Sp_Dec 0` divide-by-zero was fixed in B2 (the builder now skips instead
of dividing), so a crash here would be a NEW robustness defect.

Determinism: everything an iteration does is derived from random.Random(seed) in
a fixed draw order, so `--seed N` reproduces iteration N byte-for-byte.

Usage (from test/):
    python scripts/fuzz_crash.py --minutes 5
    python scripts/fuzz_crash.py --iterations 200
    python scripts/fuzz_crash.py --seed 137        # reproduce one iteration
"""

import argparse
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

TEST = Path(__file__).resolve().parent.parent
GEN_JUMP = TEST / "scripts" / "gen_jump.py"

# Parser-valid tone modes (config.c: Mode is 0..7 or 11; Mode_2 is 0..9 or 11)
MODE_SET = [0, 1, 2, 3, 4, 5, 6, 7, 11]
MODE2_SET = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 11]
# Speech modes the renderer knows how to say (config.h FS_CONFIG_MODE_*)
SPEECH_MODE_SET = [0, 1, 2, 3, 4, 5, 6, 7, 11, 12]


def rand_config(rng):
    """Build config_text from rng. Draw order is fixed for reproducibility."""
    L = []
    L.append("; fuzz_crash generated config")
    L.append("Model:     7")

    # Track sample interval -> measurement rate (ms), clamped to parser range.
    gen_rate = round(rng.uniform(0.1, 1.0), 3)
    rate_ms = min(1000, max(40, int(round(gen_rate * 1000))))
    L.append("Rate:      %d" % rate_ms)

    # Tone (Value 1)
    L.append("Mode:      %d" % rng.choice(MODE_SET))
    L.append("Min:       %d" % rng.randint(0, 600))   # Min/Max may end up
    L.append("Max:       %d" % rng.randint(0, 600))   # inverted or equal
    L.append("Limits:    %d" % rng.randint(0, 3))
    L.append("Volume:    6")

    # Rate (Value 2)
    L.append("Mode_2:    %d" % rng.choice(MODE2_SET))
    L.append("Min_Val_2: %d" % rng.randint(0, 1500))
    L.append("Max_Val_2: %d" % rng.randint(0, 1500))
    L.append("Min_Rate:  100")
    L.append("Max_Rate:  500")
    L.append("Flatline:  %d" % rng.randint(0, 1))

    # Speech: 0..3 entries (FS_CONFIG_MAX_SPEECH). Sp_Dec 0 is drawn on
    # purpose so mode-12 Sp_Dec-0 (the QUIRKS #19 divide-by-zero site, fixed
    # in B2) keeps getting exercised.
    num_speech = rng.randint(0, 3)
    L.append("Sp_Rate:   %d" % (rng.randint(1, 3) if num_speech else 0))
    L.append("Sp_Volume: 6")
    for _ in range(num_speech):
        L.append("Sp_Mode:   %d" % rng.choice(SPEECH_MODE_SET))
        L.append("Sp_Units:  %d" % rng.randint(0, 2))
        L.append("Sp_Dec:    %d" % rng.randint(0, 3))

    # Thresholds (cm/s)
    L.append("V_Thresh:  %d" % rng.randint(0, 2000))
    L.append("H_Thresh:  %d" % rng.randint(0, 2000))

    L.append("Use_SAS:   %d" % rng.randint(0, 1))

    # Ground elevation (m MSL): matched to the track so alarms/altitude land.
    dz_elev = rng.randint(-100, 2500)
    L.append("DZ_Elev:   %d" % dz_elev)

    # Altitude mode
    L.append("Alt_Units: %d" % rng.randint(0, 1))
    L.append("Alt_Step:  %d" % rng.randint(0, 2000))

    # Alarms: 0..20 (FS_CONFIG_MAX_ALARMS), elevations AGL, types 0..4
    num_alarms = rng.randint(0, 20)
    for _ in range(num_alarms):
        L.append("Alarm_Elev: %d" % rng.randint(0, 6000))
        L.append("Alarm_Type: %d" % rng.randint(0, 4))
        L.append("Alarm_File: 0")

    # Silence windows: 0..2 (FS_CONFIG_MAX_WINDOWS)
    num_windows = rng.randint(0, 2)
    for _ in range(num_windows):
        bottom = rng.randint(0, 4000)
        top = bottom + rng.randint(0, 2000)
        L.append("Win_Top:    %d" % top)
        L.append("Win_Bottom: %d" % bottom)

    # Nav keys
    lat = rng.randint(509900000, 510990000)
    lon = rng.randint(-1141200000, -1140100000)
    bearing = rng.randint(0, 360)
    end_nav = rng.randint(0, 2000)
    max_dist = rng.randint(0, 10000)
    min_angle = rng.randint(0, 360)
    L.append("Lat:       %d" % lat)
    L.append("Lon:       %d" % lon)
    L.append("Bearing:   %d" % bearing)
    L.append("End_Nav:   %d" % end_nav)
    L.append("Max_Dist:  %d" % max_dist)
    L.append("Min_Angle: %d" % min_angle)

    meta = {"gen_rate": gen_rate, "dz_elev": dz_elev}
    return "\n".join(L) + "\n", meta


def gen_jump_args(rng, meta, out_path):
    """Derive gen_jump.py parameters from rng (fixed draw order)."""
    exit_alt = rng.randint(800, 8000)
    terminal = rng.randint(35, 90)
    deploy_alt = rng.randint(400, min(1500, exit_alt - 100))
    climb_rate = rng.randint(3, 20)
    climb_turn = round(rng.uniform(0, 6), 2)
    freefall_turn = round(rng.uniform(0, 6), 2)
    canopy_turn = round(rng.uniform(0, 6), 2)
    return [
        sys.executable, str(GEN_JUMP),
        "--out", str(out_path),
        "--dz-elev", str(meta["dz_elev"]),
        "--rate", str(meta["gen_rate"]),
        "--exit-alt", str(exit_alt),
        "--terminal", str(terminal),
        "--deploy-alt", str(deploy_alt),
        "--climb-rate", str(climb_rate),
        "--climb-turn", str(climb_turn),
        "--freefall-turn", str(freefall_turn),
        "--canopy-turn", str(canopy_turn),
    ]


def run_iteration(seed, live_exe, audio_dir, failures_dir):
    """Return 'ok', 'crash', or 'error'. A non-zero simulator exit is a crash."""
    import random
    rng = random.Random(seed)

    config_text, meta = rand_config(rng)
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        config_path = tmp / "config.txt"
        track_path = tmp / "track.csv"
        trace_path = tmp / "out.trace"
        config_path.write_text(config_text, newline="\n")

        gj = gen_jump_args(rng, meta, track_path)
        r = subprocess.run(gj, capture_output=True, text=True)
        if r.returncode != 0 or not track_path.exists():
            _save(failures_dir, seed, config_path, track_path, meta, gj,
                  "gen_jump failed (harness bug):\n" + r.stderr)
            return "error"

        rr = subprocess.run(
            [str(live_exe), "--config", str(config_path), "--track",
             str(track_path), "--audio-dir", str(audio_dir),
             "--trace", str(trace_path)],
            capture_output=True, text=True)

        if rr.returncode != 0:
            _save(failures_dir, seed, config_path, track_path, meta, gj,
                  "audio_sim exit=%s (0x%08X)\nstderr:\n%s"
                  % (rr.returncode, rr.returncode & 0xFFFFFFFF, rr.stderr))
            return "crash"
        return "ok"


def _save(failures_dir, seed, config_path, track_path, meta, gj, detail):
    dst = failures_dir / str(seed)
    dst.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(config_path, dst / "config.txt")
    if track_path.exists():
        shutil.copyfile(track_path, dst / "track.csv")
    (dst / "repro.txt").write_text(
        "seed: %d\n" % seed
        + "reproduce: python scripts/fuzz_crash.py --seed %d\n" % seed
        + "gen_jump: " + " ".join(gj[1:]) + "\n"
        + "meta: %r\n\n" % meta
        + detail + "\n", newline="\n")


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--seed", type=int, default=None,
                   help="reproduce a single iteration by seed")
    p.add_argument("--iterations", type=int, default=None,
                   help="number of iterations to run")
    p.add_argument("--minutes", type=float, default=None,
                   help="time-box the run (minutes)")
    p.add_argument("--exe", default=str(TEST / "build" / "audio_sim.exe"),
                   help="simulator under test")
    p.add_argument("--audio-dir", default=str(TEST.parent / "TEMP" / "AUDIO"))
    args = p.parse_args()

    live_exe = Path(args.exe)
    if not live_exe.exists():
        print("error: build audio_sim.exe first (cmake --build build)")
        return 2

    failures_dir = TEST / "fuzz-failures"

    # Default budget when the user names none: 100 iterations.
    if args.seed is None and args.iterations is None and args.minutes is None:
        args.iterations = 100

    n = 0
    ok = crashes = errors = 0
    start = time.time()
    seed = args.seed if args.seed is not None else 0

    while True:
        res = run_iteration(seed, live_exe, args.audio_dir, failures_dir)
        n += 1
        if res == "ok":
            ok += 1
        elif res == "crash":
            crashes += 1
            print("CRASH  seed=%d  (saved to fuzz-failures/%d)" % (seed, seed))
        else:
            errors += 1
            print("ERROR  seed=%d  (saved to fuzz-failures/%d)" % (seed, seed))

        seed += 1
        if args.seed is not None:
            break
        if args.iterations is not None and n >= args.iterations:
            break
        if args.minutes is not None and (time.time() - start) >= args.minutes * 60:
            break

    print("\nfuzz_crash: %d iterations, %d ok, %d crash, %d error in %.1fs"
          % (n, ok, crashes, errors, time.time() - start))
    return 1 if (crashes or errors) else 0


if __name__ == "__main__":
    sys.exit(main())
