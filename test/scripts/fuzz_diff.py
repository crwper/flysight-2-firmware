#!/usr/bin/env python3
"""Differential fuzzer for the audio rewrite (card A0).

Each iteration derives, from a single seeded PRNG, a random skydive track
(via gen_jump.py) and a random-but-parser-valid device config, then runs the
live simulator (audio_sim) and the frozen-oracle simulator (audio_sim_ref)
on identical inputs and byte-compares their traces. While the rewrite is in
progress the two binaries should agree on every input; a disagreement is a
regression.

The one sanctioned exception is QUIRKS #13: when a speech entry uses mode 5
(direction to destination) or 7 (direction to bearing) and the nav gates can
fire (End_Nav > 0 or Max_Dist > 0), speakValue reads an uninitialized tVal
for the l/r suffix, so the two binaries may legitimately disagree on
garbage. Those iterations are classified as `known-13` and only counted, not
saved as failures. Every other diff is saved under fuzz-failures/<seed>/.

Determinism: everything an iteration does is derived from random.Random(seed)
in a fixed draw order, so `--seed N` reproduces iteration N of a plain run
byte-for-byte.

Usage (from test/):
    python scripts/fuzz_diff.py --minutes 5
    python scripts/fuzz_diff.py --iterations 200
    python scripts/fuzz_diff.py --seed 137        # reproduce one iteration
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
    """Build (config_text, meta) from rng. meta carries the fields the
    known-13 classifier needs. Draw order is fixed for reproducibility."""
    L = []
    L.append("; fuzz_diff generated config")
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

    # Speech: 0..3 entries (FS_CONFIG_MAX_SPEECH)
    num_speech = rng.randint(0, 3)
    L.append("Sp_Rate:   %d" % (rng.randint(1, 3) if num_speech else 0))
    L.append("Sp_Volume: 6")
    speech_modes = []
    for _ in range(num_speech):
        m = rng.choice(SPEECH_MODE_SET)
        speech_modes.append(m)
        L.append("Sp_Mode:   %d" % m)
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

    meta = {
        "gen_rate": gen_rate,
        "dz_elev": dz_elev,
        "speech_modes": speech_modes,
        "end_nav": end_nav,
        "max_dist": max_dist,
    }
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


def is_known_13(meta):
    has_57 = any(m in (5, 7) for m in meta["speech_modes"])
    gated = meta["end_nav"] > 0 or meta["max_dist"] > 0
    return has_57 and gated


def normalize(data):
    return data.replace(b"\r\n", b"\n")


def run_iteration(seed, live_exe, ref_exe, audio_dir, failures_dir):
    """Return one of 'match', 'crash', 'known-13', 'diff', 'error'.

    The two binaries agree iff they produce the same (exit code, trace).
    An identical non-zero exit (e.g. both hit the same pre-existing integer
    divide-by-zero) is agreement, not a divergence -- bucketed as 'crash'.
    Only a genuine live-vs-reference difference is a 'diff'."""
    import random
    rng = random.Random(seed)

    config_text, meta = rand_config(rng)
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        config_path = tmp / "config.txt"
        track_path = tmp / "track.csv"
        live_trace = tmp / "live.trace"
        ref_trace = tmp / "ref.trace"
        config_path.write_text(config_text, newline="\n")

        gj = gen_jump_args(rng, meta, track_path)
        r = subprocess.run(gj, capture_output=True, text=True)
        if r.returncode != 0 or not track_path.exists():
            _save(failures_dir, seed, config_path, track_path, None, None,
                  meta, gj, "gen_jump failed (harness bug):\n" + r.stderr)
            return "error"

        def sim(exe, trace):
            return subprocess.run(
                [str(exe), "--config", str(config_path), "--track",
                 str(track_path), "--audio-dir", str(audio_dir),
                 "--trace", str(trace)],
                capture_output=True, text=True)

        rl = sim(live_exe, live_trace)
        rr = sim(ref_exe, ref_trace)

        live = normalize(live_trace.read_bytes()) if live_trace.exists() else b""
        ref = normalize(ref_trace.read_bytes()) if ref_trace.exists() else b""

        if rl.returncode == rr.returncode and live == ref:
            # Identical behaviour, including an identical shared crash.
            return "crash" if rl.returncode != 0 else "match"

        if is_known_13(meta):
            return "known-13"

        import difflib
        diff = ("live exit=%s ref exit=%s\n" % (rl.returncode, rr.returncode)
                + "\n".join(list(difflib.unified_diff(
                    ref.decode(errors="replace").splitlines(),
                    live.decode(errors="replace").splitlines(),
                    fromfile="ref.trace", tofile="live.trace",
                    lineterm=""))[:60]))
        _save(failures_dir, seed, config_path, track_path, live_trace,
              ref_trace, meta, gj, diff)
        return "diff"


def _save(failures_dir, seed, config_path, track_path, live_trace,
          ref_trace, meta, gj, detail):
    dst = failures_dir / str(seed)
    dst.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(config_path, dst / "config.txt")
    if track_path.exists():
        shutil.copyfile(track_path, dst / "track.csv")
    if live_trace and live_trace.exists():
        shutil.copyfile(live_trace, dst / "live.trace")
    if ref_trace and ref_trace.exists():
        shutil.copyfile(ref_trace, dst / "ref.trace")
    (dst / "repro.txt").write_text(
        "seed: %d\n" % seed
        + "reproduce: python scripts/fuzz_diff.py --seed %d\n" % seed
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
                   help="live simulator")
    p.add_argument("--ref-exe", default=str(TEST / "build" / "audio_sim_ref.exe"),
                   help="frozen-oracle simulator")
    p.add_argument("--audio-dir", default=str(TEST.parent / "TEMP" / "AUDIO"))
    args = p.parse_args()

    live_exe = Path(args.exe)
    ref_exe = Path(args.ref_exe)
    if not live_exe.exists() or not ref_exe.exists():
        print("error: build audio_sim.exe and audio_sim_ref.exe first "
              "(cmake --build build)")
        return 2

    failures_dir = TEST / "fuzz-failures"

    # Default budget when the user names none: 100 iterations.
    if args.seed is None and args.iterations is None and args.minutes is None:
        args.iterations = 100

    n = 0
    matched = crashes = known13 = diffs = errors = 0
    start = time.time()
    seed = args.seed if args.seed is not None else 0

    while True:
        res = run_iteration(seed, live_exe, ref_exe, args.audio_dir,
                            failures_dir)
        n += 1
        if res == "match":
            matched += 1
        elif res == "crash":
            crashes += 1
        elif res == "known-13":
            known13 += 1
        elif res == "diff":
            diffs += 1
            print("DIFF   seed=%d  (saved to fuzz-failures/%d)" % (seed, seed))
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

    real = diffs + errors
    print("\nfuzz_diff: %d iterations, %d match, %d shared-crash, %d known-13, "
          "%d real diffs (%d diff, %d error) in %.1fs"
          % (n, matched, crashes, known13, real, diffs, errors,
             time.time() - start))
    return 1 if real else 0


if __name__ == "__main__":
    sys.exit(main())
