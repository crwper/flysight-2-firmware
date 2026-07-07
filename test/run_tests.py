#!/usr/bin/env python3
"""Golden-trace test runner for the FlySight audio simulator.

Each scenario is a directory under scenarios/ containing:
    config.txt   device configuration (real config-file syntax)
    track.csv    GNSS track (synthetic simple CSV or real device TRACK.CSV)
    opts.txt     optional: extra audio_sim arguments, one per line
                 (e.g. "--tail 10"), and/or a "track=<path>" line naming
                 a shared track (relative to test/, e.g.
                 "track=tracks/jump.csv") instead of a local track.csv

For each scenario the simulator writes out/<name>.trace, which is compared
byte-for-byte (after newline normalization) against golden/<name>.trace.

    python run_tests.py               run everything, report PASS/FAIL/NEW
    python run_tests.py --filter alarm    run matching scenarios only
    python run_tests.py --bless      accept current output as golden
    python run_tests.py --bless --filter <name>   bless one scenario

Exit code 0 iff every scenario passed (or was blessed).
"""

import argparse
import difflib
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent


def normalize(data: bytes) -> bytes:
    return data.replace(b"\r\n", b"\n")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--exe", default=str(HERE / "build" / "audio_sim.exe"))
    p.add_argument("--audio-dir", default=str(HERE.parent / "TEMP" / "AUDIO"))
    p.add_argument("--bless", action="store_true", help="accept output as golden")
    p.add_argument("--filter", default="", help="substring match on scenario name")
    p.add_argument("--diff-lines", type=int, default=40, help="max diff lines to show")
    args = p.parse_args()

    # Windows CreateProcess won't resolve a RELATIVE program path with
    # forward slashes (e.g. --exe build/audio_sim.exe); absolutize it.
    args.exe = str(Path(args.exe).resolve())

    scenarios_dir = HERE / "scenarios"
    golden_dir = HERE / "golden"
    out_dir = HERE / "out"
    out_dir.mkdir(exist_ok=True)
    golden_dir.mkdir(exist_ok=True)

    scenarios = sorted(
        d for d in scenarios_dir.iterdir()
        if d.is_dir() and (d / "config.txt").exists() and args.filter in d.name
    )
    if not scenarios:
        print(f"no scenarios matched '{args.filter}'")
        return 1

    n_pass = n_fail = n_new = n_bless = 0

    for sc in scenarios:
        name = sc.name
        trace_path = out_dir / f"{name}.trace"
        golden_path = golden_dir / f"{name}.trace"

        track_path = sc / "track.csv"
        extra_args = []
        opts = sc / "opts.txt"
        if opts.exists():
            for line in opts.read_text().splitlines():
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                if line.startswith("track="):
                    track_path = HERE / line[len("track="):].strip()
                else:
                    extra_args += line.split()

        cmd = [
            args.exe,
            "--config", str(sc / "config.txt"),
            "--track", str(track_path),
            "--audio-dir", args.audio_dir,
            "--trace", str(trace_path),
        ] + extra_args

        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            print(f"FAIL  {name}  (simulator exited {r.returncode})")
            sys.stderr.write(r.stderr)
            n_fail += 1
            continue

        got = normalize(trace_path.read_bytes())

        if args.bless:
            golden_path.write_bytes(got)
            print(f"BLESS {name}  ({len(got.splitlines())} lines)")
            n_bless += 1
            continue

        if not golden_path.exists():
            print(f"NEW   {name}  (no golden; run with --bless to accept)")
            n_new += 1
            continue

        want = normalize(golden_path.read_bytes())
        if got == want:
            print(f"PASS  {name}")
            n_pass += 1
        else:
            print(f"FAIL  {name}")
            diff = difflib.unified_diff(
                want.decode(errors="replace").splitlines(),
                got.decode(errors="replace").splitlines(),
                fromfile=f"golden/{name}.trace",
                tofile=f"out/{name}.trace",
                lineterm="",
            )
            for i, line in enumerate(diff):
                if i >= args.diff_lines:
                    print("  ... (diff truncated)")
                    break
                print("  " + line)
            n_fail += 1

    print()
    if args.bless:
        print(f"{n_bless} blessed")
        return 0
    print(f"{n_pass} passed, {n_fail} failed, {n_new} new")
    return 0 if (n_fail == 0 and n_new == 0) else 1


if __name__ == "__main__":
    sys.exit(main())
