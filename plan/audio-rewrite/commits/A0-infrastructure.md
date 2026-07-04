# A0 — Differential-testing infrastructure

Status: pending

## Goal

Freeze the current implementation as an oracle and add the tooling that
Phase A relies on: a reference sim build and a differential fuzzer.

## Steps

1. Copy `FlySight/audio_control.c` verbatim to
   `test/reference/audio_control_orig.c`. Add a short comment block at
   the top: frozen oracle for the rewrite, do not edit, deleted at C1.
2. In `test/CMakeLists.txt`, add target `audio_sim_ref`: identical to
   `audio_sim` but compiling `test/reference/audio_control_orig.c`
   instead of `${FS_DIR}/audio_control.c`. No coverage instrumentation
   needed on the reference target.
3. Write `test/scripts/fuzz_diff.py`:
   - Loop: seed → derive random gen_jump parameters (exit-alt 800–8000,
     terminal 35–90, deploy-alt 400–1500, dz-elev −100–2500, climb-rate
     3–20, canopy/freefall/climb turn rates 0–6, rate 0.1–1.0) and a
     random config (Mode/Mode_2 from the parser-valid sets, Min/Max
     possibly inverted or equal, Limits 0–3, Flatline, 0–20 alarms with
     random elevations/types 0–4, 0–2 silence windows, 0–3 speech
     entries with random modes/units/decimals, V_Thresh/H_Thresh,
     Alt_Step, Use_SAS, nav keys) → write both to a temp dir → run
     `audio_sim` and `audio_sim_ref` → byte-compare traces.
   - On diff: if the config has a speech entry with mode 5 or 7 AND
     (End_Nav > 0 or Max_Dist > 0), classify as `known-13` (the
     uninitialized-tVal bug makes l/r garbage; the two binaries may
     legitimately disagree) and log to a counter; otherwise save the
     full repro (config, track, both traces, seed) to
     `test/fuzz-failures/<seed>/` and count as a real diff.
   - CLI: `--seed N` (reproduce one), `--minutes M` (time-box),
     `--iterations N`. Print a summary line.
   - Determinism: single `random.Random(seed)` per iteration; derive
     the gen_jump command line and config text from it only.
4. Add `fuzz-failures/` to `test/.gitignore`.

## Allowed files

`test/reference/` (new), `test/CMakeLists.txt`,
`test/scripts/fuzz_diff.py` (new), `test/.gitignore`,
`plan/audio-rewrite/JOURNAL.md`.

## Definition of done

- `cmake --build build` builds both `audio_sim.exe` and
  `audio_sim_ref.exe`.
- `python run_tests.py` → 50 passed, 0 failed, 0 new (unchanged).
- `python run_tests.py --exe build/audio_sim_ref.exe` → 50 passed
  (the reference binary matches the goldens too).
- `python scripts/fuzz_diff.py --minutes 5` → 0 real diffs (both
  binaries are the same code today; this validates the fuzzer itself).
  `known-13` count may be nonzero.

## Stop and report if

- The reference binary fails any golden (would mean the build is
  polluted, e.g. coverage flags leaking into behaviour — they must not).

## Commit message

    Add frozen reference build and differential fuzzer for audio rewrite

## Attempt history

(none)
