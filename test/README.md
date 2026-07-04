# FlySight audio-control characterization harness

A host-side simulator that compiles the **unmodified** firmware sources
(`FlySight/audio_control.c`, `config.c`, `nav.c`, `common.c`) against fake
platform layers, replays a GNSS track through them on a deterministic
simulated clock, and records every audible event (beeps, wav playback,
stops) as a text **trace**. Traces are compared byte-for-byte against
checked-in **golden** traces.

Purpose: pin down the exact behaviour of the current audio module with
100% branch coverage, then rewrite the module and require the rewrite to
produce identical traces (modulo deliberate, reviewed changes -- see
`QUIRKS.md`).

## Layout

```text
test/
  CMakeLists.txt      host build (MinGW gcc + Ninja)
  fakes/              fake platform: main.h/app_common.h/stm32_seq.h/ff.h
                      + fake audio driver, sequencer, timer server, GNSS
  sim/                deterministic clock, event loop, trace output
  runner/             audio_sim CLI + track file parser
  scenarios/<name>/   config.txt + track.csv (+ optional opts.txt)
  golden/<name>.trace committed expected outputs
  out/                actual outputs (gitignored)
  tracks/             shared synthetic tracks (copied into scenarios)
  scripts/gen_jump.py synthetic jump-profile generator
  run_tests.py        golden-trace test runner
  QUIRKS.md           log of surprising characterized behaviours
```

## Prerequisites

Already present on this machine: MSYS2 GCC (`C:/msys64/mingw64/bin`),
CMake, Ninja, Python 3, and `gcovr` (`pip install gcovr`).

## Build

```powershell
cd test
cmake -B build -G Ninja -DCMAKE_C_COMPILER=C:/msys64/mingw64/bin/gcc.exe
cmake --build build
```

Coverage instrumentation is on by default (`-DCOVERAGE=OFF` to disable);
it only instruments the firmware sources, not the harness.

## Tutorial

### 1. Run one scenario by hand

```powershell
./build/audio_sim.exe --config scenarios/alarms-freefall/config.txt `
                      --track scenarios/alarms-freefall/track.csv `
                      --audio-dir ../TEMP/AUDIO
```

Output (stdout, or `--trace file`):

```text
# fs-audio-sim v1
# config=config.txt track=track.csv tail=2.000
# rtc_tick_ns=488191
   0.009763  BEEP f0=1760 f1=1760 ms=125 vol=10 hmsl=0.000
 147.600000  PLAY file=base.wav dur=0.820 vol=10 hmsl=1100.800
 ...
 760.200000  END
```

One line per audible event: timestamp (s), event, parameters, and the
current GNSS altitude (`hmsl`, metres MSL) so you can map the event to a
phase of the jump. `(cuts X)` on a line means the new sound preempted a
playing one; `STOP` means the firmware silenced a playing sound; `PLAYFAIL`
means it asked for a wav that doesn't exist in the audio dir (the device
would also play nothing).

### 2. Run the whole suite

```powershell
python run_tests.py                 # PASS/FAIL per scenario, diff on fail
python run_tests.py --filter alarm  # subset
```

A FAIL prints a unified diff of golden vs. actual. While the firmware
sources are untouched, everything should always pass -- a FAIL means the
harness changed (or the build is stale).

### 3. Add a scenario

```powershell
# a track: either generate one...
python scripts/gen_jump.py --out scenarios/my-test/track.csv `
       --exit-alt 4000 --deploy-alt 1000 --climb-rate 8
# ...or copy a real device log (TRACK.CSV) -- $GNSS rows are parsed directly.

# a config: real config.txt syntax (see FlySight docs / config.c)
notepad scenarios/my-test/config.txt

# optional: scenarios/my-test/opts.txt with extra runner args, e.g. --tail 10

python run_tests.py --filter my-test           # reports NEW
./build/audio_sim.exe --config ... --track ... # READ the trace first!
python run_tests.py --filter my-test --bless   # accept as golden
```

**Rule: never bless a golden you haven't read.** A golden pins whatever
the code currently does, including bugs. If the trace surprises you, add
an entry to `QUIRKS.md` (preserve vs. fix is decided later, at rewrite
time).

### 4. The coverage loop -- getting to 100% branch

```powershell
# 1. zero the counters (gcda files ACCUMULATE across every run of the exe)
Get-ChildItem -Recurse build -Filter *.gcda | Remove-Item

# 2. run the suite once
python run_tests.py

# 3. report -- lines and branches
python -m gcovr -r .. build --filter ".*audio_control"                     # lines
python -m gcovr -r .. build --filter ".*audio_control" --txt-metric branch # branches

# or a browsable annotated source view:
mkdir -Force coverage | Out-Null
python -m gcovr -r .. build --filter ".*audio_control" --html-details coverage/index.html
start coverage/index.html
```

Current baseline with the 5 seed scenarios: **52% lines / 45% branches**.

The `Missing` column (and the red lines in the HTML view) is your work
list. The loop is:

1. Pick a red region and read it. Ask: *what config + track reaches this?*
   Examples from the current baseline:
   - `getValues` modes 0/1/3/4/11 red -> scenarios with `Mode: 0`, `1`,
     `3`, `4`, `11` (the seeds only use glide ratio, `Mode: 2`).
   - `setTone` limit branches (lines ~134-175) red -> configs with
     `Limits: 0/2/3` and Min/Max chosen so the value goes out of range;
     `Flatline: 1`; inverted ranges (`Min > Max`).
   - SAS table edges -> a track with hMSL < 0 (`--dz-elev -50`) and one
     above 11,264 m.
   - `speakValue` modes -> one scenario per `Sp_Mode`, both `Sp_Units`,
     `Sp_Dec` 0/1/2; `num_speech > 1` for the label branches (`>`, then
     horz/vert/glide wavs in `consumerTask`).
   - Nav modes (5/6/7/10) -> configs with `Lat`/`Lon`/`Bearing`/`End_Nav`/
     `Max_Dist`/`Min_Angle` plus a course: gen_jump takes `--heading0` and
     per-phase turn rates (`--climb-turn`/`--freefall-turn`/`--canopy-turn`,
     deg/s). A turning canopy sweeps the relative bearing through all
     angles on a known period (3 deg/s -> 120 s), so deadband silences and
     left/right transitions land at predictable times -- see the
     `nav-direction` scenario for the geometry recipe.
2. Write the scenario, read its trace, log surprises in QUIRKS.md, bless.
3. Re-run the coverage steps. Repeat until nothing is red.

Two rules keep the suite valid for the rewrite:

- **Only drive behaviour through config + track + time.** Never add test
  hooks to audio_control.c; any branch you can't reach from the outside is
  a finding (dead code or a bug), not a reason for a hook.
- If a branch needs an odd asset (e.g. a wav the device might lack),
  point `--audio-dir` at a scenario-local directory via `opts.txt`.

When gcovr shows 100% branch, run the **mutation pass**:

```powershell
python scripts/mutation_test.py            # ~6 min: 30 mutants x rebuild x suite
python scripts/mutation_test.py --only M03 # re-check specific mutants
```

It applies 30 hand-picked mutations to `audio_control.c` one at a time
(flipped crossing bounds, dropped `dz_elev`, wrong constants, boundary
`>=` -> `>`), rebuilds, and requires at least one scenario to FAIL; the
source is restored afterwards (verify with `git status FlySight/`). A
SURVIVED mutant means a behaviour change the goldens cannot see -- add a
scenario that kills it (that is how `alarm-dz-elev-asym`, `fix-degraded`
and `alt-step-teens` came to exist). Status: all 30 mutants killed.
When the rewrite adds scenarios, extend `MUTANTS` in the script with any
new logic worth stress-testing.

### 5. The rewrite

Keep `FlySight/audio_control.c` frozen as the oracle. Develop the rewrite
in a new file (e.g. `FlySight/audio_control2.c`) exporting the same three
entry points, add a second CMake target (`audio_sim2`) that links it in
place of the original, and give `run_tests.py` an `--exe build/audio_sim2.exe`
argument. Every scenario must produce identical traces; every difference
is either a rewrite bug or a deliberate QUIRKS.md fix with a reviewed
golden update. For extra assurance, drive both executables with random
gen_jump parameters and diff (differential fuzzing) -- no goldens needed
while the oracle still builds.

## Simulation model (what the traces mean)

- **Clock**: integer nanoseconds. The timer server ticks at the target's
  real rate (RTC tick = 16/32774 s ~= 488191 ns), so the "10 ms" consumer
  timer actually fires every 20 ticks = 9.763820 ms, matching hardware.
- **Ordering**: earliest event first; ties go to the timer over a GNSS
  sample; after every event all pending sequencer tasks run to completion
  in ascending task-ID order (producer before consumer). This is one
  fixed, documented interleaving of what on hardware varies by a few ms.
- **Audio**: mirrors `audio.c` semantics -- new sounds preempt, beeps last
  exactly `ms`, wav durations are read from the real assets' headers
  (data bytes / 2 samples at 24 kHz), missing files play nothing, idle
  begins the instant the last sample ends.
- **GNSS**: `track.csv` rows are delivered at their timestamps; the
  producer task reads the latest sample, as on target.

Not modelled: ISR preemption races, sequencer latency (assumed zero),
audio buffer underruns, SD-card latency, volume-register I2C traffic,
`FS_Audio_PlayList` (unused by audio_control). Hardware listen-tests
remain the final gate before release.

## Track file formats

Real device logs work as-is: `$GNSS,time,lat,lon,hMSL,velN,velE,velD,
hAcc,vAcc,sAcc,numSV` rows are replayed with timestamps relative to the
first row (other `$` sentences are skipped, fix assumed 3D). Synthetic
tracks use `t,lat,lon,hMSL,velN,velE,velD,hAcc,vAcc,sAcc,numSV,gpsFix`
with `t` in seconds (see `scripts/gen_jump.py`). `speed`, `gSpeed` and
`heading` are derived from the velocity components in both cases.
