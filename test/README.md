# FlySight audio-control characterization harness

A host-side simulator that compiles the **unmodified** firmware sources
(`FlySight/audio_control.c`, `audio_speech.c`, `flight_params.c`,
`config.c`, `nav.c`, `common.c`) against fake platform layers, replays a
GNSS track through them on a deterministic simulated clock, and records
every audible event (beeps, wav playback, stops) as a text **trace**.
Traces are compared byte-for-byte against checked-in **golden** traces.

Purpose (original): pin down the exact behaviour of the audio module with
100% branch coverage so it could be rewritten safely.

**Status: the rewrite is complete** (the `audio_rewrite` branch, cards
A0-C1 -- see `plan/audio-rewrite/`). The single 1000-line `audio_control.c`
was split into three files (metrics / speech / control) and its behaviour
changed only where a `QUIRKS.md` row says so, each backed by a reviewed
golden update. The harness is now the **regression net** for that module:
the goldens are the contract, and any unexplained trace diff is a bug.
55 scenarios, 100% reachable line + branch coverage of the three rewritten
files (see the residue notes under "The coverage loop"), 33 mutants killed.

## Layout

```text
test/
  CMakeLists.txt      host build (MinGW gcc + Ninja)
  fakes/              fake platform: main.h/app_common.h/stm32_seq.h/ff.h
                      + fake audio driver, sequencer, timer server, GNSS
  sim/                deterministic clock, event loop, trace output
  runner/             audio_sim CLI + track file parser
  scenarios/<name>/   config.txt + track.csv, or a "track=tracks/X.csv"
                      line in opts.txt for the shared tracks (+ other opts)
  golden/<name>.trace committed expected outputs
  out/                actual outputs (gitignored)
  tracks/             shared synthetic tracks (copied into scenarios)
  scripts/gen_jump.py synthetic jump-profile generator
  scripts/mutation_test.py  hand-picked mutants x rebuild x suite
  scripts/fuzz_crash.py     crash-only random-input fuzzer (was fuzz_diff.py)
  run_tests.py        golden-trace test runner
  QUIRKS.md           decision register (every characterized behaviour + its
                      PRESERVE / DONE / asset-pending outcome)
```

## The rewritten module (what lives where)

The rewrite split the old monolithic `audio_control.c` into three files with
one responsibility each. There are **no file-scope data statics** anywhere in
the module: every piece of mutable state lives in one `static
FS_AudioControl_State_t state;` in `audio_control.c`, and the metrics / speech
layers are pure (state passed in by pointer). Config is read-only end to end.

- **`flight_params.c/h` -- the metrics layer.** Pure functions, no state.
  `FS_FlightData_t` (SI units: `float` altitude/speeds/angles, `double`
  lat/lon; `valid3d`/`vAccGood` flags) is built in exactly one place,
  `FS_FlightData_FromGNSS`. `FS_FlightParams_Get*` turn a config `Mode` +
  the current sample into a metric value (speeds, glide, direction/bearing,
  distance, dive, altitude), applying SAS, the direction fold, distance clamp
  and the End_Nav/Max_Dist nav gates. The tone RATE channel and the SAS
  interpolation deliberately stay **integer** (QUIRKS #5, ratified permanent):
  a float rate would drift the 16-bit `0x10000` tone-rate accumulator.

- **`audio_speech.c/h` -- the utterance builder + renderer.** An utterance is
  an array of `FS_SpeechToken_t` enum tokens (`TOK_NUM_*`, `TOK_TENS_*`,
  `TOK_HUNDRED/THOUSAND`, `TOK_MINUS/DOT`, `TOK_UNIT_*`, `TOK_LABEL_*`,
  `TOK_SUFFIX_LEFT/RIGHT`, `TOK_RAW_FILE`) plus one token->filename table
  (`token_files[]`). `FS_Speech_Build*` fill the queue (value, alt-step,
  ground-elev, init digits, init file); `FS_Speech_PlayNext` returns the next
  wav filename and advances. The renderer starts **one wav per idle consumer
  tick**, the same cadence as the old char interpreter. A value that cannot be
  produced (division guard, nav gate, unknown mode) clears `valid` and emits an
  empty queue -- the whole utterance is skipped (QUIRKS #12/#13/#19).

- **`audio_control.c` -- sources + arbiter + glue.** Each *source* is a struct
  plus an update function that owns its private state and emits a request:
  `ToneSource` (change history, `ToneSpec`), `AlarmSource` (filtered alarm
  list, crossing scan), `AltModeSource` (step tracking + `say_altitude`
  ground-elev latch), `SpeechSource` (rotation + period), and the startup
  first-fix beep / init speech. The **arbiter** is the *only* caller of
  `FS_Audio_*`. Priority (high to low): `ALARM > STARTUP > ALT_GROUND_ELEV >
  ALT_STEP > SPEECH > TONE`. Suppression zones OR a channel bitmask into a
  shared mask: alarm windows suppress `{TONE}`; silence windows suppress
  `{TONE, ALT_STEP}`; active speech suppresses `{TONE}` (the old `tone_hold`).
  Entering a zone stops the playing sound; alarms are never suppressed and
  never gated on vAcc. The producer task (GNSS rate) writes the inactive
  `ToneSpec` slot and publishes it to the consumer-timer ISR with a single
  aligned 32-bit index flip + compiler barrier (A6 -- the one part the sim
  cannot exercise; PENDING hardware listen-test -- see the handoff
  checklist at the end of plan/audio-rewrite/JOURNAL.md).

### How to add a new audio source

The arbiter/source split is the payoff of the rewrite: a new sound is a
self-contained struct + update function + one arbiter row, with no new
file-scope state and no new `FS_Audio_*` call site.

1. **State struct.** Add a `MyThingSource_t` (its private fields: history,
   latches, one-shot request flags) and a `MyThingSource_t mything;` member to
   `FS_AudioControl_State_t`. Do **not** add a bare static -- the whole point
   is that `FS_AudioControl_Init`'s single `memset(&state, 0, ...)` resets
   everything, and there is exactly one `state` instance.
2. **Update function.** `static void MyThingSource_Update(const
   FS_Config_Data_t *config, const FS_FlightData_t *fd, const FS_GNSS_Data_t
   *current, ...)` reads config (const!) + the sample + its own state and
   **computes a request** -- e.g. a fire index, a `want_*` flag, or a
   `ToneSpec` -- rather than calling the driver. Call it from `producerTask`
   in the correct slot of the per-sample order (the alarm-window scan must run
   *before* the shared rising-edge stop; the fire/announce *after* it -- see
   the A4 note in the journal for why).
3. **Arbiter priority row.** Add an `AudioChannel_t` entry at the right
   priority and have the arbiter act on the request in that order. If two
   sources can collide on one sample, the earlier `CH_*` wins.
4. **Suppression.** If your sound should silence lower channels while it is
   active, add a `SuppressionZone_t` and its channel bitmask to
   `suppression_table`; sources OR the table entry into the shared
   `suppress_mask`. If your sound should itself be silenced inside existing
   zones, add its channel bit to those zones' masks.
5. Add a scenario that exercises it (see "Add a scenario"), read the trace,
   bless it, and add a mutant to `mutation_test.py` for the new logic.

### How to add a speech token

1. **Enum.** Add `TOK_MYWORD` to `FS_SpeechToken_t` in `audio_speech.h` (before
   `FS_SPEECH_TOKEN_COUNT`).
2. **Filename table.** Add `[TOK_MYWORD] = "myword.wav",` to `token_files[]` in
   `audio_speech.c`. (Ship `myword.wav` in the audio pack; a missing file plays
   as silence/PLAYFAIL, exactly like the still-absent `11.wav` -- QUIRKS #2.)
3. **Builder.** Emit the token from the relevant `FS_Speech_Build*` function
   (or `labelToken()` / the Step-3 suffix switch for a new label/unit). The
   queue is a plain `uint8_t[]`; forward-built utterances append, the
   value encoder writes backward -- mirror the neighbouring code.

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
                      --track tracks/jump.csv `
                      --audio-dir ../TEMP/AUDIO
```

(The scenario's track is named by the `track=` line in its `opts.txt` —
most scenarios share the tracks under `tracks/`; some carry a bespoke
local `track.csv`.)

Output (stdout, or `--trace file`):

```text
# fs-audio-sim v1
# config=config.txt track=jump.csv tail=2.000
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
# a track: reference a shared one (preferred -- single source of truth)...
Add-Content scenarios/my-test/opts.txt "track=tracks/jump.csv"
# ...or generate a bespoke one into the scenario dir...
python scripts/gen_jump.py --out scenarios/my-test/track.csv `
       --exit-alt 4000 --deploy-alt 1000 --climb-rate 8
# ...or copy a real device log (TRACK.CSV) -- $GNSS rows are parsed directly.

# a config: real config.txt syntax (see FlySight docs / config.c)
notepad scenarios/my-test/config.txt

# optional: more opts.txt lines with extra runner args, e.g. --tail 10

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

# 3. report -- lines and branches. The filter spans all three rewritten files.
$F = ".*audio_control|.*flight_params|.*audio_speech"
python -m gcovr -r .. build --filter $F                     # lines
python -m gcovr -r .. build --filter $F --txt-metric branch # branches

# or a browsable annotated source view:
mkdir -Force coverage | Out-Null
python -m gcovr -r .. build --filter $F --html-details coverage/index.html
start coverage/index.html
```

(During Phase A the filter was just `".*audio_control"`; it was widened at
A7 when the metrics/speech code moved into their own files.)

Current state (55 scenarios): **audio_control.c 100% line + 100% branch**;
`audio_speech.c` and `flight_params.c` 100% *reachable*, with a small set of
documented, unreachable-or-exempt residue branches:

- `flight_params.c` -- the reserved `LEFT_RIGHT` (mode 10) metrics entry
  (config.c never accepts mode 10; kept as reserved code, QUIRKS #15) and
  `FS_FlightParams_GetSASCorrectionFactor` (develop-branch-merge scaffolding,
  intentionally unused here, QUIRKS #20b). Both are ratified exempt.
- `audio_speech.c` -- two switch **no-match / default arms** in
  `FS_Speech_BuildValue`'s label + Step-3-suffix switches (`labelToken`'s
  `default`, the Step-3 switch's implicit no-match). They are structurally
  dead: the earlier value switch only marks a value `valid` for modes
  {0-7,11,12} and returns early for anything else, and those exact modes are
  all explicit cases downstream -- so no parser-valid config can reach the
  default arm. Same class as the flight_params switch's own no-match arm; left
  in place because the module is frozen (can't delete the arms) and documented
  rather than covered.

The `Missing` column (and the red lines in the HTML view) is your work
list. The loop is:

1. Pick a red region and read it. Ask: *what config + track reaches this?*
   Examples from the original characterization pass (the function names below
   are the pre-rewrite ones: `getValues`/`speakValue`/`consumerTask` are now
   `FS_FlightParams_Get*`, `FS_Speech_BuildValue`, and the arbiter consumer
   tick -- but the config-knob recipes still apply):
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
python scripts/mutation_test.py            # ~6 min: 33 mutants x rebuild x suite
python scripts/mutation_test.py --only M03 # re-check specific mutants
```

It applies 33 hand-picked mutations one at a time (flipped crossing bounds,
dropped `dz_elev`, wrong constants, boundary `>=` -> `>`, swapped arbiter
priority rows, a dropped suppression-table entry), rebuilds, and requires at
least one scenario to FAIL; the source is restored afterwards (verify with
`git status FlySight/`). Because the module is now three files, each mutant
carries an optional `file` field (`audio_control.c` default, or
`flight_params.c` / `audio_speech.c`) and the harness restores every patched
file per iteration. A SURVIVED mutant means a behaviour change the goldens
cannot see -- add a scenario that kills it (that is how `alarm-dz-elev-asym`,
`fix-degraded`, `alt-step-teens` and the arbiter-collision scenarios came to
exist). Status: all 33 killed, 0 NO-MATCH. Each mutant is anchored to a source
substring, so **if a commit moves the code under a mutant, re-anchor it** (a
NO-MATCH is a stale anchor, not a passing mutant).

### 5. Rewrite complete -- the harness as a regression net

The rewrite is done (branch `audio_rewrite`, plan in `plan/audio-rewrite/`).
The workflow is now the ordinary golden loop: change the module, run
`run_tests.py`, and any trace diff is either a bug or a deliberate change that
you justify and re-bless (reading the whole diff first). Whenever a change adds
or removes an utterance, **trace it through the arbiter suppression rules** --
an added/removed utterance changes `tone_hold`, so tone beeps appear/disappear
in its window; those secondary tone diffs are expected, not bugs.

The Phase-A safety nets are retired:

- **Differential fuzzer / frozen oracle (gone).** Phase A kept a byte-frozen
  copy of the pre-rewrite `audio_control.c` (`test/reference/`) behind an
  `audio_sim_ref` build target, and `fuzz_diff.py` ran both binaries on random
  inputs and byte-compared. Once B1 moved the metrics to SI floats the frozen
  integer oracle diverged on every rounding boundary by design, so the
  comparison stopped being a gate. Both were removed at C1.
- **`fuzz_crash.py` (kept).** `fuzz_diff.py` was converted to a crash-only
  fuzzer: it runs `audio_sim` alone on random parser-valid configs + tracks and
  flags any non-zero exit (`python scripts/fuzz_crash.py --minutes 5`). Useful
  for hunting robustness faults (e.g. a new divide-by-zero) with the goldens
  frozen.

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
