# Audio rewrite — shared brief (read by every sub-agent)

You are working on FlySight 2 firmware (STM32WB55, but you build and test
on the Windows host). Branch: `audio_rewrite`. The module being rewritten
is `FlySight/audio_control.c` — tones, speech, altitude announcements and
alarms for skydivers. Users depend on alarms firing at exact altitudes in
competition, so behaviour is preserved **byte-exactly** except where a
decision in `test/QUIRKS.md` says otherwise. Your commit card tells you
which regime you are in.

## 1. The characterization harness (already built; your safety net)

`test/` contains a host-side simulator that compiles the firmware sources
UNMODIFIED against fake platform layers, replays GNSS tracks on a
deterministic clock, and writes every audible event to a text trace.
50 scenarios' traces are committed as goldens and compared byte-for-byte.
Coverage of the current audio_control.c: 100% of reachable lines and
branches. A 30-mutant mutation suite (`test/scripts/mutation_test.py`)
is fully killed.

Commands (Windows, PowerShell, from `test/`):

```powershell
cmake -B build -G Ninja -DCMAKE_C_COMPILER=C:/msys64/mingw64/bin/gcc.exe  # once
cmake --build build
python run_tests.py                     # goal state: "50 passed, 0 failed, 0 new"
python run_tests.py --filter <name>     # subset
python scripts/mutation_test.py         # ~6 min; goal: "30 killed"
```

Wav assets: `TEMP/AUDIO` (untracked). Full harness docs: `test/README.md`.

Facts the goldens depend on (do not change any of this):

- RTC tick = 16/32774 s (modelled 488191 ns); the "10 ms" consumer timer
  is 20 ticks = 9.763820 ms, exactly as on hardware.
- Sim event order: earliest first; ties → timer before GNSS sample; after
  each event, pending sequencer tasks run in ascending ID order
  (producer before consumer).
- Fake audio mirrors `FlySight/audio.c`: new sounds preempt (never
  queue); beeps last exactly `ms`; wav duration = data bytes / 2 samples
  at 24 kHz read from the real file header; missing file ⇒ silence,
  immediately idle; `FS_Audio_Stop` is instant; idle begins the moment
  the last sample ends.

## 2. External contract (frozen)

`FS_AudioControl_Init/DeInit/UpdateGNSS`; producer task at GNSS rate and
consumer task driven by a 20-tick repeated timer via
`UTIL_SEQ_*`/`HW_TS_*`; reads `FS_Config_Get()` (treat as **const**) and
`FS_GNSS_GetData()`; the only outputs are `FS_Audio_Beep/Play/Stop` +
`FS_Audio_IsIdle` polling. `config.c` storage/parsing is untouched
(except card B7).

## 3. Target architecture

```text
FlySight/flight_params.c/h   metrics layer: pure functions, no state
FlySight/audio_speech.c/h    utterance builder + token→wav renderer
FlySight/audio_control.c     sources + arbiter + producer/consumer glue
test/reference/audio_control_orig.c  frozen oracle (created in A0, deleted in C1)
```

Data flow: `GNSS ints → FS_FlightData_t → sources → arbiter → FS_Audio_*`.

- **FS_FlightData_t** (arrives in B1): SI units, `float` for altitude/
  speeds/angles, **`double` for lat/lon**; `valid3d` = (gpsFix == 3);
  `vAccGood` = (vAcc < 10 m). Built in exactly one function. Until B1,
  all internal math stays INTEGER, verbatim from the old code.
- **Metrics**: `FS_FlightParams_Get(mode, ...)` → `{bool valid; value,
  min, max}`. One switch, one SAS implementation, direction fold
  (180−|angle|), distance clamp, End_Nav/Max_Dist gates → valid=false.
  Config integers convert to metric units at this boundary only.
- **Sources** (struct + update fn each; private state; NO file-scope
  statics anywhere):
  ToneSource (owns change-history x0/x1/x2; emits ToneSpec{pitch, chirp,
  rate}); AlarmSource (owns prevHMSL; one-shot beep/play requests; type-0
  alarms filtered at init — from B5); AltModeSource (step tracking +
  `ground_elev_announced` latch; ground-elevation utterance once
  valid3d && vAccGood); SpeechSource (rotation + period); StartupSource
  (init speech/file, first-fix beep).
- **Arbiter** — the ONLY caller of `FS_Audio_*`. Priority table:
  `ALARM > STARTUP > ALT_GROUND_ELEV > ALT_STEP > SPEECH > TONE`.
  Suppression zones: alarm windows suppress {TONE}; silence windows
  suppress {TONE, ALT_STEP}; active speech suppresses {TONE} (the old
  `toneHold`). Entering any zone stops the playing sound. Alarms are
  never suppressed and never gated on vAcc.
- **Speech tokens**: utterance = array of enum tokens (`NUM_0..NUM_19`,
  `TENS_2..TENS_9`, `HUNDRED`, `THOUSAND`, `MINUS`, `DOT`, `UNIT_*`,
  `LABEL_*`, `SUFFIX_LEFT/RIGHT`, `RAW_FILE`) + one token→filename table.
  Renderer starts ONE wav per idle consumer tick (same cadence as the
  old char interpreter). The legacy `o/a/b/c` tokens are NOT carried
  over.
- **Tone handoff** (producer task → consumerTimer ISR): two ToneSpec
  slots + `volatile uint32_t active` index (single aligned 32-bit store
  is atomic on Cortex-M4). The sim cannot test this; it is verified by
  review and eventual hardware listen-test.

Must NOT change, ever: the 20-tick consumer period; the 16-bit tone-rate
accumulator (`0x10000` wrap) and `FS_CONFIG_RATE_FLATLINE` semantics;
125 ms beeps; driver preemption semantics; the alarm-crossing spec
(QUIRKS #8: "fires on the first sample pair whose [low, high) interval
contains the alarm elevation"); the public entry points; anything under
`test/fakes/` or `test/sim/`.

## 4. Known bugs in the CURRENT code (fixed only at their Phase B card)

- QUIRKS #13: `speakValue` reads uninitialized `tVal` for the l/r suffix
  when modes 5/7 are gated. Goldens `speech-nav` / `speech-nav-gated`
  contain garbage-dependent (but per-binary deterministic) output. In
  Phase A you must PRESERVE this observable behaviour; if a golden for
  these two scenarios flips l/r after a pure refactor, the stack layout
  changed — see your card's stop conditions.
- QUIRKS #16: Mode_2==7 recalc passes raw deg×1e5 heading to
  `calcRelBearing` (expects degrees). Preserve until B3.
- QUIRKS #9: config.c parses repeated-group keys into index −1 when the
  group opener is missing. Fixed in B7 only.
- `11.wav` missing from assets ("eleven" is silently skipped —
  PLAYFAIL). Asset fix is Michael's; not your problem.

## 5. Decision register (canonical: test/QUIRKS.md — read it too)

PRESERVE: #1 climb alarms; #3 first-fix beep waits for empty speech
queue; #4 ground-elevation announcement; #5 20-tick period; #8 crossing
interval spec; #11 threshold-gated direction-agnostic step
announcements; #15 keep LEFT_RIGHT (mode 10) code as reserved.
FIX at the named card: #6 state reset (A1); #7 const config (A7);
#12+#13 skip whole utterance when value uncomputable or gated (B2);
#14 drop o/a/b/c token routes (A3, confirmed B6); #16 heading scale
(B3); #17 only ALTITUDE indications wait for the ground-elevation
announcement, other speech unaffected, alarms unconditional (B4);
#18 Alarm_Type 0 ignored completely (B5); #9 parser (B7).
#10 window-edge interval convention: implementer's choice for simplest
code; exact-edge golden diffs acceptable in Phase B with review.

Numbers: `float` SI internally, `double` lat/lon, conversions only at
the boundaries — but ONLY from card B1 onward.

## 6. Working agreements

- Goldens are the contract. In Phase A a trace diff means your change is
  wrong — no exceptions, no blessing.
- Read any trace before blessing it (Phase B), and explain every changed
  line in the commit message.
- Keep the code style of the file you are in (tabs, brace style,
  FS_-prefixed naming, sparse comments explaining constraints only).
- Journal discipline: one entry per commit attempt, format in
  JOURNAL.md. Record deviations, surprises, and anything the next agent
  needs.
- When a card conflicts with reality, STOP and report; do not improvise
  behaviour decisions. Several current behaviours that look like bugs
  are features (climb alarms, deferred first-fix beep) — the QUIRKS file
  is the arbiter of intent.
