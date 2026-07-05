# Audio rewrite — journal

Append-only handoff log. One entry per commit attempt, newest at the
bottom. Format:

```text
## <card> — <date> — <SUCCESS | BLOCKED | FAILED>
What was done, in 2-6 lines.
Deviations from the card (if any) and why.
Gotchas / notes for the next agent.
```

Orchestrator may add curation notes marked `[orchestrator]`.

---

## (setup) — 2026-07-04 — baseline

Harness committed by Michael: 50 scenarios green, 100% reachable
line+branch coverage of audio_control.c, mutation suite 30/30 killed.
No rewrite code exists yet. Plan folder created; TEMP/audio-rewrite.md
(untracked) is superseded by plan/audio-rewrite/.
Known environment quirks: PowerShell sessions reset cwd between tool
calls — always `cd test` in the same command; gcovr filter must be
`".*audio_control"` (path-relative filters don't match on this setup);
gcda files ACCUMULATE across runs — delete before measuring coverage.

---

## A0 — 2026-07-04 — SUCCESS

Froze FlySight/audio_control.c to test/reference/audio_control_orig.c
(with a do-not-edit / deleted-at-C1 header). Added CMake target
audio_sim_ref (same as audio_sim but the frozen ref instead of the live
module, no coverage). Wrote test/scripts/fuzz_diff.py (differential
fuzzer) and gitignored test/fuzz-failures/.

Deviation/gotcha 1 — coverage leak: the existing
set_source_files_properties(... "--coverage") is DIRECTORY-scoped, so it
also instrumented config.c/nav.c/common.c in audio_sim_ref (shared
sources), which then failed to link (undefined __gcov_*). Fixed by
gating the flag on a per-target property:
COMPILE_OPTIONS "$<$<BOOL:$<TARGET_PROPERTY:FS_COVERAGE>>:--coverage>"
with FS_COVERAGE set only on audio_sim. audio_sim's coverage is
unchanged (gcno/gcda still generated, goldens still 50/50); audio_sim_ref
has zero gcov instrumentation (verified: no .gcno under its .dir). If you
ever see stale .gcno in the ref .dir, it is leftover from an aborted
build — a clean `rm -rf build` rebuild clears it.

Deviation/gotcha 2 — DoD command path form: `run_tests.py --exe
build/audio_sim_ref.exe` (forward-slash RELATIVE path) throws WinError 2
on this Windows host — Python subprocess/CreateProcess won't resolve a
relative program path with `/` separators. Use a backslash or absolute
path: `--exe build\audio_sim_ref.exe` → 50 passed. (run_tests.py is not
an A0-allowed file, so I did not "fix" it; the fuzzer itself always uses
absolute exe paths to sidestep this.)

Fuzzer notes for later cards: single random.Random(seed) drives one
iteration (gen_jump args then config, fixed draw order) so `--seed N`
reproduces exactly. It compares (exit_code, trace) as a unit. The
CURRENT code has a pre-existing shared integer divide-by-zero on some
random configs (exit 0xC0000094); both binaries crash identically, so
that is AGREEMENT, bucketed as "shared-crash", NOT a real diff. Only a
live-vs-ref divergence is saved to fuzz-failures/<seed>/. known-13
(speech mode 5/7 AND End_Nav>0 or Max_Dist>0) is counted, not saved.
Verification observed:
  cmake --build build      -> audio_sim.exe + audio_sim_ref.exe
  run_tests.py             -> 50 passed, 0 failed, 0 new
  run_tests.py --exe (ref) -> 50 passed, 0 failed, 0 new
  fuzz_diff.py --minutes 5 -> 1668 iters, 0 real diffs, 65 known-13,
                              53 shared-crash

---

## A1 — 2026-07-04 — SUCCESS

Consolidated every mutable module static into one FS_AudioControl_State_t
(single `static FS_AudioControl_State_t state;`). Members: timer_id,
cur_speech, sp_counter, flags, prev_flags, prevHMSL, g_suppress_tone,
speech_buf[16], speech_ptr, the four volatile ISR-read tone slots
(tonePitch/toneChirp/toneRate/toneHold, volatile kept inside the struct),
plus the former function-local statics x0/x1/x2 (updateTones) and
tone_timer (consumerTimer). FS_AudioControl_Init now does
memset(&state,0,sizeof(state)) then the existing explicit inits PLUS
state.x0 = INVALID_VALUE (x1/x2 stay zero — matches the old
zero-initialized statics; INVALID propagation gates the first two samples
exactly as before, QUIRKS #6). Pure mechanical rename otherwise.

[orchestrator] The A1 sub-agent (Opus) was terminated by an API monthly
spend limit immediately AFTER its single full-file Write completed but
BEFORE it built/tested/journaled/committed. I verified the completed
working tree independently to the full Phase-A standard rather than
re-dispatch a proven-correct result: builds clean; run_tests.py -> 50
passed, 0 failed, 0 new (live) AND on audio_sim_ref; git diff --
test/golden/ empty (speech-nav / speech-nav-gated l/r unchanged, so no
stack-layout shift); grep confirms no data statics remain except the
single `state` instance (static functions + the const sas_table are the
only other `static` hits); every struct member's type matches the
original declaration; volatile preserved on the four ISR slots. Journal
entry written and commit made by the orchestrator on the agent's behalf.

---

## A2 — 2026-07-04 — SUCCESS

Extracted the shared metrics layer to FlySight/flight_params.c/h. Moved
verbatim: the sas_table + SAS interpolation (now
FS_FlightParams_GetSpeedMul(config, hMSL) -> uint16_t, returns 1024 when
!use_sas) and the tone-path value switch (now
FS_FlightParams_GetValue(mode, current, config, scale, val, min, max)).
getValues is a one-line adapter calling GetValue with scale=10000;
speakValue now sources speed_mul from GetSpeedMul (then applies its own
unit conversion) and keeps ALL its local value computations. Added the
develop-compat FS_FlightParams_GetSASCorrectionFactor(hMSL) -> double
(same table, /1024.0), unused on this branch. Verified byte-exact:
run_tests.py 50/50 on both audio_sim and audio_sim_ref; fuzz_diff 2 min
= 700 iters, 0 real diffs.

Deviation from the card (deliberate, behaviour-preserving): speakValue is
NOT a thin adapter over GetValue for its value computations, only over
the SAS helper. Reason = an integer-math quirk that would be a hidden
behaviour change if unified now (that unification is B1, not A2):
  - speech inverse-glide is `100 * (int32_t)velD / current->gSpeed` where
    gSpeed is uint32_t, so the division is UNSIGNED; the tone path casts
    gSpeed to int32_t (SIGNED). For negative velD (climb) these differ.
  - speech dive-angle multiplies by 100 BEFORE truncation; the tone path
    does not — `100*trunc(x) != trunc(100*x)`.
  - speech speed modes use a unit-converted speed_mul (KMH/MPH/KNOTS
    scaling) that the tone path never applies.
So `scale` has one live caller on this branch (getValues, 10000); it
stays a parameter per the card so B1 can supply 100 without renumbering.
The glide scales are NOT unified (that is B1), exactly as instructed.

Mutation re-anchoring (mutation_test.py, an allowed file): gave each
MUTANT an optional 6th "file" field (default audio_control.c) and made
the harness restore each patched file per-iteration so mutants targeting
different files don't contaminate one another. M17 (glide scale) and M18
(SAS divisor) now point at flight_params.c — M17 mutates the scale-based
glide line (scale->literal 1000, kills the tone path), M18 the SAS
interpolation (count 2: it also appears in the dead-on-this-branch
GetSASCorrectionFactor; the live GetSpeedMul copy is what the goldens
catch). Both KILLED.

GOTCHA for future milestone cards: the mutation suite had FIVE OTHER
broken anchors that were pre-existing A1 collateral, NOT introduced here
— A1 moved statics into `state` but did not re-anchor M13/M20/M21/M27/M28
(tone_timer, toneRate, tonePitch, cur_speech, prevHMSL, flags), so they
were silently NO-MATCH/BUILD-FAIL after A1 (A1 was not a milestone card
and evidently never ran the mutation suite). This milestone card requires
30 killed / 0 NO-MATCH, so I re-anchored those five to the `state.`-
prefixed names too (pure text, mutation semantics unchanged). Final full
run: 30 killed, 0 NO-MATCH.

CMake: flight_params.c added to FIRMWARE_SOURCES (so it inherits the
FS_COVERAGE-gated --coverage, ON only for audio_sim) and to the explicit
audio_sim_ref source list (harmless: the frozen ref is self-contained and
just links the unused object). Verified no .gcno under the ref .dir and a
flight_params.c.gcno under the audio_sim .dir — no gcov leak into the
reference. NOTE for coverage cards: the gcovr filter is still
".*audio_control", so the moved code in flight_params.c is currently
OUTSIDE the measured set — a future coverage card should widen the filter
to include flight_params.

Verification observed:
  cmake --build build       -> audio_sim.exe + audio_sim_ref.exe
  run_tests.py              -> 50 passed, 0 failed, 0 new (live)
  run_tests.py --exe (ref)  -> 50 passed, 0 failed, 0 new
  mutation_test.py          -> 30 killed, 0 needing attention
  fuzz_diff.py --minutes 2  -> 700 iters, 0 real diffs, 26 known-13,
                               28 shared-crash

---

## A3 — 2026-07-04 — SUCCESS

Replaced the sentinel-char speech bytecode with a token queue in new
FlySight/audio_speech.c/h (FS_SpeechToken_t enum + token->wav table +
FS_Speech_t queue owned by audio_control's `state`). Deleted numberToSpeech,
the speakValue char-buffer body, and the ~150-line consumerTask ladder;
audio_control.c now just calls FS_Speech_Build*/HasPending/Clear/PlayNext.
Value math (speeds, glide/iglide, direction/bearing/distance, dive, altitude,
alt-step, ground-elev) is ported VERBATIM incl. the mode-12 Sp_Dec-0
divide-by-zero (preserved for B2; shared-crash in the fuzzer). All 50 goldens
byte-identical; fuzz_diff 3 min = 1126 iters, 0 real diffs (74 known-13, 39
shared-crash); mutation 30 killed / 0 NO-MATCH.

QUIRKS #13 handling (goldens' STOP condition NOT triggered — they are
byte-identical). CORRECTED RATIONALE — the original claim in this entry,
that the garbage suffix "is never played," was FALSE and has been retracted
after orchestrator differential testing (see [orchestrator] note below).
The truth: in gated mode 5/7 the `decimals = 0` override never runs (it lives
inside the gated block), so the garbage suffix/terminator land at
Sp_Dec-dependent offsets in the backward-built buffer and CAN corrupt the
utterance — e.g. overwriting the label region so the old code voices a
garbage `right.wav` in place of the label (fuzz seed 372: old plays
right.wav where new plays directn.wav, with downstream utterances shifting).
The new token code emits the golden-pinned sequences BYTE-IDENTICALLY and
intentionally DROPS this garbage for all other (non-golden) gated inputs.
Michael reviewed and ACCEPTED this on 2026-07-05: reproducing arbitrary
stack garbage across the architecture change is impossible and pointless,
and B2 removes the behaviour regardless — so A3 effectively pulls the #13
fix forward for the gated path. All such divergence is confined to the
fuzzer's known-13 class (End_Nav>0 or Max_Dist>0), verified below.

Also found while grounding the nav gates: `end_nav`/`max_dist` are uint16_t
(config.h), so End_Nav 300 -> *1000 -> 300000 wraps to 37856 mm; the gate
silences nav features below ~37.8 m AGL, not 300 m as the scenario comment
claims. Pre-existing; the goldens already bake it in; I use config->end_nav
verbatim so it matches. Not my card to fix.

QUIRKS #12 reproduction (mode 2/3 glide/iglide empty, NOT known-13 so the
fuzzer byte-checks it): FS_Speech_BuildValue mirrors the legacy backward-write
+ `end_ptr -= (dec==0)?4:(3-dec)` truncation index arithmetic on the token
buffer, so the pathology "falls out naturally" per the card. Deviation from
the card's literal token design: the label is ONE token (not '>'+mode+1 two
bytes). Verified this still reproduces #12 exactly: for Sp_Dec 1 the truncation
terminator lands on the single label slot and nulls it -> whole utterance
silent; Sp_Dec 0/2 leave the label -> label alone. Teens/tens are single
tokens (NUM_10..19 / TENS_2..9) as the card wants; they only occur on the
forward, non-truncated altitude/announce path so the width change is
output-invariant.

Mutation re-anchoring (mutation_test.py, allowed file; added an `AS` path):
M26 teens boundary, M27 label index, M28 announcement scale, M30 distance
rounding moved into audio_speech.c (file=AS). M27's old anchor
(`...mode + 1`) no longer exists (labelToken() switch) so it now mutates
`return TOK_LABEL_HORZ;`->VERT (mode 0, non-nav, well covered). M07 (alt-step
rounding) was count-2 in audio_control.c; the two copies split across files and
ONLY the audio_speech.c ALTITUDE copy is golden-covered (the updateAlarms copy
survives on these tracks), so M07 is now count-1 file=AS. Final: 30/30.

CMake: audio_speech.c added to FIRMWARE_SOURCES only (audio_sim; inherits the
FS_COVERAGE-gated --coverage). NOT added to the audio_sim_ref source list —
the frozen ref (audio_control_orig.c) is self-contained and must build without
it; verified ref builds + passes 50/50 and has zero audio_speech .gcno.

NOTE for the coverage card: the gcovr filter is still ".*audio_control", so
the new audio_speech.c is OUTSIDE the measured set (same gap A2 flagged for
flight_params.c). Widen the filter to include audio_speech before trusting a
coverage number.

Verification observed:
  cmake --build build       -> audio_sim.exe + audio_sim_ref.exe
  run_tests.py              -> 50 passed, 0 failed, 0 new (live)
  run_tests.py --exe (ref)  -> 50 passed, 0 failed, 0 new
  mutation_test.py          -> 30 killed, 0 needing attention
  fuzz_diff.py --minutes 3  -> 1126 iters, 0 real diffs, 74 known-13,
                               39 shared-crash

[orchestrator] 2026-07-05 — Independently verified and ACCEPTED (with the
rationale correction above). Re-ran build + 50/50 (live and ref) + golden
diff-vs-HEAD~ EMPTY + mutation 30/30. Because the agent's equivalence claim
was the one thing the fuzzer's known-13 bucket CANNOT check, I ran a
scratch copy of fuzz_diff with is_known_13 forced False (gated mode-5/7
byte-compared, not excused): 29/400 configs diverged, 100% classified
MODE57-GATED, ZERO out-of-class diffs — i.e. no genuine bug, and the old
code demonstrably DOES voice garbage in gated configs (disproving "never
played"). Michael's decision: accept; correct journal + QUIRKS #13 + BRIEF
§6 (done in the following orchestrator bookkeeping commit). Scratch tooling
removed; tree clean at 686fd9b.

---

## A4 — 2026-07-05 — SUCCESS

Split updateAlarms into AlarmSource_Update (alarm-window suppression + crossing
scan, QUIRKS #8) and AltModeSource_Update (silence-window + Alt_Step-window
suppression + the Alt_Step announcement decision). Added empty AlarmSource_t /
AltModeSource_t to `state` as scaffolding (fields land in A5; prevHMSL stays a
single shared member passed explicitly, NOT duplicated, per the card). All 50
goldens byte-identical (live AND ref); mutation 30/30; fuzz 3 min = 686 iters,
0 real diffs (42 known-13, 28 shared-crash).

DESIGN — why the sources emit requests instead of calling FS_Audio directly:
the six-step per-sample order interleaves the two concerns around the SHARED
rising-edge stop (step 4). AlarmSource's window scan (step 1) must run BEFORE
step 4 while its crossing fire (step 5) must run AFTER (else the zone-entry
FS_Audio_Stop would swallow the just-started alarm beep — verified this reorder
is observable). Likewise AltMode's windows (steps 2,3) precede step 4 but its
announce (step 6) follows it. So a single call per source is only possible if
the sources COMPUTE (suppress flags + a fire-index request + a want_alt_step
request) and producerTask applies step 4, then the alarm sound, then the
announce, in the original order. This matches the card's "outputs for suppress
flags / requests" wording and the brief's arbiter direction. Every state read
was checked to be unaffected by the move: the announce's FS_Speech_HasPending
(the only guard that observes step 4's Clear) is evaluated in producerTask at
the same point as before; want_alt_step folds in only side-effect-free guards
(alt_step>0, ALT_MIN floor, !FLAG_SAY_ALTITUDE, !suppress_alt, step_elev in
[min,max), V/H thresholds), and producerTask ANDs in !alarm_fired (== old
`i == num_alarms`) and !HasPending. Type-0/unknown alarm still "fires" (clears
speech, suppresses the announce, no sound) exactly as before — untouched, B5.

MUTATION — mutation_test.py NOT modified (no anchor moved). Relocating the
alarm-fire switch into producerTask would have turned `config->volume * 5`
into `config.volume * 5` (config is the producer's local struct, not a
pointer), breaking M05 and M22 (count-5 anchor). To avoid re-anchoring across
two text forms I kept the fire code in a small `fireAlarm(FS_Config_Data_t
*config, uint8_t index)` helper that reproduces the original lines verbatim on
a pointer — all five `config->volume * 5` sites and the M05 beep line stay
byte-identical, and M01-M04/M08/M09/M10 anchors were preserved by keeping the
source functions on `config->` pointer params. Full run: 30 killed, 0 NO-MATCH.

GOTCHA — the source structs (state.alarm/state.altmode) are intentionally
unused in A4 (empty scaffolding); no -Wall warning since unused struct members
don't warn. A5 populates them (step tracking, ground_elev latch). Also re-noted
the A0 quirk: `run_tests.py --exe build/audio_sim_ref.exe` (relative, forward
slashes) throws WinError 2 — use an absolute path for the ref binary.

Verification observed:
  cmake --build build       -> audio_sim.exe (clean, no warnings)
  run_tests.py              -> 50 passed, 0 failed, 0 new (live)
  run_tests.py --exe (ref, abs path) -> 50 passed, 0 failed, 0 new
  git diff test/golden/     -> EMPTY
  mutation_test.py          -> 30 killed, 0 needing attention
  fuzz_diff.py --minutes 3  -> 686 iters, 0 real diffs, 42 known-13,
                               28 shared-crash

---

## A5 — 2026-07-05 — SUCCESS

Completed the source decomposition and introduced the arbiter, the sole caller
of FS_Audio_* in the module. Dissolved the `flags` word, `toneHold` and
`g_suppress_tone`: FLAG_HAS_FIX/VERTICAL_ACC/prev_flags -> producer-level
has_fix/prev_has_fix/vacc_good; FLAG_FIRST_FIX/BEEP_DONE -> StartupSource;
FLAG_SAY_ALTITUDE -> AltModeSource.say_altitude latch; toneHold ->
Arbiter.tone_hold; g_suppress_tone -> Arbiter.tone_suppressed. New SpeechSource
(cur_speech + sp_counter) owns periodic-speech selection (SpeechSource_MaybeSpeak
/ _Tick, extracted verbatim from updateTones). All FS_Audio_* now live in four
Arbiter_* functions (ApplySuppression/FireAlarm/GrantTone/ConsumerTick) under an
ARBITER banner; producerTask/consumerTimer/consumerTask/Init call into them and
contain zero driver calls. All 50 goldens byte-identical (live AND ref);
mutation 30/30; fuzz 3 min = 683 iters, 0 real diffs (42 known-13, 28 shared-crash).

POLICY-AS-DATA: added AudioChannel_t (priority order) + a suppression_table
mapping each SuppressionZone_t to a channel bitmask. Sources OR the table entry
into a shared suppress_mask instead of writing bare suppress_tone/suppress_alt
bools; the arbiter derives suppress_tone = (mask & SUP_TONE) for the rising-edge
stop, and AltModeSource reads (mask & SUP_ALT_STEP) for the want_alt_step gate.
This is a pure re-encoding of A4's two bools (only ZONE_SILENCE_WINDOW sets the
ALT_STEP bit, matching the old suppress_alt = silence-window-only), verified
byte-identical. Two extras beyond the brief's three zones, both preserved from
the old code and noted in-source: ZONE_ALT_STEP_WINDOW (the alt-step suppression
window also silences TONE) and ZONE_SPEECH_ACTIVE (applied dynamically via
tone_hold at the consumer tick, not through the producer mask).

PRECEDENCE SPECIAL CASE (1, old code wins; well under the ~3 budget): the table
ranks STARTUP > SPEECH, but the STARTUP first-fix beep is deferred until the
speech queue is EMPTY and the driver idle (QUIRKS #3), so it actually yields to
any queued speech. I did NOT change behaviour to match the table -- the first-fix
grant stays in the empty-queue consumer branch. The table's STARTUP rank governs
only its precedence over ALT_GROUND_ELEV within that branch (first-fix checked
before ground-elev). Documented at the AudioChannel_t enum and Arbiter_ConsumerTick.
All other adjacent pairs (ALARM>STARTUP, ALT_GROUND_ELEV>ALT_STEP via the
say_altitude latch, ALT_STEP>SPEECH via the shared !HasPending guard within a
producer sample, SPEECH>TONE via tone_hold) match the table exactly. FLAG_SAY_ALTITUDE
still blocks Alt_Step AND all periodic speech (QUIRKS #17 refinement deferred to
B4); type-0 alarms still fire+cancel-speech (#18 deferred to B5).

QUEUE OWNERSHIP MOVED (audio_speech.c/h, an allowed file): FS_Speech_PlayNext no
longer calls FS_Audio_Play -- it now returns the wav filename (or NULL) and
advances; the arbiter (Arbiter_ConsumerTick) performs FS_Audio_Play with
sp_volume. Dropped the now-unused config param and the audio.h include from
audio_speech.c. This is what lets the grep check pass: after A5, FS_Audio_ in the
audio module appears ONLY inside the four Arbiter_* functions (audio_control.c
lines ~589-674) and never in audio_speech.c. (Other firmware files -- start_control.c,
config_mode.c, active_mode.c, start_mode.c -- use FS_Audio_ directly; they are
outside the rewritten module and out of scope for the grep check.)

MUTATION re-anchoring (mutation_test.py, allowed file): only M13 and M23 moved.
M13 "state.flags |= FLAG_BEEP_DONE;" -> "state.startup.beep_done = true;"; M23
"toneHold = 1;" -> "arb.tone_hold = 1;" (replacement "arb.tone_hold = 0;", which
now also matches the else-branch reset line -- harmless, the count check is on the
unique old string). M24/M25/M29 did NOT need moving despite the card's heads-up:
their anchor strings ("sp_counter >= config->sp_rate &&", "sp_counter +=
config->rate;", "config->init_mode == 1") are substrings that survived the
state.->state.speech_src. rename. M20/M21 (tone slots kept as state members) and
all source-expression anchors (M01-M10) unchanged. Full run: 30 killed, 0 NO-MATCH.

NOTE for later cards: the brief's §3 ToneSpec double-buffer handoff (two slots +
volatile active index) is NOT built here -- A5's Steps don't call for it and the
sim can't test it. The tone slots remain single volatile members
(tonePitch/toneChirp/toneRate) read directly by Arbiter_GrantTone; a future card
can introduce the double-buffer. gcovr filter is still ".*audio_control" (A2/A3
gap unchanged): the priority/suppression data lives in audio_control.c so it is
measured, but audio_speech.c/flight_params.c remain outside the filter.

Verification observed:
  cmake --build build       -> audio_sim.exe (clean, no warnings)
  run_tests.py              -> 50 passed, 0 failed, 0 new (live)
  run_tests.py --exe (ref, abs path) -> 50 passed, 0 failed, 0 new
  git diff test/golden/     -> EMPTY
  mutation_test.py          -> 30 killed, 0 needing attention
  fuzz_diff.py --minutes 3  -> 683 iters, 0 real diffs, 42 known-13,
                               28 shared-crash

---

## A6 — 2026-07-05 — SUCCESS

Made the producer-task -> consumerTimer-ISR tone handoff atomic. Folded the
three independent volatiles (tonePitch/toneChirp/toneRate) into one coherent
ToneSpec_t {uint32_t pitch; int32_t chirp; uint16_t rate;} and double-buffered
it: `ToneSpec_t tone_slots[2]` + `volatile uint32_t tone_active`. The set*
helpers now write the DRAFT (inactive) slot via toneDraft(); producerTask seeds
the draft from the published slot on entry (so a pass touching only a subset of
fields carries the rest forward, as the old persistent volatiles did) and
publishes with a single flip `state.tone_active = !state.tone_active;` at the
very end. Arbiter_GrantTone (ISR) snapshots the index once and copies that whole
slot into a local `spec`, so pitch/chirp/rate are mutually coherent per tick.
Accumulator kept bit-identical: `state.tone_timer += spec.rate`, fire when
`0x10000 - state.tone_timer <= spec.rate`; 125 ms beep and 20-tick period
untouched.

HARDWARE LISTEN-TEST REQUIRED (Phase C): this commit is SIM-INVISIBLE. The sim
is single-threaded (producerTask always runs to completion before any
consumerTimer event), so it can never exercise the torn-read race this fixes,
and goldens stayed byte-identical (as the card predicts). The ISR-concurrency
correctness of the double-buffer is verified ONLY by review here; it MUST be
confirmed on target by ear in Phase C.

Memory-ordering rationale (commented at Arbiter_GrantTone): single-writer
(producer task) / single-reader (consumerTimer ISR) on one Cortex-M4 core. The
producer only ever mutates the inactive slot and publishes via a single
naturally-aligned 32-bit store to tone_active (atomic on M4). The ISR cannot be
preempted by the producer, so it reads a slot atomically; and same-core
exception entry orders the producer's slot writes before its tone_active store
as observed by the ISR. No DMB needed -- a barrier would only matter for a
second core or a DMA/peripheral observer. Only the producer writes tone_active,
so its read-modify-write flip cannot be lost. tone_hold (the old toneHold) is
NOT part of the spec: it is a single-byte latch on the consumerTask->ISR
boundary (not producer->ISR), inherently atomic as one aligned store, and does
not participate in the pitch/chirp/rate coherency group -- left as-is; the
card's ToneSpec_t is exactly the 3 producer-published fields.

Mutation re-anchoring (mutation_test.py, NOT in this card's allowed list --
touched only because two anchors went NO-MATCH, as the card permits): M20
(`state.toneRate` -> `spec.rate`) and M21 (`state.tonePitch + state.toneChirp`
-> `spec.pitch + spec.chirp`) reference strings that no longer exist after the
rename; re-anchored to the new ISR-local `spec.*` form with identical mutation
semantics. M13 (`state.startup.beep_done = true;`) in the same region was
unaffected. Both re-anchored mutants KILLED; full run 30 killed, 0 NO-MATCH.

Verification observed:
  cmake --build build       -> audio_sim.exe (clean, no warnings)
  run_tests.py              -> 50 passed, 0 failed, 0 new (live)
  run_tests.py --exe (ref, abs path) -> 50 passed, 0 failed, 0 new
  git diff test/golden/     -> EMPTY
  mutation_test.py          -> 30 killed, 0 needing attention
  fuzz_diff.py --minutes 3  -> 628 iters, 0 real diffs, 38 known-13,
                               24 shared-crash

---

## A6 (follow-up) — 2026-07-05 — SUCCESS

Compiler-barrier hardening for the A6 tone-spec handoff, authorized by
Michael after orchestrator review. The producer's draft-slot writes (the
seed copy at the top of producerTask plus every set*/setRate/setPitch/
setChirp) are NON-volatile, and the C standard only orders volatile
accesses relative to each other -- so the optimizer could legally sink
those slot stores PAST the volatile `state.tone_active = !state.tone_active;`
flip, re-introducing a same-core torn read. Added a pure compiler barrier
`__asm__ volatile ("" ::: "memory");` immediately before the flip (NOT a
DMB -- no hardware barrier wanted; the single-core hardware ordering was
already argued correct). Updated the adjacent publish comment and the
Arbiter_GrantTone memory-ordering note to explain the barrier pairs with
that hardware argument to make publish-then-flip airtight. ISR read side
(idx then slot[idx], address-dependent) untouched.

SIM-INVISIBLE (as A6): single-threaded sim can't exercise the race;
goldens byte-identical. Still folded into the A6 HARDWARE listen-test
(Phase C). Touched only FlySight/audio_control.c + this journal.

Verification observed:
  cmake --build build       -> audio_sim.exe (clean, only audio_control.c
                               recompiled, no warnings)
  run_tests.py              -> 50 passed, 0 failed, 0 new (live)
  run_tests.py --exe (ref)  -> 50 passed, 0 failed, 0 new
  git diff -- test/golden/  -> EMPTY
