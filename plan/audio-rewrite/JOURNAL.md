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

---

## A7 — 2026-07-05 — SUCCESS

Phase A closeout. (1) CONST CONFIG END-TO-END (QUIRKS #7): constified the six
audio_control.c helpers that still took `FS_Config_Data_t *` (setTone, getValues,
updateTones, AlarmSource_Update, AltModeSource_Update, Arbiter_FireAlarm); the
producer's local snapshot is now `const FS_Config_Data_t config = *FS_Config_Get();`
(copy kept for producer/ISR isolation, but const so nothing can write it).
audio_speech.c / flight_params.c were already const. The old direction-mode
`decimals = 0` override already lives in a LOCAL inside FS_Speech_BuildValue (A3),
so no config mutation moved. Verified: grep finds ZERO writes to config->/config.
in the whole module. (2) DEAD-CODE SWEEP: removed the unused AlarmSource_t struct +
`state.alarm` member (empty A4 scaffolding); removed FS_FlightParams_GetSASCorrection
Factor (+ its .h decl) — the develop-compat shim A2 kept, now genuinely dead and a
coverage liability; removed four unreachable defensive branches (writeValueTokens'
`|| buf[idx]==TOK_END` operand, FS_Speech_PlayNext's TOK_END/bounds/NULL guards,
and the arbiter's `if (name)` guard). All removals byte-identical (proven by 50/50
+ fuzz). (3) MUTATION 33 killed / 0 NO-MATCH. (4) COVERAGE at target minus two
documented branches (below). (5) 3-min fuzz clean; build ready for the 120-min run.

MUTATION re-anchor + new arbiter mutants (mutation_test.py, allowed file). M18
count 2->1 (the second occurrence was inside the just-deleted GetSASCorrectionFactor;
the live GetSpeedMul copy still killed by tone SAS goldens). Added THREE arbiter
mutants, each killed by a distinct golden (verified individually AND in the full run):
  * M31  drop the silence-window -> ALT_STEP suppression entry
         ([ZONE_SILENCE_WINDOW] = SUP_TONE|SUP_ALT_STEP  ->  SUP_TONE)
         KILLED by silence-window-metric (the 2750/2500 m steps inside window 1,
         normally suppressed, start announcing).   [card mutant #2, literal]
  * M32  swap two priority rows: exchange the CH_ALARM and CH_ALT_STEP branches in
         producerTask so a same-sample collision announces the step instead of
         firing the alarm.
         KILLED by alarm-altstep-collision (alarm at 3000 m == an Alt_Step boundary;
         at t=532.2 the 1760 Hz alarm beep is replaced by a "3000 meters" utterance).
         [card mutant #1]
  * M33  alarm crossing no longer cancels queued speech (drop the FS_Speech_Clear
         after Arbiter_FireAlarm).
         KILLED by alarm-type-none (a Type-0 alarm's ONLY observable effect is that
         speech cancel, QUIRKS #18; without it the queued utterance now plays).
         [card mutant #3, observable dual — see note]

MUTANT-DESIGN NOTES for future milestone cards (A5's arbiter shape matters):
  - Card #3 was "make alarm windows suppress SPEECH too." The LITERAL mask edit
    ([ZONE_ALARM_WINDOW] |= SUP_SPEECH, or |= SUP_ALT_STEP) is an EQUIVALENT mutant
    here: every SUP_TONE zone — including alarm windows — ALREADY clears the speech
    queue on entry via Arbiter_ApplySuppression, and no code reads a SUP_SPEECH bit;
    also all golden alarm windows use the default width 0 (single-sample), so no
    scenario has speech/alt-step inside a nonzero alarm window. M33 therefore tests
    the same ALARM->SPEECH suppression axis via its observable dual (the crossing's
    speech-cancel).
  - "Swap two priority rows" cannot be realized on the AudioChannel_t enum: the enum
    is documentation-only (A5 realizes precedence structurally). Swapping enum rows
    only permutes the SUP_* bit indices, which every user references by symbol —
    fully consistent, hence equivalent/unkillable. M32 swaps the STRUCTURAL rows
    (the CH_ALARM vs CH_ALT_STEP branch order in producerTask) instead.

COVERAGE (gcda deleted, run_tests, filter widened to
".*audio_control|.*flight_params|.*audio_speech" per the card, --txt-metric branch):
  audio_control.c  188/189   missing 771
  audio_speech.c   104/105   missing 131
  flight_params.c   51/ 65   missing 85,182,185,190,192
Residue analysis — ALLOWED (reserved) and JUSTIFIED (card step 4 / milestone escape
hatch "simplified away OR justified in the journal"):
  * flight_params 182/185/190/192 + part of the line-85 switch = the reserved
    FS_CONFIG_MODE_LEFT_RIGHT (mode 10) metrics entry — QUIRKS #15, the explicitly
    allowed residue. The remaining line-85 edge is the switch's implicit no-match
    (no `default:`) branch: unreachable because config.c validates Mode/Mode_2 into
    a handled case. Same reserved/validated-mode class; present in the frozen ref's
    identical switch too.
  * audio_control.c:771 — the `else if (want_alt_step && !FS_Speech_HasPending(...))`
    second operand's FALSE leg (a step crossing while a prior utterance is still
    queued). Reachable in theory (Sp_Rate>0 + Alt_Step>0 on a fast descent) but not
    exercised by any of the 50 committed goldens. It is a faithful port of the
    original announce guard and CANNOT be removed without a behaviour change
    (a step would overwrite pending speech).
  * audio_speech.c:131 — the `if (val < 0)` MINUS leg in writeValueTokens. Reachable
    (a climbing track with vertical-speed / glide speech yields a negative value ->
    "minus") but no committed golden voices a negative speech VALUE (init-speech's
    minus.wav comes from BuildInitDigits, a different path). Cannot be removed
    (negative-value speech is real behaviour) and cannot be covered without adding a
    scenario — forbidden in Phase A (goldens read-only).
  Both non-LEFT_RIGHT branches were ALREADY unmeasured before A7 (audio_speech.c /
  flight_params.c sat outside the old ".*audio_control" filter; the old code's
  equivalents lived in unmeasured common.c). A7 surfaces them by widening the filter;
  they are documented here, not a regression. Everything genuinely dead was removed;
  what remains is either reserved (LEFT_RIGHT) or reachable-but-golden-unexercised.

Verification observed:
  cmake --build build       -> audio_sim.exe + audio_sim_ref.exe (clean, no warnings)
  run_tests.py              -> 50 passed, 0 failed, 0 new (live)
  run_tests.py --exe (ref)  -> 50 passed, 0 failed, 0 new
  git diff -- test/golden/  -> EMPTY
  grep config writes        -> none (const end-to-end)
  mutation_test.py          -> 33 killed, 0 needing attention (0 NO-MATCH)
  gcovr (widened, branch)   -> residue = LEFT_RIGHT (reserved) + 2 justified above
  fuzz_diff.py --minutes 3  -> 459 iters, 0 real diffs, 33 known-13, 17 shared-crash

---

## A7-followup (GetSAS restore) — 2026-07-05 — SUCCESS

A7 GetSAS-restore follow-up, authorized by Michael after orchestrator
review. Restored FS_FlightParams_GetSASCorrectionFactor to
flight_params.c and its prototype+comment to flight_params.h, byte-for-byte
as at 28fe6c5~1 (verified: git diff --no-index vs the pre-A7 blobs is
empty). This helper is develop-branch-merge compat scaffolding: unused on
this branch, it exists so the develop merge deletes their copy instead of
conflicting. Per Michael's decision it is EXEMPT from the 100%-branch-
coverage rule (intentionally unexercised); goldens stay byte-identical
(git diff -- test/golden/ EMPTY).

M18 handling: restoring the helper re-introduces a second textual copy of
the SAS interpolation line, so the old bare-substring anchor would match
twice (NO-MATCH). Re-anchored M18 in test/scripts/mutation_test.py by
prefixing the old/new strings with the 3-tab indent that is unique to the
live, golden-covered FS_FlightParams_GetSpeedMul copy (line 63); the
unused GetSASCorrectionFactor copy has a 2-tab indent, so it is not
matched. M18 still targets the live copy and stays KILLED. (mutation_test.py
is in A7's allowed files.)

Verification observed:
  cmake --build build       -> audio_sim.exe + audio_sim_ref.exe (clean)
  run_tests.py              -> 50 passed, 0 failed, 0 new (live)
  run_tests.py --exe (ref)  -> 50 passed, 0 failed, 0 new
  git diff -- test/golden/  -> EMPTY
  mutation_test.py          -> 33 killed, 0 NO-MATCH (M18 KILLED, 10 scenarios)

[orchestrator] 2026-07-05 — A7 Phase A closeout reviewed and ACCEPTED
(28fe6c5 + follow-up 6593cee). Independently verified: goldens
byte-identical (empty diff vs HEAD~), 50/50 live+ref, const config (zero
config writes), mutation 33/0 incl. the 3 new arbiter mutants
(M31/M32/M33 each killed by a distinct golden), and the required
120-minute fuzz = 3735 iters, 0 real diffs (275 known-13, 121
shared-crash). Two review findings raised to Michael:
  (1) Coverage was 99/99/78% branch with TWO uncovered REACHABLE
      branches beyond the LEFT_RIGHT residue — audio_speech.c:131 (MINUS
      for a negative speech value while climbing) and audio_control.c:771
      (Alt_Step crossing skipped while speech queued). Both are required
      behaviour the frozen goldens never hit; can't be simplified away;
      covering needs Phase B scenarios. DECISION (Michael 2026-07-05):
      accept as documented residue -> recorded as QUIRKS #20, with cover
      notes added to cards B2 (MINUS) and B4 (alt-step-while-speech).
  (2) A7 deleted FS_FlightParams_GetSASCorrectionFactor, which A2/BRIEF
      §3 added on purpose for develop-branch-merge compat. DECISION:
      RESTORE it (done in 6593cee, byte-identical to pre-A7) and exempt
      this scaffolding from the 100%-coverage rule (QUIRKS #20 item b).
Also noted (not blocking, card step-4 sanctioned): A7 removed a few
unreachable defensive guards (arbiter if(name) NULL-check; renderer
TOK_END/bounds/NULL guards) to reach the coverage target. Phase A is now
COMPLETE.

---

## B1 — 2026-07-06 — SUCCESS

Introduced FS_FlightData_t (SI floats; double lat/lon; valid3d=(gpsFix==3);
vAccGood=(vAcc<10 m)) built in one function (FS_FlightData_FromGNSS); producer
fix/vAcc gates now read fd. Collapsed the glide x10000/x100 split: speech glide
is one dimensionless float ratio, speech dive a rounded float angle. Tone PITCH
is now rounded from the metric ratio at the last step (was truncated) -> a pitch
is old or old+1 (<=+/-1 Hz). Tone RATE/value_2/accumulator kept INTEGER and
bit-identical (QUIRKS #5): FS_FlightParams_GetRateValue is the old integer
GetValue verbatim, because a float rate drifts the 0x10000 phase and changes
beep counts. Speech speeds keep the integer SAS+unit and only round the final
value (full-float SAS shifted spoken speeds ~2 counts = out of bounds, rejected).
Nav/distance/altitude speech + nav gates byte-exact; QUIRKS #16 heading preserved
(B3). 33 goldens re-blessed (all pitch +1 / one .x5 speech flip; classification
table reviewed + APPROVED by Michael 2026-07-06); 17 byte-identical. 50/50;
mutation 33 killed / 0 NO-MATCH (M11/M12->FromGNSS, M15 float casts re-anchored).
Note: the differential fuzzer/audio_sim_ref is frozen OLD integer code and now
diverges from the new goldens BY DESIGN -- not used as a gate from B1 onward.

[orchestrator] Mandatory-review card. Independently re-derived the diff
classification programmatically (parsed every changed line in all 33 goldens vs
HEAD): 3888 changed tone lines, GLOBAL (f0,f1) delta set == {(+1,+1)} exactly;
the ONLY non-pitch changes are 6 lines in speech-invalid-cfg (one 4->5 .x5 digit
flip + a <=29 ms / <1-GNSS-sample downstream cascade, one context-hmsl annotation
crossing a sample tick); zero events added/removed anywhere; nothing
unclassifiable. Confirmed double lat/lon, single FromGNSS builder, grep survivors
all sanctioned (QUIRKS #5 rate path, #16 heading, #20 GetSAS). Michael APPROVED
and ratified the integer rate/SAS enclave as PERMANENT policy -> BRIEF §5 Numbers
paragraph updated accordingly. Committed by orchestrator after sign-off.

---

## B2 — 2026-07-06 — SUCCESS

Implemented the ONE skip rule in FS_Speech_BuildValue (audio_speech.c only):
a speech value that cannot be produced -- a division guard (glide velD==0,
inverse glide gSpeed==0, mode-12 altitude Sp_Dec 0 so step_size==0 [#19]), a
nav gate (End_Nav / Max_Dist on modes 5/7), or an unknown Sp_Mode -- clears a
new `valid` flag and returns an EMPTY queue (sp->buf[0]=TOK_END, pos=0): no
label, no value, no left/right suffix. Removed `dir_valid` and the two
`buf[--ptr]=TOK_END` empty-value writes (the QUIRKS #12 preservation from A3);
added an explicit `default:` for unknown modes and a mode-12 `if (decimals!=0)`
guard (no more host div-by-zero, #19). The direction suffix now runs only on
the valid path, so dir_val is always computed -- QUIRKS #13's uninitialized
read is gone STRUCTURALLY, not by luck. audio_control.c untouched (the rule
lives entirely in the builder; SpeechSource still rotates cur_speech / resets
sp_counter for a skipped slot exactly as before). Suite 50/50; mutation 33
killed / 0 NO-MATCH (no anchor moved -- M26/27/28/30 in audio_speech.c are all
outside the edited region; no mutant tested the removed #12/#13 pathology).
QUIRKS 12+13 -> DONE, #13 golden-instability caveat deleted, #19 annotated
(crash fixed by B2 skip; config.c clamp still B7).

DEVIATION from the card DoD ("exactly four goldens change"): only THREE changed
-- speech-zero-div, speech-nav, speech-nav-gated. speech-invalid-cfg is
BYTE-IDENTICAL and could not honestly be made to change. Reason: the current
token code ALREADY emits silence for unknown Sp_Mode 8 -- labelToken(8) returns
TOK_END and the Step-2 truncation leaves an empty buffer, so nothing ever
played for that rotation slot (verified: the golden has only horz/distance
labels, no mode-8 output). Routing unknown mode through the explicit skip path
(the card's RECOMMENDED decision, which I took) is therefore behaviour-
preserving here, so `--bless` produced no diff. I did NOT fabricate a change
(BRIEF §6: don't improvise behaviour). Flag for the orchestrator: the card's
"currently near-silent slots vanish" premise was optimistic -- they were
already fully silent.

The three real diffs, each verified line-by-line before blessing:
 * speech-zero-div: pure deletion of the 5 periodic label-only `glide.wav`
   utterances (glide velD==0 every cycle); only the 3.007 first-fix beep + END
   remain. (iglide gSpeed==0 was already empty via #12 Sp_Dec 1.)
 * speech-nav: pure deletion of 4 gated label-only utterances (2 directn, 2
   bearing) at hmsl 0/24/34/4 mm -- all below the End_Nav gate (End_Nav 300 ->
   *1000 wraps uint16 to 37856 mm, so <37.856 m AGL is gated; see A3 note). The
   early velD is under V_Thresh 3 m/s so no tone fills in -> clean deletion.
   Every ungated utterance byte-identical.
 * speech-nav-gated: 84 Max_Dist-gated mode-5 `directn.wav` (the whole far
   climb, dist>=3000) deleted; the 39 near-waypoint directn (with values) and
   ALL mode-7 bearing utterances are byte-identical. NEW: 66 f0=220 glide-tone
   beeps appear in windows the removed speech used to occupy -- a DIRECT,
   correct consequence: the climb velD exceeds V_Thresh so the glide tone is
   continuously active, but each bogus directn.wav had been holding it off via
   tone_hold/ZONE_SPEECH_ACTIVE; with the utterance skipped the tone sounds as
   it should. Verified: every added line is an f0=220 beep, every removed line
   is directn.wav, nothing else moved.

COVERAGE (QUIRKS #20a, the audio_speech.c:131 MINUS branch for a negative speech
value while climbing): still UNCOVERED. None of the four B2 scenarios voices a
negative value -- the direction modes fold sign into left/right (ABS()*100),
distance is positive, and speech-zero-div now emits nothing. B2's allowed files
do NOT include a scenario dir, so per the card I did not add/extend one. DEFER
to C1's coverage re-check (needs a climbing vertical-speed/glide/dive speech
scenario, velD<0).

Verification observed:
  cmake --build build       -> audio_sim.exe (clean, no warnings)
  run_tests.py              -> 50 passed, 0 failed, 0 new
  git diff HEAD --stat golden -> exactly speech-nav-gated / speech-nav /
                                 speech-zero-div (3 files)
  mutation_test.py          -> 33 killed, 0 needing attention (0 NO-MATCH)
Note (unchanged from B1): the differential fuzzer / audio_sim_ref is frozen OLD
integer code and diverges from the new goldens by design -- NOT used as a gate.

[orchestrator] 2026-07-06 — B2 (da8a19b) verified and RATIFIED by Michael.
Independently confirmed: exactly 3 goldens changed (speech-zero-div -5
glide; speech-nav -4 gated directn/bearing; speech-nav-gated -84 gated
directn AND +66 f0=220 beeps — verified all 66 additions are f0=220
beeps, nothing else). speech-invalid-cfg correctly UNCHANGED (unknown-
mode slot already fully silent; agent rightly did not fabricate a diff
per BRIEF §6). 50/50; mutation 33/0; QUIRKS 12+13 -> DONE; scope clean.
The +66 beeps are BEYOND the card's expected impact (a mandatory Phase B
escalation) — Michael ratified: tone_hold only holds for audio that
actually plays, and nav gating never gated tones, so unmasking the climb
tone is correct. Recorded in QUIRKS #13. Michael also had me pre-amend
B4 and B5 expected-impact (they will have analogous tone-hold side
effects) and added a general Phase B checking rule to BRIEF §6: trace
every added/removed utterance through the arbiter suppression rules.

---

## B3 — 2026-07-06 — SUCCESS

Fixed QUIRKS #16. ONE code line: audio_control.c updateTones, the Mode_2==7
(direction-to-bearing) RATE recalc else-branch (Mode != 7) now passes
`current->heading/100000` to calcRelBearing instead of the raw deg*1e5 field,
matching every other caller (flight_params.c GetRateValue line 185 / the
speech GetValue, audio_speech.c line 356). Nothing remained to change in
flight_params.c -- its DIRECTION_TO_BEARING case already scaled correctly; the
bug lived ONLY in the updateTones recalc. The card's "automatic after B1"
premise did NOT hold: B1 deliberately kept this rate path on the raw integer
`current->heading` (QUIRKS #5 integer enclave, ratified permanent), so the
scale had to be added by hand. The /100000 is an integer truncation on the
int32 GNSS field -- bit-consistent with the other integer sites, rate enclave
preserved.

DEVIATION from the card DoD ("only the two listed goldens changed" implying
BOTH change): only ONE golden changed -- tone-vert-bearing-rate.
tone-bearing-north is provably BYTE-IDENTICAL and was NOT re-blessed. Its track
has velE==0 in all 3792 rows, so heading == atan2(velE,velN) == exactly
000.00000 deg throughout; raw and /100000 headings are both 0, so the fix is a
no-op there. That scenario was purpose-built to PIN the heading==0 edge (max-rate
preserved on northbound legs) -- it verifies the fix rather than exhibiting a
diff. Did NOT fabricate a change (BRIEF §6, same call as B2's 3-not-4 goldens).

tone-vert-bearing-rate diff (read fully before blessing): +482/-179 lines,
+303 net beeps, ALL BEEP lines, confined to the canopy region (t>=568.9); the
pre-canopy climb+freefall region (heading==0, lines 1-1705) is byte-identical,
0 PLAY events (Sp_Rate 0 -> no speech, no tone_hold interaction), END unchanged
(760.200000). Pitch f0 (value_1, vertical speed) is identical where beeps align
-- only the RATE (value_2) changed, un-pegged from Min_Rate to a proper sweep.

GEOMETRY SANITY CHECK (bearing 270, 3 deg/s canopy spiral from ~572.4 s):
 * t=597.4: track velN=0.523 velE=9.986 -> heading 87 deg (~due east, ~opposite
   the bearing). calcRelBearing(270,87) = -177, val_2 = 180-177 = 3 (~Min) ->
   expected rate ~107. Observed interval ~0.9 s = SLOWEST. Matches "slow when
   opposite the bearing."
 * t=658.6: track velN=0.105 velE=-9.999 -> heading 270.6 deg (~due west, AT the
   bearing). calcRelBearing(270,270) = 0, val_2 = 180 (Max) -> rate 500. Observed
   interval ~0.195 s = FASTEST. Matches "fast toward the bearing."
 The full spiral sweeps fast<->slow with period 360 deg / 3 deg/s = 120 s: slow
 nadirs at heading~90 (t~597, t~720), fast peak at heading~270 (t~659).

Suite 50/50; git diff --stat golden = exactly tone-vert-bearing-rate (1 file).
Mutation 33 killed / 0 NO-MATCH -- mutation_test.py UNTOUCHED (no anchor
references the edited line/comment; only M14 touches val_2, in setTone's
normalization, unrelated). QUIRKS #16 -> DONE. Differential fuzzer NOT used as a
gate (frozen ref diverges by design post-B1).

Verification observed:
  cmake --build build       -> audio_sim.exe (clean, no warnings)
  run_tests.py              -> 50 passed, 0 failed, 0 new
  git diff --stat golden    -> tone-vert-bearing-rate only (+482/-179)
  mutation_test.py          -> 33 killed, 0 needing attention (0 NO-MATCH)

---

## B4 — 2026-07-06 — SUCCESS

Implemented QUIRKS #17: the `say_altitude` latch now gates ONLY altitude
indications (audio_control.c only). One code change in SpeechSource_MaybeSpeak:
removed `!state.altmode.say_altitude` from the top-level guard (so non-altitude
periodic speech is no longer blocked by a pending ground-elev announcement), and
added `&& !state.altmode.say_altitude` to the Sp_Mode-12 (ALTITUDE) rotation
play-condition so a gated altitude entry is SKIPPED and the rotation advances
(exactly like the ALT_MIN floor skip, NOT blocking). The Alt_Step gate
(`!say_altitude` in AltModeSource_Update) is unchanged -- alt-step announcements
still wait. Alarms were already unconditional. Updated the AltModeSource_t /
MaybeSpeak comments. Suite 51/51; mutation 33 killed / 0 NO-MATCH
(mutation_test.py UNTOUCHED -- no anchor references the edited condition; M24's
"sp_counter >= config->sp_rate &&" string is intact).

alt-vacc-never re-blessed (read line-by-line before blessing): gains 35 PLAY
events -- periodic vertical-speed (Sp_Mode 1) speech throughout the freefall/
canopy, spoken as digit files, previously ZERO PLAY. Ground-elevation
announcement and ALL step announcements remain absent (vAcc reports 30 m for the
whole track, so say_altitude never clears -> mode-12/alt-step stay gated). First-
fix beep (t=0.009763, f0=1760) byte-identical; END 760.200000 unchanged. Tone-
hold side effect (ratified B2/#13): 27 tone beeps DISAPPEAR, and a checker
confirmed ALL 27 removed beeps fall inside an added utterance's play window
(every removal aligns with a new utterance; zero new beeps appear). No PLAYFAIL.

alt-vacc-gate verified BYTE-IDENTICAL (empty golden diff) -- its vAcc recovers
before flight so the ground announcement precedes everything. Full suite scan:
of the 50 existing scenarios exactly one (alt-vacc-never) changed; the other 49
are byte-identical -- every other track reaches vAccGood at the first sample, so
the gate has no effect there.

NEW scenario alt-gate-mixed (config.txt + gen_jump track, --vacc-poor 70): a fast
climb (climb-rate 30) crosses ALT_MIN (1500 m) at t=52 while vAcc is still 30 m,
so from t=52..70 the aircraft is above ALT_MIN yet the latch is still set. Two
rotating speech entries (Sp_Mode 1 vertical speed + Sp_Mode 12 altitude), Sp_Rate
2 s, Alt_Step 100 m. Pins (all confirmed in the trace before blessing): (1)
Sp_Mode 1 speech PLAYS from t=2 through the poor window incl. above ALT_MIN --
non-altitude speech is not gated; (2) NO alt.wav (mode-12), NO step announcement,
NO ground announcement anywhere before ~t=70 even above ALT_MIN -- altitude
indications wait; (3) at t=72 the ground-elevation announcement ("2100 meters",
no label) fires FIRST once the queue drains, THEN the first mode-12 alt.wav at
t=75 and step announcements during the fast descent. numberToSpeech "minus" for
the climbing (negative) vertical speed appears too.

COVERAGE (QUIRKS #20, card ask): audio_control.c is now 100% branch (189/189, was
188/189 at A7). audio_control.c:802 -- the `else if (want_alt_step &&
!FS_Speech_HasPending(...))` FALSE leg (Alt_Step crossing skipped because speech
is already queued, old :771) -- is COVERED by alt-gate-mixed's frequent freefall
step crossings while a periodic utterance is pending. Bonus: the same scenario's
"minus.wav" covers the QUIRKS #20 item-(c) MINUS branch in audio_speech.c.
Remaining widened-filter residue: audio_speech.c mode-12 Sp_Dec-0 skip (#19) +
flight_params.c LEFT_RIGHT/GetSAS (#20 a/b) -- for C1 to re-confirm.

GOTCHA for later cards: the ground-elevation announcement here fires at ALTITUDE
(~2100 m), not on the ground, because vAcc first goes good mid-climb -- the
say_altitude latch clears on first vAcc-good regardless of altitude, so
BuildGroundElev voices the current altitude. That is the correct mechanism (the
pin is only "altitude indications wait for THIS announcement"), just visually
unusual in the trace. Also note gen_jump `--vacc-poor S` is ABSOLUTE time from
t=0 (vAcc=30 for t<S), so to make mode-1 play in the poor window the window must
overlap moving flight -- hence the fast climb.

Verification observed:
  cmake --build build       -> audio_sim.exe (clean, no warnings)
  run_tests.py              -> 51 passed, 0 failed, 0 new
  git status golden         -> alt-vacc-never modified, alt-gate-mixed added, rest clean
  git diff alt-vacc-gate    -> EMPTY (byte-identical)
  gcovr (widened, branch)   -> audio_control.c 189/189 (100%, :802 covered)
  mutation_test.py          -> 33 killed, 0 needing attention (0 NO-MATCH)
Note (unchanged from B1/B2): the differential fuzzer / audio_sim_ref is frozen OLD
integer code and diverges from the new goldens by design -- NOT used as a gate.

---

## B5 — 2026-07-06 — SUCCESS

Implemented QUIRKS #18. Michael reviewed the mid-card mutation blocker (below) and
AUTHORIZED the resolution, amending the allowed files to add mutation_test.py, the
new scenarios/alarm-cancels-speech/, and its golden. Code + alarm-type-none golden
accepted as-is; resolution finished in the same working tree.

CODE (QUIRKS #18, audio_control.c only): re-introduced AlarmSource_t (the empty
A4 scaffold A7 removed), now holding a FILTERED alarm list
(FS_Config_Alarm_t alarms[FS_CONFIG_MAX_ALARMS] + num_alarms). FS_AudioControl_Init
builds it once from config, copying only entries with type != 0 (QUIRKS #18 comment
in-source). AlarmSource_Update's suppression-window scan AND crossing scan, plus
Arbiter_FireAlarm's index, plus the producer's "did any alarm fire" collision check
now all read state.alarm.alarms / state.alarm.num_alarms -- the single filtered set,
so filtering can neither shift an index nor miscount (the CRITICAL step-2 check).
A type-0 alarm now plays nothing, cancels no speech, and creates no suppression
window. git diff --stat golden = EXACTLY alarm-type-none (of the pre-existing 51).

GOLDEN (alarm-type-none, read fully before blessing): the alarm elev is 2000 m
(DZ_Elev 0), crossed descending between t=550.19 (hmsl 2013.738) and t=550.76
(hmsl 1980.738) -- old code fired the type-0 alarm at t=550.757558 and ran
FS_Speech_Clear, clipping the running vertical-speed speech. Removing that, the
utterance now completes: everything above the crossing is byte-identical; the
speech stream re-phases from the first visible divergence at t=553.589 (digit
alignment shifts by one), and the new trace plays ONE ADDITIONAL token
(2.wav @ 572.716) because the previously-clipped utterance now finishes. END
760.200000 unchanged. TONE-HOLD side effect (B2/#13): NONE VISIBLE here -- the
descent has continuous back-to-back speech that already holds the tone off in
BOTH versions (only 1 BEEP in the whole trace, the t=0.009763 first-fix beep,
byte-identical), and Win_Above/Below are 0 so no suppression-window change. So
the diff is speech-only re-phasing, exactly the card's expected impact.
Full-suite scan: alarm-type-none is the ONLY scenario with an Alarm_Type 0
(grep Alarm_Type:\s*0 over scenarios/ -> 1 file); all other 50 goldens
byte-identical.

NEW SCENARIO (alarm-cancels-speech): the CONTRAST to speech-interrupt (Win 50,
which STOPs on window entry). Here a REAL type-1 (beep) alarm sits at 2500 m with
Win_Above/Below 0 -- NO suppression window, so the speech queue is NOT pre-cleared
before the crossing; the crossing ITSELF is the cancel. Glide-tone config (Model 7,
Mode 2 glide / Mode_2 9), Sp_Rate 1, ONE speech entry Sp_Mode 1 mph Sp_Dec 0 ->
tightly spaced continuous vertical-speed speech. Track: scripts/gen_jump.py normal
jump, exit 4000 m AGL, terminal 50 m/s, freefall-hspeed 15 (so the glide tone
sounds), deploy 1500 m. Trace read before blessing: first-fix beep t=0.009763;
a CLIMB crossing of 2500 m fires an alarm beep at t=510.2 into silence (QUIRKS #1,
climb-alarm; speech/tones threshold-gated during the 5 m/s climb); freefall speech
runs t=811+; the FREEFALL 2500 m crossing at t=843.600 fires the alarm beep which
"(cuts 9.wav)" mid-token and the correct code drops the queued remainder (a fresh
utterance restarts at t=843.731); the glide tone resumes (f0=419 at t=865) once the
speech thins near deploy; END 1157.4. This is exactly M33's observable: dropping
the crossing's FS_Speech_Clear leaves the queued remainder playing, diverging here.

MUTATION (mutation_test.py, now allowed): re-anchored the two mutants my filtered-
list design moved -- M03 "config->alarms[i].elev" -> "state.alarm.alarms[i].elev"
(x2), M32 _M32_OLD/_M32_NEW "config.num_alarms" -> "state.alarm.num_alarms" (the
QUIRKS-#18 comment I inserted sits ABOVE the `if`, outside both anchor blocks, so
no further adjustment). M33's two anchor lines were already unchanged; it is now
KILLED by the new alarm-cancels-speech scenario (its A7 kill via alarm-type-none is
gone by design -- that was the type-0 speech-cancel #18 removes). Full run: 33
killed, 0 needing attention, 0 NO-MATCH; `--only M33` -> KILLED, 1 scenario
(alarm-cancels-speech). Suite 52/52 (new scenario included).

Verification observed:
  cmake --build build       -> audio_sim.exe (clean, no warnings)
  run_tests.py              -> 52 passed, 0 failed, 0 new
  git diff --stat golden    -> alarm-type-none (modified) + alarm-cancels-speech (new)
  mutation_test.py          -> 33 killed, 0 needing attention (0 NO-MATCH), M33 killed
Note (unchanged from B1): the differential fuzzer / audio_sim_ref is frozen OLD
integer code and diverges from the new goldens by design -- NOT used as a gate.

--- Original mid-card BLOCKER report (retained for history; RESOLVED above) ---
Per the card's "STOP and report rather than editing mutation_test.py", the initial
run was 30 killed / 3 needing attention, mutation_test.py not yet allowed:
  * M03 NO-MATCH: anchor moved by the filtered-list rename (re-anchored above).
  * M32 NO-MATCH: anchor moved + inserted comment (re-anchored above).
  * M33 SURVIVED (SEMANTIC): its ONLY kill scenario was alarm-type-none's type-0
    speech-cancel -- the exact behaviour this card removes -- so 0/51 goldens killed
    it; needed a real-alarm-mid-speech scenario (added above). I stopped and
    reported; Michael authorized the amended allowed files and this resolution.

---

## B6 — 2026-07-06 — SUCCESS

Bookkeeping/verification card (QUIRKS #14): confirmed the legacy `'o'/'a'/'b'/'c'`
speech routes (oclock/10/11/12 o'clock, FlySight-1 leftovers) did NOT survive A3's
tokenization, and closed #14. NO firmware change (FlySight/* untouched). Suite 52/52.

Grep evidence (from repo root):
  Select-String -Path FlySight\audio_speech.c,FlySight\audio_control.c -Pattern 'oclock'
    -> zero matches (both files).
  Select-String ... -Pattern "'o'|'a'|'b'|'c'"  (char-route case labels)
    -> zero matches. No `case 'o'/'a'/'b'/'c'` remnant anywhere in FlySight/.
  Grep '\.wav' FlySight/audio_speech.c
    -> the ONLY *.wav literals are the token->filename table (lines 43-78): NUM_0..19,
       TENS_2..9, HUNDRED/THOUSAND, MINUS/DOT, UNIT_*, LABEL_*, SUFFIX_LEFT/RIGHT.
       `10.wav`=TOK_NUM_10 (ten), `11.wav`=TOK_NUM_11 (teens token, required for
       "eleven"), `12.wav`=TOK_NUM_12 -- all legitimate NUM_* entries, NOT the dead
       10/11/12-o'clock routes. (Line 177 `"1X.wav"` is a code COMMENT; line 516 the
       RAW_FILE ".wav" suffix concat.)
  Grep '\.wav' FlySight/audio_control.c
    -> only line 677, the RAW_FILE ".wav" suffix strncat -- no route literal.
  Grep 'oclock' test/golden/  -> zero matches. No golden references oclock.wav.
QUIRKS #14 -> DONE (evidence cited in the row).

CONDITIONAL (QUIRKS #2) -- 11.wav asset has NOT landed. `ls TEMP/AUDIO/11.wav` ->
No such file (10.wav and 12.wav ARE present). Per the card I did NOT bless anything:
`alt-step-feet` is UNCHANGED, git diff -- test/golden/ is EMPTY, count stays 52.
REMINDER FOR THE NEXT AGENT (QUIRKS #2 still open): when Michael's `11.wav` asset
lands in TEMP/AUDIO, re-bless `test/golden/alt-step-feet.trace` -- the 11,000 ft
"eleven" announcement flips PLAYFAIL->PLAY and downstream timestamps cascade; read
the whole diff, confirm it is exactly that one PLAYFAIL->PLAY plus its cascade, then
bless and mark QUIRKS #2 -> DONE. QUIRKS #2 row annotated with this pending state.

No mutation run (no code change). Differential fuzzer NOT used (frozen ref diverges
by design post-B1). Touched only test/QUIRKS.md + this journal.

Verification observed:
  run_tests.py              -> 52 passed, 0 failed, 0 new
  git diff -- test/golden/  -> EMPTY (no golden blessed)
  git diff -- FlySight/     -> EMPTY (no firmware change)

---

## B7 — 2026-07-06 — SUCCESS

Hardened config.c repeated-group parsing (QUIRKS #9). Added `config.num_x > 0`
to all FIVE group-member key guards so a member appearing BEFORE its opener is
ignored instead of writing index -1 (out of bounds): Alarm_Type/Alarm_File
(need num_alarms>0), Win_Bottom (need num_windows>0), Sp_Units/Sp_Dec (need
num_speech>0). Audited the whole parser: alarms / silence windows / speech are
the ONLY repeated-group keys (the legacy `Window` key is a plain dual-write, not
indexed). Added a sparse constraint comment above the Alarm_Type guard. New
scenario `config-malformed` (53rd). Suite 53/53; git diff --stat golden shows
ONLY config-malformed.trace added, zero existing goldens modified; mutation 33
killed / 0 NO-MATCH (config.c is not in the mutant set -- unaffected, as
expected).

DEVIATION from the card's `<=`->`<` suggestion (DELIBERATE, golden-driven).
The card asked to also "correct" `num_x <= FS_CONFIG_MAX_x` to the opener's
`< MAX` bound, claiming it "lets you write x[MAX-1] again after the array is
full." I KEPT `<= MAX` and only ADDED `> 0`, because the card's premise is
wrong for THIS parser: the opener increments num_x BEFORE the member runs and
its own `< MAX` guard caps num_x at MAX, so after a legitimately full group
num_x == MAX and the member must write index MAX-1 (the last entry). Changing
the member to `< MAX` would DROP that last member and CHANGE existing goldens.
Proven concretely: MAX_WINDOWS==2 and `silence-window-metric` defines 2 windows
(window 2 = 900..1200 m is crossed under canopy and its Win_Bottom 900 must be
stored); MAX_SPEECH==3 and five scenarios (speech-alt-mode / -multi-a / -multi-b
/ -nav / -invalid-cfg) define 3 speech entries whose 3rd Sp_Units/Sp_Dec must be
stored. So `<` would violate the CRITICAL "existing goldens UNCHANGED"
constraint. The real defect was the MISSING lower bound; `> 0` is the fix, and
`<= MAX` is the correct (not off-by-one) upper bound. RESIDUAL noted for the
orchestrator: the "[MAX-1] rebind" the card worried about only occurs with MORE
than MAX openers (e.g. a 4th Sp_Mode with MAX_SPEECH==3) -- that overflow opener
is silently dropped (num stays MAX) and its members then rebind to index MAX-1,
corrupting the last accepted entry. A numeric bound change cannot fix this
without breaking the legit-full case (num==MAX is valid for both); a proper fix
needs an opener-side "group full / current-group-invalid" flag. No golden or the
new scenario exercises >MAX openers, and it is pre-existing behaviour (not a
regression from this card), so I left it as documented residual rather than
add uncovered flag logic to a near-frozen file.

QUIRKS #19 clamp DECISION: DECLINED (deferred, not implemented), per the card's
recommendation. B2 already fixes the mode-12 Sp_Dec-0 divide-by-zero at runtime
(step_size==0 -> skip utterance). A parse-time `Sp_Dec >= 1` clamp for Sp_Mode 12
would (a) be a BEHAVIOUR change (a mode-12 Sp_Dec-0 entry would announce instead
of skip) and (b) make the audio_speech.c `decimals != 0` guard's FALSE leg
permanently unreachable, deleting the QUIRKS #20 coverage residue item that C1
is tracking. It changes no existing golden either way (none has mode-12
Sp_Dec-0), but the coverage interaction plus the needless behaviour change make
the clamp unwanted. QUIRKS #19 row annotated with this decision.

NEW SCENARIO (config-malformed): config.txt LEADS with Alarm_Type 2 / Win_Bottom
500 / Sp_Dec 2 (all before any opener; each has a comment noting the old
index -1 write), then normal GPS/tone settings (V_Thresh 10000 = alarms only,
Sp off), then a well-formed Alarm_Elev 2000 / Alarm_Type 1. Track reused from
alarm-hold-at-elev (short descent to 2000 m, then hold and touch-from-below).
Trace read before blessing: first-fix beep (t=0.009763, hmsl 2600) + EXACTLY ONE
alarm beep (t=13.000000, hmsl 2000, f0=1760) + END 33.0 -- i.e. the three
leading strays created NO phantom/-1 alarm (num_alarms ends at 1, one crossing
fires) and the well-formed alarm works. Byte-identical audible result to
alarm-hold-at-elev by design (same track); the malformed keys make no
difference, which is the point.

GOTCHA: `audio_sim.dir/.../config.c.gcda` emits a "overwriting an existing
profile data with a different checksum" libgcov warning after config.c is
recompiled (stale gcda from a prior build) -- harmless, cleared by deleting the
gcda (the A0 "gcda accumulate" note). Not a test failure; run_tests ignores it.
Differential fuzzer / audio_sim_ref NOT used as a gate (frozen ref diverges by
design post-B1, and it is anyway irrelevant to a config.c parser change).

Verification observed:
  cmake --build build       -> audio_sim.exe + audio_sim_ref.exe (clean)
  run_tests.py              -> 53 passed, 0 failed, 0 new
  git diff --stat golden    -> EMPTY (no tracked golden modified)
  git status golden         -> only config-malformed.trace added (untracked)
  mutation_test.py          -> 33 killed, 0 needing attention (0 NO-MATCH)

---

## B7 follow-up — 2026-07-06 — SUCCESS

Michael reviewed the B7 `<= MAX` analysis and ACCEPTED it: config.c is CORRECT
as committed at 15d6d48 and stays unchanged. The `<= MAX` member guards are
load-bearing (a MAX-full group's last member legitimately writes index MAX-1);
the card's `<= MAX -> <` suggestion was wrong. This follow-up only DOCUMENTS the
correctness note and PINS the accepted overflow-rebind residual. NO firmware
change (config.c untouched); touched only test/scenarios/config-malformed/
(config.txt + track.csv), test/golden/config-malformed.trace, test/QUIRKS.md,
this journal.

QUIRKS bookkeeping: narrowed #9's FIXED scope to exactly the member-before-opener
index -1 write, spelled out that `<= MAX` is correct/load-bearing and the `<`
suggestion was NOT applied, marked #9 -> DONE. Added NEW row #21 for the
overflow-rebind residual: with MORE than MAX openers, the overflow opener is
dropped (num_x caps at MAX) but its member keys still write x[MAX-1], rebinding
the last VALID entry's field to the overflow value. In-bounds (not the -1 bug) --
a numeric guard can't fix it (legit-full and overflow both have num_x==MAX);
needs an opener-side overflow flag. Decision: ACCEPTED RESIDUAL (Michael
2026-07-06), out of plan scope, pinned by config-malformed.

EXTENDED SCENARIO (config-malformed) -- reworked so ONE golden pins all three
things. New config: a vertical-speed tone (Mode 1, Max 8000 so the ~4000 cm/s
descent sits mid-scale -> f0=990, distinct from the 1760 Hz alarm/first-fix
beep), constant 1 Hz rate (Min_Rate==Max_Rate==100), Use_SAS 0, V_Thresh 300 so
the tone runs throughout the descent. Two well-formed silence windows
(window 1 [1600,1800], window 2 [1000,1200]) reach MAX_WINDOWS=2, THEN a 3rd
Win_Top 5000 (overflow, dropped) + Win_Bottom 600 rebinds windows[1].bottom
1000 -> 600, extending window 2 to [600,1200]. Leading strays kept
(Alarm_Type 2 / Win_Bottom 500 / Sp_Dec 2 before any opener); well-formed alarm
now Alarm_Elev 1500 / Alarm_Type 1. New synthetic track: steady vertical descent
2000 -> 400 m at velD 40 m/s (201 samples), no horizontal motion.

Trace read line-by-line before re-blessing (29 lines). All three pins visible:
 (a) STRAYS IGNORED: exactly ONE alarm beep in the whole trace (f0=1760 at
     t=12.600000, hmsl 1496 -- the 1500 crossing). The leading Alarm_Type 2
     created no phantom/-1 alarm; num_alarms == 1.
 (b) WELL-FORMED ALARM FIRES: that same f0=1760 beep at hmsl 1496 (crossing
     1500 between the 1504 and 1496 samples, QUIRKS #8-consistent), off the 1 Hz
     tone grid so it is unmistakably the alarm.
 (c) OVERFLOW REBIND OBSERVABLE: the f0=990 tone is active above window 1
     (STOP at t=5.0 hmsl 1800), silent through window 1, active in the gap
     [1200,1600], then STOP at t=20.0 hmsl 1200 and SILENT continuously down to
     hmsl 560 -- i.e. NO tone beeps anywhere in the 600..1000 m band. Tone
     resumes only at hmsl 560 (t=36.17), just below the rebound bottom 600. Had
     the overflow Win_Bottom been ignored, window 2 would be [1000,1200] and the
     tone would sound from ~1000 m down; its absence there is the pin.

No config.c change this commit, so the mutation set (which never targets
config.c, and whose NO-MATCH status depends only on source-anchor strings, not
on scenarios) is unaffected -- a richer scenario can only add kill opportunities,
never remove them; left at the B7 result (33 killed, 0 NO-MATCH). Differential
fuzzer / audio_sim_ref NOT used (frozen ref diverges by design post-B1, and is
irrelevant to a scenario/doc change).

Verification observed:
  run_tests.py              -> 53 passed, 0 failed, 0 new
  git diff --stat golden    -> ONLY config-malformed.trace (26 ins / 3 del), no
                               other golden modified
  git diff -- FlySight/     -> EMPTY (config.c and all firmware unchanged)
