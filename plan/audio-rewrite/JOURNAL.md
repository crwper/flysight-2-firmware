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
