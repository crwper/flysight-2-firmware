# A5 — SpeechSource, StartupSource, and the arbiter

Status: pending
Risk: HIGH (with A3). Orchestrator runs a 30-minute fuzz after
acceptance.

## Goal

Complete the source decomposition and introduce the arbiter: the single
owner of "what is audible", the only caller of `FS_Audio_*`, with the
priority/suppression policy as data. The old `flags` word, `toneHold`
and `g_suppress_tone` dissolve into source/arbiter state.

## Mapping old → new (all behaviour-identical)

- `FLAG_HAS_FIX/prev_flags` → producer-level fix tracking feeding
  sources.
- `FLAG_FIRST_FIX/FLAG_BEEP_DONE` → StartupSource state; the first-fix
  beep still waits until the speech queue is empty AND driver idle
  (QUIRKS #3).
- `FLAG_SAY_ALTITUDE/FLAG_VERTICAL_ACC` → AltModeSource latch +
  producer vAcc tracking; the pending ground-elevation announcement
  still blocks Alt_Step announcements AND all periodic speech (current
  behaviour; the #17 refinement happens in B4, not here).
- `toneHold` → arbiter rule "speech active suppresses TONE".
- `g_suppress_tone` edge → arbiter "entering a suppression zone stops
  playback + clears queued speech" (today's exact combination).
- Priority table: `ALARM > STARTUP > ALT_GROUND_ELEV > ALT_STEP >
  SPEECH > TONE`. In Phase A this table must reproduce the CURRENT
  emergent precedence exactly; if you find a pairwise case where the
  table and the old code disagree, the OLD CODE wins — adjust the
  policy entry and note it in the journal (candidate future decision).
- consumerTimer keeps its exact accumulator logic; it now reads the
  ToneSpec granted by the arbiter.

## Steps

1. SpeechSource: rotation index, sp_counter accumulation
   (`+= config->rate` per producer run, threshold `>= sp_rate`),
   ALT_MIN skip for mode-12 entries.
2. StartupSource: init speech/file queuing at Init, first-fix beep.
3. Arbiter module-within-the-file: request inbox from sources per
   producer sample + per consumer tick grant logic.
4. Remove the dissolved globals from the state struct.
5. Re-anchor mutation strings (many will have moved: M13, M23, M24,
   M25, M29 at least).

## Allowed files

`FlySight/audio_control.c`, `FlySight/audio_speech.c/h` (if the queue
ownership moves), `test/scripts/mutation_test.py`,
`plan/audio-rewrite/JOURNAL.md`.

## Definition of done

- `python run_tests.py` → 50 passed, goldens untouched.
- `python scripts/mutation_test.py` → 30 killed, 0 NO-MATCH.
- Grep check: `FS_Audio_` appears only in the arbiter section (and the
  fake, obviously).

## Stop and report if

- Any pairwise precedence case cannot be expressed in the table without
  a special case — document the special case in the journal and keep
  it (do not change behaviour to make the table prettier). If more
  than ~3 special cases accumulate, stop: the table design may need
  revisiting with Michael.

## Commit message

    Introduce audio arbiter with explicit priority and suppression table

## Attempt history

(none)
