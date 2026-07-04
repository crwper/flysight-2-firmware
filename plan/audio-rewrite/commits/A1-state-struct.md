# A1 — Single state struct

Status: pending

## Goal

Move ALL mutable module state into one `FS_AudioControl_State_t`,
zeroed and explicitly initialized in `FS_AudioControl_Init`. Fixes
QUIRKS #6 (stale CHANGE_IN_VALUE_1 history across DeInit/Init) as an
invisible side effect.

## Steps

1. In `FlySight/audio_control.c`, define a struct containing every
   current file-scope static (`timer_id`, `cur_speech`, `sp_counter`,
   `flags`, `prev_flags`, `prevHMSL`, `g_suppress_tone`, `speech_buf`,
   `speech_ptr`, `tonePitch`, `toneChirp`, `toneRate`, `toneHold`) AND
   the function-local statics: `x0/x1/x2` from `updateTones`,
   `tone_timer` from `consumerTimer`.
2. One static instance `static FS_AudioControl_State_t state;`. Members
   read from the timer ISR (`tonePitch`, `toneChirp`, `toneRate`,
   `toneHold`) keep their `volatile` qualifier inside the struct.
3. `FS_AudioControl_Init`: `memset(&state, 0, sizeof(state))`, then the
   existing explicit initializations, PLUS `x0 = INVALID_VALUE` (this
   reproduces the boot-time initializer; x1/x2 zero is what the old
   zero-initialized statics had, and the INVALID propagation keeps the
   first two samples gated exactly as before — see QUIRKS #6 analysis).
4. Mechanical rename of all references. No logic changes whatsoever.

## Allowed files

`FlySight/audio_control.c`, `plan/audio-rewrite/JOURNAL.md`.

## Definition of done

- `python run_tests.py` → 50 passed; `git diff -- test/golden/` empty.
- No file-scope or function-local statics remain in audio_control.c
  except the single `state` instance (grep for `static` to confirm;
  `static` FUNCTIONS are fine, static const tables are fine).

## Stop and report if

- Any golden differs (including `speech-nav`/`speech-nav-gated`: this
  commit must not move stack layout in `speakValue`; if their l/r
  output flips, report rather than bless).

## Commit message

    Consolidate audio control state into a single struct

    Behaviour-identical on all 50 goldens. Also resets CHANGE_IN_VALUE_1
    history on Init (QUIRKS #6) - invisible to the harness by design.

## Attempt history

(none)
