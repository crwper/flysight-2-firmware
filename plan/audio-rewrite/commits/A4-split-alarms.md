# A4 — Split updateAlarms into AlarmSource + AltModeSource

Status: pending

## Goal

Separate the two concerns tangled in `updateAlarms`: alarm crossing/
window logic (AlarmSource) and altitude-mode announcements + silence
windows (AltModeSource). Each gets a struct in the module state and one
update function. Evaluation order and every side effect preserved.

## Order to preserve (from the current code, per producer sample)

1. Alarm-window scan → suppress_tone.
2. Silence-window scan → suppress_tone + suppress_alt.
3. Alt_Step window check → suppress_tone (with the ALT_MIN floor).
4. On suppress_tone rising edge: clear speech queue, rate 0,
   `FS_Audio_Stop`.
5. If prev sample had fix: alarm crossing scan ([min,max) interval,
   QUIRKS #8); on fire: sound per type (type 0 = silent, STILL cancels
   speech — preserved until B5), cancel queued speech.
6. Alt_Step announcement: only if no alarm fired this sample
   (`i == num_alarms`), above ALT_MIN, queue empty, no pending
   say-altitude flag, not suppress_alt, and both speed thresholds met.

Keep the FLAG_SAY_ALTITUDE / FLAG_VERTICAL_ACC interactions where they
are (they move into sources in A5, not here).

## Steps

1. Define `AlarmSource_t { int32_t prevHMSL; }`-ish and
   `AltModeSource_t` state (whatever fields the split needs — prevHMSL
   is shared input; pass the sample pair explicitly rather than
   duplicating state).
2. Two functions with explicit parameters (state, config, current
   sample, outputs for suppress flags / requests). `producerTask` calls
   them in the order above.
3. No behaviour changes; suppression flags may become return values but
   their consumers see identical values in identical order.

## Allowed files

`FlySight/audio_control.c`, `plan/audio-rewrite/JOURNAL.md`
(and `test/scripts/mutation_test.py` if anchors moved).

## Definition of done

- `python run_tests.py` → 50 passed, goldens untouched.

## Stop and report if

- You find yourself wanting to "fix" the type-0 cancel or the window
  edge conventions — those are B5 / frozen decisions.

## Commit message

    Split alarm and altitude-mode logic into separate sources

## Attempt history

(none)
