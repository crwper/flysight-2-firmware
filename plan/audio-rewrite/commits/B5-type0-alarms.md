# B5 — Ignore Alarm_Type 0 completely (QUIRKS #18)

Status: pending

## Goal

Alarms with type 0 are filtered out when AlarmSource initializes from
config: they play nothing, cancel nothing, and do NOT create an alarm
suppression window (Win_Above/Win_Below around a type-0 elevation no
longer silences the tone).

## Expected golden impact

- `alarm-type-none`: the speech cadence no longer clips at the 2000 m
  crossing; otherwise identical. (Its config has Win_Above/Below 0, so
  no suppression-window change is visible there.) **Also (arbiter side
  effect, cf. B2/QUIRKS #13): the no-longer-clipped speech now holds the
  tone (`tone_hold`) longer, so tone beeps near the 2000 m crossing may
  shift or disappear. Trace each such beep change to the extended
  utterance window; expected, not a bug.**
- Scan for any other scenario with a type-0 alarm: none exist today —
  confirm via grep over scenario configs.

## Steps

1. Filter in AlarmSource init; document in the source comment that this
   implements QUIRKS #18.
2. Consider (and note in journal) that alarm COUNT-dependent logic
   (the old `i == num_alarms` collision rule, now arbiter policy) must
   use the filtered list consistently.
3. Suite; read the one diff; re-bless; QUIRKS #18 → DONE.

## Amendment (Michael 2026-07-06 — after implementation surfaced a card defect)

The original allowed-files omitted `test/scripts/mutation_test.py`, but
the state.alarm-filtering rename moves the M03/M32 anchors and, more
importantly, M33 ("alarm crossing cancels queued speech") becomes
SEMANTICALLY unkillable: it tests live behaviour for real (type 1-4)
alarms, but the only golden that killed it was `alarm-type-none` via the
type-0 speech-cancel that #18 removes. Resolution (authorized):
- Add `test/scripts/mutation_test.py` to allowed files; re-anchor M03
  (`config->alarms[i].elev`→`state.alarm.alarms[i].elev`, x2) and M32
  (`config.num_alarms`→`state.alarm.num_alarms`).
- Add NEW scenario `alarm-cancels-speech` that pins real-alarm-mid-speech
  (also becomes M33's kill scenario): jump.csv track; glide-tone config
  with `Sp_Rate 1`, one `Sp_Mode 1` entry (mph, `Sp_Dec 0`), ONE TYPE-1
  alarm at 2500 m, `Win_Above`/`Win_Below` 0 (NO suppression window, so
  the queue is NOT pre-cleared before the crossing — this is what
  distinguishes it from `speech-interrupt`). Expected golden: a
  mid-utterance alarm beep with the queued speech remainder dropped
  ("cuts N.wav"). Read + bless.

## Allowed files

`FlySight/audio_control.c`, `test/scripts/mutation_test.py`,
`test/scenarios/alarm-cancels-speech/` (new: config.txt + track.csv),
`test/golden/alarm-type-none.trace`,
`test/golden/alarm-cancels-speech.trace` (new), `test/QUIRKS.md`,
`plan/audio-rewrite/JOURNAL.md`.

## Definition of done

Suite green (52/52 with the new scenario); `alarm-type-none` changed +
`alarm-cancels-speech` added among goldens; QUIRKS #18 → DONE; mutation
>= 30 killed, 0 NO-MATCH with M33 KILLED (verify `--only M33`).

## Commit message

    Phase B5: ignore Alarm_Type 0 entries entirely (QUIRKS #18)

## Attempt history

(none)
