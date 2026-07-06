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

## Allowed files

`FlySight/audio_control.c`, `test/golden/alarm-type-none.trace`,
`test/QUIRKS.md`, `plan/audio-rewrite/JOURNAL.md`.

## Definition of done

Suite green; exactly one golden changed; QUIRKS updated.

## Commit message

    Phase B5: ignore Alarm_Type 0 entries entirely (QUIRKS #18)

## Attempt history

(none)
