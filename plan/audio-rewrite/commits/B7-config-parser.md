# B7 — config.c repeated-group parser hardening (QUIRKS #9)

Status: pending
Note: this is the only card that touches `FlySight/config.c`.

## Goal

A group-member key arriving before its group opener must be ignored
instead of writing to index −1:

- `Alarm_Type` / `Alarm_File` require a preceding `Alarm_Elev`
  (num_alarms > 0)
- `Win_Bottom` requires `Win_Top` (num_windows > 0)
- `Sp_Units` / `Sp_Dec` require `Sp_Mode` (num_speech > 0)

Audit for the same pattern in any other grouped keys on this branch.
Also fix the related `<= MAX` off-by-one guards (`num_alarms <=
FS_CONFIG_MAX_ALARMS` allows writing alarms[MAX-1] again after the
array is full — verify and correct to match the opener's `<` bound).
Also consider clamping Sp_Dec >= 1 for Sp_Mode 12 (QUIRKS #19): Sp_Dec 0
makes the mode-12 step_size zero and divides by it.

## New scenario required

`config-malformed`: a config.txt that leads with `Alarm_Type: 2`,
`Win_Bottom: 500`, `Sp_Dec: 2` before any openers, then a normal alarm;
ground or short jump track. Pins: the strays are ignored, the wellformed
alarm still works. Read + bless.

## Expected golden impact

None on existing scenarios (all their configs are well-formed). Only
the new scenario's golden is added.

## Allowed files

`FlySight/config.c`, `test/scenarios/config-malformed/` (new),
`test/golden/config-malformed.trace` (new), `test/QUIRKS.md`,
`plan/audio-rewrite/JOURNAL.md`.

## Definition of done

Suite green including the new scenario; existing goldens untouched;
QUIRKS #9 → DONE.

## Commit message

    Phase B7: ignore group-member config keys before their group opener
    (QUIRKS #9)

## Attempt history

(none)
