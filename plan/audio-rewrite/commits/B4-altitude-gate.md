# B4 — Altitude-indication gate refinement (QUIRKS #17)

Status: pending

## Goal

Only ALTITUDE indications wait for the ground-elevation announcement:
Alt_Step announcements and Sp_Mode-12 speech entries check
AltModeSource's `ground_elev_announced` latch. Non-altitude speech
(vertical speed, glide, direction, distance…) plays regardless. Alarms
remain fully unconditional (no vAcc, no announcement gating) — decided.

Mechanism note: mode-12 rotation entries are SKIPPED (rotation moves on,
like the existing ALT_MIN skip), not blocking.

## Expected golden impact

- `alt-vacc-never`: gains periodic vertical-speed speech throughout the
  jump (previously zero PLAY events); ground-elevation announcement and
  all step announcements remain absent (vAcc never improves). First-fix
  beep unchanged.
- `alt-vacc-gate`: UNCHANGED (announcement precedes everything there) —
  verify explicitly.
- Check whether any other scenario has speech configured while the
  announcement is pending: scan all goldens for diffs; expected-none
  beyond the above because every other track reaches vAccGood at the
  first sample. If one appears, analyze before blessing.

## Coverage (QUIRKS #20)

The new `alt-gate-mixed` scenario (Alt_Step + speech) is a natural fit to
cover the Phase A residue branch `audio_control.c:771` (Alt_Step crossing
skipped because speech is already queued). Confirm it does — if not,
tweak the scenario timing so a step crossing lands while speech is
pending. Flag the result for C1's coverage re-check.

## New scenario required

Add `alt-gate-mixed`: vAcc poor for 30 s (track via
`gen_jump.py --vacc-poor 30`), config with Alt_Step AND two speech
entries (mode 12 + mode 1), Sp_Rate small. Pins: mode-1 speech plays
during the poor-vAcc window, mode-12 and step announcements start only
after the ground announcement. Bless after reading; describe in the
scenario's config comment.

## Allowed files

`FlySight/audio_control.c`, `FlySight/audio_speech.c/h` (if needed),
`test/scenarios/alt-gate-mixed/` (new),
`test/golden/{alt-vacc-never,alt-gate-mixed}.trace`, `test/QUIRKS.md`,
`plan/audio-rewrite/JOURNAL.md`.

## Definition of done

Suite 51/51 (new scenario included); only listed goldens changed/added;
QUIRKS #17 → DONE.

## Commit message

    Phase B4: gate only altitude indications on ground-elevation
    announcement (QUIRKS #17)

## Attempt history

(none)
