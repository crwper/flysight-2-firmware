# B6 — Confirm dead-token removal (QUIRKS #14) — bookkeeping card

Status: pending

## Goal

Verify the legacy `o/a/b/c` speech routes (oclock/10/11/12.wav) did not
survive A3 in any form, that `11.wav` remains reachable via the teens
token (NUM_11 → "11.wav" — required for "eleven"), and close QUIRKS #14.

## Steps

1. Grep audio_speech.c/audio_control.c for oclock/`"10.wav"`-style
   literals; confirm only the token table's legitimate entries exist
   (10.wav IS legitimate — it is NUM_10/"ten"; oclock.wav must be gone).
2. Confirm no golden references oclock.wav (grep test/golden/).
3. QUIRKS #14 → DONE. If Michael's `11.wav` asset has landed in
   TEMP/AUDIO by now, also re-bless `alt-step-feet` (PLAYFAIL becomes
   PLAY + downstream timestamps shift) and note QUIRKS #2 done;
   otherwise leave a journal reminder.

## Allowed files

`test/QUIRKS.md`, `plan/audio-rewrite/JOURNAL.md`
(+ `test/golden/alt-step-feet.trace` iff the asset landed).

## Definition of done

Suite green; QUIRKS updated; greps documented in commit message.

## Commit message

    Phase B6: confirm legacy speech token routes removed (QUIRKS #14)

## Attempt history

(none)
