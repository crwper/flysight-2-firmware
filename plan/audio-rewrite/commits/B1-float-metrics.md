# B1 — Float/SI metrics (FS_FlightData_t)

Status: pending
Phase B rules apply: golden diffs are EXPECTED but every changed line
must be read, categorized, and summarized in the commit message.

## Goal

Retire the AVR-era integer scaling. Introduce `FS_FlightData_t` (SI
floats; `double` lat/lon; `valid3d`; `vAccGood`), built from
`FS_GNSS_Data_t` in exactly one function. `flight_params.c` internals go
float; the glide ×10000/×100 split collapses into one dimensionless
float ratio; config integers convert to SI once at the module boundary.
Driver-facing outputs (pitch Hz, rate accumulator units, volume) remain
integers, converted at the last step.

## Expected golden impact

Small last-digit / boundary shifts across many tone and speech
scenarios: a pitch off by ±1 Hz where integer truncation and float
rounding disagree; a spoken decimal flipping at a .x5 boundary; an
alarm/step firing one sample earlier/later ONLY where a value sits
exactly on a threshold. Diffs larger than that (different wav, different
event count away from a threshold boundary, timing shifts > 1 GNSS
sample) are BUGS — fix, don't bless.

## Procedure

1. Implement; run suite; expect many FAILs.
2. For EACH failing scenario: read the diff, classify every hunk
   (rounding boundary vs bug). Keep a table in the commit message:
   scenario → N lines changed → classification.
3. Re-bless only after the whole table is "rounding boundary".
4. Update mutation anchors (float constants replace integer ones; keep
   equivalent mutants, e.g. SAS divisor → SAS table value tweak).
5. Note in QUIRKS.md that #5's rate accumulator and beep length remain
   integer/unchanged.

## Allowed files

`FlySight/flight_params.c/h`, `FlySight/audio_control.c`,
`FlySight/audio_speech.c/h`, `test/golden/*.trace` (re-bless per
procedure), `test/scripts/mutation_test.py`, `test/QUIRKS.md`,
`plan/audio-rewrite/JOURNAL.md`.

## Definition of done

- Suite 50/50 against the NEW goldens; mutation ≥30 killed.
- Commit message contains the per-scenario diff classification table.
- No `int32` unit-scaled arithmetic remains in flight_params internals
  (grep for `1024`, `* 100`, `/ 10` and justify any survivor —
  the driver-boundary conversions and the rate accumulator are the
  legitimate ones).

## Stop and report if

- Any diff cannot be traced to a rounding boundary.
- Cumulative diff feels wholesale (hundreds of changed pitches in one
  scenario is fine if each is ±1; different event SEQUENCES are not).

## Mandatory human review

Do NOT commit this card autonomously. When the classification table is
complete, STOP and present it (with the full golden diffs available)
for Michael's review; commit only after his approval. This is the one
card where a wrong judgment silently becomes the new truth.

## Commit message

    Phase B1: float/SI metrics with double lat/lon

    <classification table>

## Attempt history

(none)
