# B3 — Fix Mode_2 7 heading scale (QUIRKS #16)

Status: pending

## Goal

The direction-to-bearing rate channel uses a properly scaled heading
(degrees) like every other consumer. After B1 this should be automatic
(heading is a float degree field in FS_FlightData_t); this card VERIFIES
the fix and re-blesses the two goldens that pinned the bug. If B1
already changed these goldens, this card may reduce to verification +
QUIRKS bookkeeping — that is fine; say so in the commit message.

## Expected golden impact

- `tone-vert-bearing-rate`: beep RATE now varies with the canopy sweep
  (fast when heading is at the bearing, slow when opposite — the
  `180 - |relBearing|` fold feeding min/max rate), instead of being
  pinned at minimum throughout.
- `tone-bearing-north`: the due-north/bearing-0 case keeps max-rate
  behaviour on the northbound legs; check the trace makes geometric
  sense end-to-end.

Sanity check to perform (write result in the commit message): pick two
timestamps in the new tone-vert-bearing-rate trace, compute expected
relative bearing from the track geometry (3 deg/s turn from canopy
start ~572 s, bearing 270), confirm the beep-rate direction matches.

## Allowed files

`FlySight/flight_params.c` (if anything remains),
`FlySight/audio_control.c`,
`test/golden/{tone-vert-bearing-rate,tone-bearing-north}.trace`,
`test/QUIRKS.md`, `plan/audio-rewrite/JOURNAL.md`.

## Definition of done

Suite 50/50; only the two listed goldens changed; QUIRKS #16 → DONE;
geometry sanity check documented.

## Commit message

    Phase B3: correct heading scale in direction-to-bearing rate channel
    (QUIRKS #16)

## Attempt history

(none)
