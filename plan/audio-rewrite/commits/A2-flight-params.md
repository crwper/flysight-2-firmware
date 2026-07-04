# A2 — Extract flight_params.c (metrics layer, integer math verbatim)

Status: pending

## Goal

One shared implementation of flight-metric computation, used by both the
tone path (`getValues`) and the speech path (`speakValue`). Integer math
preserved EXACTLY — this commit moves code, it does not renumber it.

## Steps

1. Create `FlySight/flight_params.c/h` (GPL header like the other
   files). Move into it:
   - the SAS table + interpolation (verbatim; it is byte-identical in
     both current call sites),
   - a value-computation function per the architecture:
     `FS_FlightParams_GetValue(mode, const FS_GNSS_Data_t*, const
     FS_Config_Data_t*, int32_t scale, int32_t *val, int32_t *min,
     int32_t *max)` or equivalent. The `scale` parameter is REQUIRED:
     the tone path computes glide as `10000*gSpeed/velD` and speech as
     `100*gSpeed/velD`; integer division means these must stay separate
     computations. Do NOT try to unify the scales (that is B1).
   - the nav gating (Max_Dist / End_Nav / Min_Angle logic) exactly as
     it appears today — including its quirks (mode_2-dependent deadband
     condition; the direction fold).
2. `getValues` and `speakValue` become thin adapters calling
   flight_params. `speakValue` keeps its OWN quirks locally: the
   `decimals = 0` local override (write to a local, not the config —
   but note the config here is already a copy; keep observable
   behaviour identical either way), the distance unit conversions and
   `tVal + 5` rounding, and the (buggy, preserved) uninitialized-tVal
   l/r suffix behaviour — the suffix decision stays in audio_control.c
   using a variable whose initialization state mirrors today's.
3. Provide `double FS_FlightParams_GetSASCorrectionFactor(int32_t
   hMSL)` — develop-branch-compatible signature, implemented from the
   same table (linear interpolation, /1024.0). Unused on this branch;
   it exists so the develop merge deletes their copy.
4. Add `flight_params.c` to `test/CMakeLists.txt` (both targets — the
   reference target needs it too since audio_control_orig.c does NOT
   use it; only add to `audio_sim`... check: the reference file is
   self-contained, so add flight_params.c to the shared source list; it
   is harmless to link into both).
5. Re-anchor `test/scripts/mutation_test.py`: M17 (glide scale) and M18
   (SAS divisor) now live in flight_params.c — the script mutates
   `FlySight/audio_control.c` only, so either extend it to patch
   multiple files (preferred: give each MUTANT an optional file field)
   or re-target equivalent mutations. Keep 30 meaningful mutants.

## Allowed files

`FlySight/flight_params.c/h` (new), `FlySight/audio_control.c`,
`test/CMakeLists.txt`, `test/scripts/mutation_test.py`,
`plan/audio-rewrite/JOURNAL.md`.

## Definition of done

- `python run_tests.py` → 50 passed, goldens untouched.
- `python scripts/mutation_test.py` → 30 killed, 0 NO-MATCH.
- `audio_sim_ref` still builds and passes
  (`python run_tests.py --exe build/audio_sim_ref.exe`).

## Stop and report if

- Any integer result differs from the old code for any input in the
  suite (visible as a golden diff). Likely causes: accidentally unified
  glide scales, reordered a division, or changed a min/max in-out
  mutation order.

## Commit message

    Extract flight_params metrics layer (integer math verbatim)

## Attempt history

(none)
