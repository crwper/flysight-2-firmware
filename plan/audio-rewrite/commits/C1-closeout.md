# C1 — Closeout

Status: pending

## Goal

Final quality gates, remove scaffolding, hand back to Michael.

## Steps

1. Mutation: full re-anchor + extension pass against the final code —
   ≥33 mutants including the arbiter-table mutants from A7; all killed.
2. Coverage (fresh gcda): 100% reachable lines+branches across
   `audio_control.c`, `flight_params.c`, `audio_speech.c`. Allowed
   residue: the reserved LEFT_RIGHT metrics entry (QUIRKS #15) only.
   Any other residue: add scenarios or simplify code.
3. Fuzz: the reference comparison is retired (post-B1 it diffs on
   every rounding boundary by design). Convert `fuzz_diff.py` into
   `fuzz_crash.py` mode or simply document in test/README.md that
   differential fuzzing served Phase A and is closed; delete
   `test/reference/` and the `audio_sim_ref` CMake target.
4. Update `test/README.md`: rewrite-era notes (three source files, how
   to extend the token table / add a source / add an arbiter rule —
   a short "how to add a new audio source" section is the payoff of
   this whole project; write it).
5. QUIRKS.md: every row has a final state (PRESERVED / DONE /
   asset-pending). Leave #2 open iff 11.wav still absent.
6. Journal closing entry: summary of the whole run, open items.
7. HANDOFF TO MICHAEL (list in the commit message / journal):
   - Hardware listen-test checklist: init speech + file modes;
     first-fix beep; glide tone through a real jump; one alarm; one
     altitude announcement; A6 handoff under real ISR load (listen for
     pitch/chirp glitches during rapid config value swings).
   - `11.wav` asset (if still pending) → re-bless `alt-step-feet`.
   - develop merge notes: enable_nav gate → one `valid=false` condition
     in FS_FlightParams_Get + on/off scenarios; ActiveLook switches to
     FS_FlightParams_* (their flight_params.c stub is superseded —
     verify ActiveLook displayed values pre/post); harness config.c
     compilation will need a fake FS_State_Get for Device_ID.

## Allowed files

`test/` (README, scripts, CMakeLists, reference removal),
`test/QUIRKS.md`, `plan/audio-rewrite/JOURNAL.md`, `plan/audio-rewrite/README.md`
(status table completion).

## Definition of done

Suite green; mutation ≥33/33; coverage clean; `test/reference/` gone;
docs updated; handoff list written.

## Commit message

    Phase C: rewrite closeout - final gates, scaffolding removal, handoff notes

## Attempt history

(none)
