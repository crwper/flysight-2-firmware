# C1 — Closeout

Status: pending

## Goal

Final quality gates, remove scaffolding, hand back to Michael.

## Steps

1. Mutation: full re-anchor + extension pass against the final code —
   ≥33 mutants including the arbiter-table mutants from A7; all killed.
2. Coverage (fresh gcda): 100% reachable lines+branches across
   `audio_control.c`, `flight_params.c`, `audio_speech.c`. Allowed
   residue (updated to carry forward prior Michael decisions):
   - the reserved LEFT_RIGHT metrics entry (QUIRKS #15); AND
   - `FS_FlightParams_GetSASCorrectionFactor` — develop-branch-merge
     scaffolding, intentionally unused on this branch, EXEMPT from the
     100% rule (ratified A7 / QUIRKS #20b).
   Everything else must be 100%. Known residue to CLOSE here (QUIRKS #20
   orchestrator note, post-B4): `audio_speech.c` had ~3 uncovered
   branches — the QUIRKS-#19 `decimals != 0` guard FALSE leg (reachable
   only via a mode-12 `Sp_Dec 0` entry) and two `labelToken`/Step-3-suffix
   `switch` cases for label/unit modes no scenario config exercises. C1
   may NOT edit firmware (FlySight/* is frozen at closeout), so close
   these by ADDING scenarios (a mode-12 Sp_Dec-0 config; configs that
   exercise the missing label/unit modes). If any such branch is
   genuinely UNREACHABLE from a parser-valid config (e.g. tied to the
   reserved mode 10), STOP and report it as a residue decision rather
   than blessing an unexplained gap.
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
