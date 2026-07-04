# A7 — Phase A closeout: const config, dead code sweep, milestone checks

Status: pending

## Goal

Finish the restructure: `const FS_Config_Data_t *` end-to-end (QUIRKS
#7 — any local overrides like the direction-mode decimals live in
locals), remove now-unused helpers, confirm nothing mutable exists at
file scope beyond the state instance, and run the full Phase A
milestone verification.

## Steps

1. Constify every config pointer/parameter in audio_control.c,
   audio_speech.c, flight_params.c. The producer may still take a local
   copy if needed for ISR isolation — but nothing WRITES to it.
2. Delete dead remnants (old macros, unused fields). The preserved
   LEFT_RIGHT metrics entry stays (QUIRKS #15).
3. Full mutation re-anchor pass: every MUTANT maps to a live behaviour
   in the new code; extend with 3+ arbiter mutants (swap two priority
   rows; drop the silence-window→ALT_STEP suppression entry; make
   alarm windows suppress SPEECH too). Target: 30+ mutants, all killed.
4. Coverage check of the NEW files:
   ```powershell
   Get-ChildItem -Recurse build -Filter *.gcda | Remove-Item
   python run_tests.py
   python -m gcovr -r .. build --filter ".*audio_control|.*flight_params|.*audio_speech" --txt-metric branch
   ```
   Requirement: 100% of reachable branches (allowed residue: the
   LEFT_RIGHT metrics entry only). If restructuring created new
   unreachable branches, either they are justified-and-documented in
   the journal or the code is simplified until they are gone.
5. Overnight-scale fuzz (orchestrator runs it; you just make sure the
   build is ready): `python scripts/fuzz_diff.py --minutes 120` —
   zero unclassified diffs.

## Allowed files

`FlySight/audio_control.c`, `FlySight/audio_speech.c/h`,
`FlySight/flight_params.c/h`, `test/scripts/mutation_test.py`,
`plan/audio-rewrite/JOURNAL.md`.

## Definition of done

- Suite 50/50 byte-identical; mutation ≥30 killed 0 NO-MATCH; coverage
  as above; fuzz clean.

## Commit message

    Phase A closeout: const config, dead-code sweep, extended mutants

## Attempt history

(none)
