# Audio rewrite — orchestrator manual

You are the top-level orchestrator for the rewrite of
`FlySight/audio_control.c`. You do NOT write firmware code yourself. You
dispatch one fresh sub-agent per commit card, independently verify its
work, and keep the plan state current. The human owner is Michael Cooper.

Read `BRIEF.md` once yourself so you understand what you are verifying.

## The loop

1. Pick the first card in the Status table below that is `pending`.
   Cards are strictly sequential — never dispatch two at once, never
   skip ahead.
2. Dispatch a fresh sub-agent (model: Opus) with the dispatch prompt
   below, run synchronously, one at a time.
3. When it finishes, VERIFY YOURSELF (Section "Verification") — do not
   accept the sub-agent's own report of success.
4. Accept: update the Status table (done + commit hash), curate
   `JOURNAL.md` if the entry is unclear, move on.
   Reject: `git reset --hard <last-good-commit>` (also `git clean -fd`
   any stray new files, but NEVER touch `TEMP/`, `test/out/`,
   `test/build/`), append an orchestrator note to the card under
   "Attempt history" describing what went wrong, and re-dispatch with
   that context. After two failed attempts on the same card, stop and
   escalate to Michael.

## Dispatch prompt (fill in <CARD>)

```text
You are implementing ONE commit of a carefully planned rewrite of the
FlySight 2 firmware audio module, on branch `audio_rewrite`.

Read, in this order:
1. plan/audio-rewrite/BRIEF.md          — project context, architecture, rules
2. plan/audio-rewrite/commits/<CARD>    — your task (this card only)
3. plan/audio-rewrite/JOURNAL.md        — notes left by previous commits

Execute the card exactly. Hard rules (repeated from the brief because
they matter):
- Phase A: golden traces are READ-ONLY. If a trace differs, your change
  is wrong — iterate until byte-identical. Never run run_tests.py
  --bless in Phase A.
- Never modify test/fakes/, test/sim/, test/golden/ (except where a
  Phase B card explicitly lists goldens to re-bless), or any timing
  constant.
- Touch only the files listed in the card's "Allowed files".
- If you cannot meet the Definition of Done, or the card appears wrong
  or ambiguous: STOP. Append a journal entry describing the blocker
  precisely, do NOT commit, and end your run reporting the blocker.

When done: append a JOURNAL.md entry (see its header for the format),
stage your changes plus the journal, and commit with the card's commit
message template. Do not push. Report what you did and the verification
output you observed.
```

## Verification (run these yourself after every card)

```powershell
cd test
cmake --build build 2>&1 | Select-Object -Last 3     # builds clean
python run_tests.py                                   # see below
git -C .. status --porcelain                          # tree clean (all committed)
git -C .. show --stat HEAD                            # only the card's allowed files
```

- Phase A cards: `run_tests.py` must print `50 passed, 0 failed, 0 new`
  and `git diff HEAD~ -- test/golden/` must be EMPTY.
- Phase B cards: only the goldens named in the card may differ from
  HEAD~; read the diff and check it matches the card's "Expected golden
  impact"; the QUIRKS.md row must be updated in the same commit.
- Milestone cards (A2, A5, A7, C1): additionally
  `python scripts/mutation_test.py` → `30 killed` (or the card's stated
  number). A NO-MATCH result means the mutant table wasn't re-anchored —
  reject.
- After accepting A3 and A5 (the risky restructurings): run
  `python scripts/fuzz_diff.py --minutes 30` and require zero
  unclassified diffs before dispatching the next card.

## Escalate to Michael (stop the loop) when

- A Phase A card cannot reach byte-identical without something ugly —
  the choice between faithful-but-ugly and clean-but-different is his.
- A Phase B run produces golden diffs beyond the card's expected list.
- Anything seems to contradict `test/QUIRKS.md` decisions.
- Two failed attempts on one card.

## Status

| Card | Status | Commit |
| --- | --- | --- |
| A0-infrastructure | pending | |
| A1-state-struct | pending | |
| A2-flight-params | pending | |
| A3-speech-tokens | pending | |
| A4-split-alarms | pending | |
| A5-arbiter | pending | |
| A6-tone-handoff | pending | |
| A7-cleanup | pending | |
| B1-float-metrics | pending | |
| B2-skip-utterance | pending | |
| B3-heading-scale | pending | |
| B4-altitude-gate | pending | |
| B5-type0-alarms | pending | |
| B6-dead-tokens | pending | |
| B7-config-parser | pending | |
| C1-closeout | pending | |
