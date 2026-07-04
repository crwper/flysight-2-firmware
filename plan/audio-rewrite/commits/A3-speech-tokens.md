# A3 — Tokenize speech (audio_speech.c)

Status: pending
Risk: HIGHEST of Phase A. Iterate against the suite; expect several
rounds. Orchestrator runs a 30-minute fuzz after acceptance.

## Goal

Replace the sentinel-character bytecode (`speech_buf`/`speech_ptr`,
`numberToSpeech`, the ~150-line if/else ladder in `consumerTask`) with
an explicit token queue and a token→filename table, byte-identical.

## Token design (from BRIEF.md)

Enum tokens: `NUM_0..NUM_19`, `TENS_2..TENS_9` (→ "20.wav".."90.wav"),
`HUNDRED` (00.wav), `THOUSAND` (000.wav), `MINUS`, `DOT`,
`UNIT_METERS/FEET/KM/MILES/KNOTS_FILE` ('K'→km.wav, 'i'→miles.wav,
'n'→knots.wav), `LABEL_HORZ/VERT/GLIDE/IGLIDE/SPEED/DIRECTN/DISTANCE/
BEARING/DIVE/ALT`, `SUFFIX_LEFT/RIGHT`, `RAW_FILE` (carries a filename,
for Init_Mode 2 and alarm type 4 — check: alarm files currently bypass
the speech buffer via FS_Audio_Play; keep that. RAW_FILE is for
Init_Mode 2's '/' mechanism). Do NOT create tokens for the legacy
`o/a/b/c` routes (QUIRKS #14; they are dead).

## Cadence contract (what byte-identical means here)

- The consumer starts ONE wav per idle consumer tick while the queue is
  non-empty (matches the old one-char-or-two-per-tick interpreter:
  `t`/`x`/`>` consumed two chars but still played one wav).
- Numbers: the old digit-string path (values via `writeInt32ToBuf`)
  plays digit-by-digit ("81" = 8.wav, 1.wav); `numberToSpeech` (altitude
  announcements) plays teens/tens/hundreds/thousands words. Both
  encoders must produce token sequences that map to the IDENTICAL wav
  sequences as today, including "0" → 0.wav, negative → MINUS first,
  teens → "1X.wav", tens with remainder → "X0.wav" then digit.
- Cancellation (`*speech_ptr = '\0'` today) becomes queue-clear: current
  wav finishes, queued tokens dropped. Same call sites.
- Labels: `>` + mode+1 chars become one LABEL token (played on one
  tick, exactly like today).
- QUIRKS #12 MUST BE REPRODUCED BUG-FOR-BUG in this commit: an
  empty/uncomputable value with Sp_Dec 2 plays the label alone; with
  Sp_Dec 1 the WHOLE utterance is silent (old code overwrote the label
  byte); with Sp_Dec 0 label alone. Recommended approach: build the
  utterance with the same "truncate N trailing tokens by decimals"
  arithmetic the old buffer used, so the pathology falls out naturally;
  a hand-coded special case is acceptable if commented as
  QUIRKS-#12-preservation (removed in B2).
- The uninitialized-tVal l/r suffix (QUIRKS #13): the suffix
  emission logic keeps today's structure (append SUFFIX_LEFT/RIGHT
  based on the same possibly-uninitialized variable). If goldens
  `speech-nav`/`speech-nav-gated` flip l/r because stack layout moved:
  STOP and report (see below).

## Steps

1. Create `FlySight/audio_speech.c/h`: token enum, token→filename table,
   `FS_Speech_QueueNumber…`/utterance-building helpers, queue state (in
   a struct owned by audio_control's state for now), renderer function
   `FS_Speech_PlayNext()` called from consumerTask when idle.
2. Rewrite `speakValue`, the alt announcements in `updateAlarms`, the
   ground-elevation announcement, and Init_Mode 1/2 to build token
   utterances.
3. Delete `numberToSpeech` and the consumerTask ladder.
4. Update `test/CMakeLists.txt` (audio_speech.c → audio_sim target only;
   the reference target keeps compiling without it — verify).
5. Re-anchor mutation strings that moved (M26 teens boundary, M27 label
   index, M28 announcement scale now live in audio_speech.c/new call
   sites).

## Allowed files

`FlySight/audio_speech.c/h` (new), `FlySight/audio_control.c`,
`test/CMakeLists.txt`, `test/scripts/mutation_test.py`,
`plan/audio-rewrite/JOURNAL.md`.

## Definition of done

- `python run_tests.py` → 50 passed, goldens untouched (byte-identical
  timing included — a wav starting one tick late is a failure).
- `python scripts/mutation_test.py` → 30 killed, 0 NO-MATCH.

## Stop and report if

- `speech-nav` / `speech-nav-gated` differ ONLY in l/r suffixes while
  everything else is byte-identical. That is QUIRKS #13's stack-garbage
  dependence, not your bug. Do not bless; report to the orchestrator —
  Michael decides (options: accept new garbage as golden with a QUIRKS
  note, or pull B2 forward).
- Any cadence difference you cannot eliminate without reproducing the
  old buffer byte arithmetic wholesale.

## Commit message

    Replace speech bytecode with token queue (audio_speech.c)

## Attempt history

(none)
