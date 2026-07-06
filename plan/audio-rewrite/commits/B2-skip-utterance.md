# B2 — Skip whole utterance when value is uncomputable or gated (QUIRKS #12+#13)

Status: pending

## Goal

One rule: if a speech value cannot be produced — division guard (glide
with vD==0, inverse glide with gSpeed==0, or spoken altitude (mode 12)
with Sp_Dec 0 so step_size == 0, QUIRKS #19) or nav gating (End_Nav /
Max_Dist) — the ENTIRE utterance is skipped: no label, no value, no
left/right suffix. This structurally eliminates the uninitialized-tVal
read (#13): the suffix decision only ever runs when a direction value
was actually computed.

Remove the QUIRKS-#12-preservation special case introduced in A3.

## Expected golden impact (exactly these; anything else is a bug)

- `speech-zero-div`: the periodic `glide.wav` label-only utterances
  disappear (silent scenario except first-fix beep).
- `speech-nav`: garbage `right.wav` (and any label-only utterances)
  during gated periods disappear; ungated utterances unchanged.
- `speech-nav-gated`: same — gated-climb utterances (directn + garbage
  suffix) disappear; near-waypoint speech unchanged.
- `speech-invalid-cfg`: the Sp_Mode 8 (invalid mode) entry now also
  skips entirely if you route "unknown mode" through the same
  no-value path — DECIDE: unknown mode ⇒ uncomputable ⇒ skip (recommended,
  same rule); expect its (currently near-silent) utterance slots to
  vanish. If its diff shows anything else, investigate.

## Steps

1. Implement in SpeechSource/audio_speech: utterance built only when
   metrics return valid.
2. Run suite; verify ONLY the four scenarios above differ; read diffs
   against the list; re-bless those four.
3. Update QUIRKS rows 12 and 13 → DONE (and delete the golden-
   instability caveat from #13).
4. Update/replace mutation M30-adjacent anchors if touched.
5. Coverage (QUIRKS #20): ensure a speech scenario exercises a negative
   value while climbing (velD<0 vertical-speed/glide/dive) so the
   `audio_speech.c` MINUS-token branch is covered — a Phase A residue
   ratified for deferral to this card. If none of the re-blessed
   scenarios hits it, extend one (or note it for C1's coverage re-check).

## Allowed files

`FlySight/audio_speech.c/h`, `FlySight/audio_control.c`,
`test/golden/{speech-zero-div,speech-nav,speech-nav-gated,speech-invalid-cfg}.trace`,
`test/QUIRKS.md`, `test/scripts/mutation_test.py`,
`plan/audio-rewrite/JOURNAL.md`.

## Definition of done

Suite 50/50 on updated goldens; only the listed goldens changed
(`git diff HEAD~ --stat -- test/golden/` shows exactly them); QUIRKS
updated; commit message quotes each diff.

## Commit message

    Phase B2: skip entire utterance for uncomputable/gated speech values
    (QUIRKS #12, #13)

## Attempt history

(none)
