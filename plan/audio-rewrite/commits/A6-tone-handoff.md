# A6 — Atomic tone-spec handoff

Status: pending

## Goal

Replace the four independently-written volatiles (`tonePitch`,
`toneChirp`, `toneRate`, `toneHold`-successor) crossing the
producer-task → consumerTimer-ISR boundary with a coherent handoff:

```c
typedef struct { uint32_t pitch; int32_t chirp; uint16_t rate; } ToneSpec_t;
ToneSpec_t slots[2];
volatile uint32_t active;   // producer writes slots[!active], then active = !active
```

A single aligned 32-bit store is atomic on Cortex-M4; the ISR reads
`slots[active]` once into a local. This removes the (sim-invisible)
torn-read race on target.

## Steps

1. Producer side: build the new spec in the inactive slot, flip.
2. consumerTimer: snapshot `active` once per tick, read that slot.
3. Keep the accumulator (`tone_timer += rate`, fire when
   `0x10000 - tone_timer <= rate`) bit-identical.
4. Comment the memory-ordering assumption (single-writer/single-reader,
   aligned word store; no DMB needed for this pattern on M4 — say why).

## Allowed files

`FlySight/audio_control.c`, `plan/audio-rewrite/JOURNAL.md`.

## Definition of done

- `python run_tests.py` → 50 passed, goldens untouched (the sim is
  single-threaded; this change is invisible to it — that is expected).
- Journal entry flags this commit for the eventual HARDWARE listen-test
  (Phase C): the sim cannot validate ISR-concurrency behaviour.

## Commit message

    Make producer->ISR tone handoff atomic via double-buffered spec

## Attempt history

(none)
