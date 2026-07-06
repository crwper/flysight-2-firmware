#!/usr/bin/env python3
"""Mutation pass: deliberately break audio_control.c one mutation at a
time, rebuild, run the golden suite, and require at least one FAIL.

A KILLED mutant means the suite detects that class of rewrite mistake.
A SURVIVED mutant means a behaviour change the suite CANNOT see -- either
an equivalent mutant (no observable behaviour change is possible) or a
real assertion gap that needs a scenario.

The source file is restored (and rebuilt) no matter what; verify with
`git diff FlySight/audio_control.c` afterwards.

Usage: python scripts/mutation_test.py [--only M01,M07]
"""

import subprocess
import sys
from pathlib import Path

TEST = Path(__file__).resolve().parent.parent
SRC = TEST.parent / "FlySight" / "audio_control.c"
# Some metric mutations (glide scale, SAS interpolation) were extracted to
# flight_params.c in card A2; the speech encoders/labels/announcement scale
# moved to audio_speech.c in card A3. A mutant may name the file it patches.
FP = TEST.parent / "FlySight" / "flight_params.c"
AS = TEST.parent / "FlySight" / "audio_speech.c"

# --- Card A7 arbiter mutants (M31-M33) ---
# Multi-line anchors are defined here for readability. M32 swaps the CH_ALARM and
# CH_ALT_STEP "priority rows" in producerTask: on a same-sample alarm+alt-step
# collision the step is announced instead of the alarm firing.
_M32_OLD = (
    "\t\t\t\tif (fired_index != state.alarm.num_alarms)\n"
    "\t\t\t\t{\n"
    "\t\t\t\t\tArbiter_FireAlarm(&config, fired_index);\n"
    "\t\t\t\t\tFS_Speech_Clear(&state.speech);\n"
    "\t\t\t\t}\n"
    "\t\t\t\telse if (want_alt_step && !FS_Speech_HasPending(&state.speech))\n"
    "\t\t\t\t{\n"
    "\t\t\t\t\tFS_Speech_BuildAltStep(&state.speech, &config, step);\n"
    "\t\t\t\t}"
)
_M32_NEW = (
    "\t\t\t\tif (want_alt_step && !FS_Speech_HasPending(&state.speech))\n"
    "\t\t\t\t{\n"
    "\t\t\t\t\tFS_Speech_BuildAltStep(&state.speech, &config, step);\n"
    "\t\t\t\t}\n"
    "\t\t\t\telse if (fired_index != state.alarm.num_alarms)\n"
    "\t\t\t\t{\n"
    "\t\t\t\t\tArbiter_FireAlarm(&config, fired_index);\n"
    "\t\t\t\t\tFS_Speech_Clear(&state.speech);\n"
    "\t\t\t\t}"
)

# M33 tests the alarm->speech suppression axis (card #3, "alarm windows suppress
# SPEECH too"). The literal mask edit is EQUIVALENT under A5's arbiter: every
# SUP_TONE zone -- including alarm windows -- already clears the speech queue on
# entry via Arbiter_ApplySuppression, so widening the alarm-window mask changes
# nothing observable. The observable dual is the alarm CROSSING's speech-cancel;
# dropping it is killed by alarm-type-none, where a Type-0 alarm's ONLY effect is
# that cancel (QUIRKS #18).
_M33_OLD = (
    "\t\t\t\t\tArbiter_FireAlarm(&config, fired_index);\n"
    "\t\t\t\t\tFS_Speech_Clear(&state.speech);"
)
_M33_NEW = "\t\t\t\t\tArbiter_FireAlarm(&config, fired_index);"

# (id, expected occurrence count, old, new, description[, file])
# The optional 6th field is the source file to patch (default: audio_control.c).
MUTANTS = [
    # --- alarm crossing / windows (competition-critical) ---
    ("M01", 1, "alarm_elev >= min && alarm_elev < max",
               "alarm_elev > min && alarm_elev < max",
     "alarm crossing: exclude exact-hit lower bound"),
    ("M02", 1, "alarm_elev >= min && alarm_elev < max",
               "alarm_elev >= min && alarm_elev <= max",
     "alarm crossing: include upper bound (fires one sample early on ascent)"),
    ("M03", 2, "state.alarm.alarms[i].elev + config->dz_elev",
               "state.alarm.alarms[i].elev",
     "alarm elevation: drop dz_elev offset"),
    ("M04", 1, "alarm_elev + config->alarm_window_above",
               "alarm_elev + config->alarm_window_below",
     "alarm window: use below-window on the above side"),
    ("M05", 1, "FS_Audio_Beep(TONE_MIN_PITCH, TONE_MAX_PITCH, 125, config->volume * 5)",
               "FS_Audio_Beep(TONE_MAX_PITCH, TONE_MIN_PITCH, 125, config->volume * 5)",
     "alarm type 2: chirp direction reversed"),
    ("M06", 1, "#define ALT_MIN         1500L",
               "#define ALT_MIN         150L",
     "ALT_MIN: altitude-mode floor 1500 -> 150 m"),
    ("M07", 1, "* 10 + step_size / 2) / step_size",
               "* 10 + step_size / 3) / step_size",
     "alt step rounding: half-step -> third-step", AS),
    ("M08", 1, "(config->windows[i].bottom + config->dz_elev <= current->hMSL)",
               "(config->windows[i].bottom + config->dz_elev > current->hMSL)",
     "silence window: bottom condition inverted (window never matches)"),
    ("M09", 1, "(config->windows[i].top + config->dz_elev >= current->hMSL)",
               "(config->windows[i].top + config->dz_elev > current->hMSL)",
     "silence window: top boundary exclusive (equality leg)"),
    # --- thresholds / flags ---
    ("M10", 1, "if (ABS(velD) >= config->threshold &&",
               "if (ABS(velD) > config->threshold &&",
     "V_Thresh: boundary equality (tone gate)"),
    # M11/M12 re-anchored in card B1: the fix / vAcc gates moved into
    # FS_FlightData_FromGNSS (valid3d / vAccGood) in flight_params.c.
    ("M11", 1, "g->vAcc < 10000",
               "g->vAcc < 100000",
     "vAcc gate: 10 m -> 100 m (FromGNSS)", FP),
    ("M12", 1, "g->gpsFix == 3",
               "g->gpsFix >= 2",
     "fix gate: accept 2D fix (FromGNSS)", FP),
    ("M13", 1, "state.startup.beep_done = true;",
               ";",
     "first-fix beep: never marked done (repeats)"),
    # --- tone mapping ---
    ("M14", 1, "(val_2 - min_2) / (max_2 - min_2))",
               "(val_2 - max_2) / (max_2 - min_2))",
     "rate interpolation: wrong origin"),
    # M15 re-anchored in card B1: the in-band pitch is now rounded from a float
    # ratio (was integer truncation), so the interpolation carries (float) casts.
    ("M15", 1, "(float) (val_1 - min_1) / (float) (max_1 - min_1))",
               "(float) (val_1 - min_1) / (float) (max_1 - min_1 + 1))",
     "pitch interpolation: off-by-one denominator"),
    ("M16", 2, "setChirp(TONE_MAX_PITCH - TONE_MIN_PITCH);",
               "setChirp(0);",
     "limit chirps: flattened"),
    ("M17", 1, "*val = scale * (int32_t) current->gSpeed / velD;",
               "*val = 1000 * (int32_t) current->gSpeed / velD;",
     "glide ratio tone scale: 10000 -> 1000", FP),
    # M18: the SAS interpolation line appears twice in flight_params.c -- the
    # live, golden-covered FS_FlightParams_GetSpeedMul copy (3-tab indent, inside
    # config/else) and the develop-compat FS_FlightParams_GetSASCorrectionFactor
    # copy (2-tab indent), which is intentionally unused on this branch and so is
    # NOT exercised by any golden. Anchor on the 3-tab prefix so the mutation
    # always targets the live copy (unique match, count 1); a mutation in the
    # unused copy would survive and must not be the target. (Card A7 briefly
    # re-anchored this to count 1 while GetSASCorrectionFactor was deleted; the
    # A7 GetSAS-restore follow-up re-adds the second copy, hence the tab anchor.)
    ("M18", 1, "\t\t\tspeed_mul = y1 + ((y2 - y1) * j) / 1024;",
               "\t\t\tspeed_mul = y1 + ((y2 - y1) * j) / 1000;",
     "SAS interpolation: wrong divisor", FP),
    ("M19", 1, "(int32_t) (2 * config->rate);",
               "(int32_t) (3 * config->rate);",
     "change-in-value: wrong time base"),
    # --- timing / consumer ---
    # M20/M21 re-anchored in card A6: the tone volatiles tonePitch/toneChirp/
    # toneRate were folded into a double-buffered ToneSpec_t; the ISR now reads a
    # local snapshot `spec` (spec.pitch/spec.chirp/spec.rate). Mutation semantics
    # unchanged.
    ("M20", 1, "0x10000 - state.tone_timer <= spec.rate",
               "0x10000 - state.tone_timer < spec.rate",
     "tone scheduler: accumulator boundary"),
    ("M21", 1, "spec.pitch + spec.chirp, 125",
               "spec.pitch + spec.chirp, 120",
     "tone beep length 125 -> 120 ms"),
    ("M22", 5, "config->volume * 5",
               "config->volume * 4",
     "tone/alarm volume scale"),
    ("M23", 1, "arb.tone_hold = 1;",
               "arb.tone_hold = 0;",
     "tone hold during speech: disabled"),
    # --- speech ---
    ("M24", 1, "sp_counter >= config->sp_rate &&",
               "sp_counter > config->sp_rate &&",
     "speech period: boundary equality"),
    ("M25", 1, "sp_counter += config->rate;",
               "sp_counter += config->rate * 2;",
     "speech period: counts twice as fast"),
    ("M26", 1, "(number < 20)",
               "(number < 19)",
     "numberToTokens: teens boundary (19)", AS),
    ("M27", 1, "return TOK_LABEL_HORZ;",
               "return TOK_LABEL_VERT;",
     "speech labels: wrong label token", AS),
    ("M28", 1, "(prevHMSL - config->dz_elev) / 1000",
               "(prevHMSL - config->dz_elev) / 100",
     "first-fix altitude announcement: wrong scale", AS),
    ("M29", 1, "config->init_mode == 1",
               "config->init_mode == 3",
     "Init_Mode 1: speech test disabled"),
    ("M30", 1, "tVal = tVal + 5;",
               "tVal = tVal;",
     "distance speech: rounding offset dropped", AS),
    # --- arbiter / policy layer (card A7) ---
    ("M31", 1, "[ZONE_SILENCE_WINDOW]  = SUP_TONE | SUP_ALT_STEP,",
               "[ZONE_SILENCE_WINDOW]  = SUP_TONE,",
     "arbiter: drop silence-window -> ALT_STEP suppression (kill: silence-window-metric)"),
    ("M32", 1, _M32_OLD, _M32_NEW,
     "arbiter: swap ALARM/ALT_STEP priority rows (kill: alarm-altstep-collision)"),
    ("M33", 1, _M33_OLD, _M33_NEW,
     "arbiter: alarm crossing no longer cancels queued speech (kill: alarm-type-none)"),
]


def run(cmd):
    return subprocess.run(cmd, cwd=TEST, capture_output=True, text=True)


def main():
    only = None
    if len(sys.argv) > 2 and sys.argv[1] == "--only":
        only = set(sys.argv[2].split(","))

    # Pristine bytes per patched file (line endings survive mutate/restore).
    # A mutant only ever mutates ONE file at a time; the patched file is
    # restored after each mutant so the next mutant starts from a clean tree
    # even when consecutive mutants target different files.
    originals = {}

    def base_bytes(path):
        if path not in originals:
            originals[path] = path.read_bytes()
        return originals[path]

    results = []

    try:
        for mut in MUTANTS:
            mid, count, old, new, desc = mut[:5]
            path = mut[5] if len(mut) > 5 else SRC
            if only and mid not in only:
                continue

            base = base_bytes(path)
            old_b, new_b = old.encode(), new.encode()
            n = base.count(old_b)
            if n != count:
                results.append((mid, "NO-MATCH", f"{n} occurrences (expected {count})", desc))
                continue

            path.write_bytes(base.replace(old_b, new_b))
            try:
                b = run(["cmake", "--build", "build"])
                if b.returncode != 0:
                    results.append((mid, "BUILD-FAIL", "", desc))
                    continue

                t = run([sys.executable, "run_tests.py"])
                fails = [ln.split()[1] for ln in t.stdout.splitlines() if ln.startswith("FAIL")]
                if t.returncode != 0 and fails:
                    results.append((mid, "KILLED", f"{len(fails)} scenario(s), e.g. {fails[0]}", desc))
                elif t.returncode != 0:
                    results.append((mid, "ERROR", (t.stderr or t.stdout)[-120:].replace("\n", " "), desc))
                else:
                    results.append((mid, "SURVIVED", "", desc))
                print(f"{results[-1][0]}  {results[-1][1]:<10} {desc}", flush=True)
            finally:
                path.write_bytes(base)
    finally:
        for p, b0 in originals.items():
            p.write_bytes(b0)
        run(["cmake", "--build", "build"])

    print()
    print(f"{'ID':<5} {'RESULT':<10} DETAIL")
    survivors = 0
    for (mid, res, detail, desc) in results:
        print(f"{mid:<5} {res:<10} {desc}" + (f"  [{detail}]" if detail else ""))
        if res in ("SURVIVED", "NO-MATCH", "ERROR", "BUILD-FAIL"):
            survivors += 1

    killed = sum(1 for r in results if r[1] == "KILLED")
    print(f"\n{killed} killed, {len(results) - killed} needing attention, of {len(results)} mutants")
    return 1 if survivors else 0


if __name__ == "__main__":
    sys.exit(main())
