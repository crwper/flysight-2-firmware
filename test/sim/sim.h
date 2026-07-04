/*
 * Simulator core: deterministic clock, event ordering, and trace output.
 *
 * Time model
 * ----------
 * All simulator time is in integer nanoseconds from t=0 (the moment
 * FS_AudioControl_Init runs).
 *
 * The target's timer server ticks off the RTC at LSE/CFG_RTCCLK_DIV =
 * 32774/16 Hz, i.e. one tick every 16e9/32774 ns. We round that to an
 * integer:
 *
 *   SIM_RTC_TICK_NS = 488191 ns   (error < 1 ppm)
 *
 * audio_control.c computes its consumer period as 10*1000/CFG_TS_TICK_VAL
 * = 20 ticks, so the consumer timer fires every 20 * 488191 = 9763820 ns
 * (~9.764 ms), just like on hardware (where the nominal "10 ms" period is
 * also actually 20 RTC ticks).
 *
 * Event ordering (deterministic by construction)
 * ----------------------------------------------
 * The run loop repeatedly picks the earliest of:
 *   1. the next timer-server expiry (fake_seq_ts.c), and
 *   2. the next GNSS track point.
 * Ties go to the timer (on target the timer server runs at RTC interrupt
 * priority). After every fired event, all pending sequencer tasks are
 * drained in ascending task-ID order (producer before consumer), looping
 * until no task is pending. This models "the main loop runs the sequencer
 * to completion between interrupts".
 */

#ifndef SIM_H_
#define SIM_H_

#include <stdint.h>
#include <stdio.h>

#include "gnss.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SIM_RTC_TICK_NS 488191ULL

typedef struct
{
	uint64_t t_ns;
	FS_GNSS_Data_t data;
} Sim_TrackPoint_t;

/* Clock */
uint64_t sim_now_ns(void);
void sim_set_now_ns(uint64_t t_ns); /* used by the run loop only */

/* Trace: each line is "<seconds>.<microseconds> <text>\n" */
void sim_trace_open(const char *path);
void sim_trace_close(void);
void sim_trace_raw(const char *fmt, ...);   /* no timestamp prefix */
void sim_trace_event(const char *fmt, ...); /* timestamped */

/* Run the event loop over a track, then keep the timers running for
 * tail_ns after the last point (lets queued speech play out). */
void sim_run(const Sim_TrackPoint_t *points, size_t count, uint64_t tail_ns);

/* Provided by fake_seq_ts.c */
uint64_t sim_timers_next_ns(void);   /* UINT64_MAX if none active */
void     sim_timers_fire_due(void);  /* fire all timers due at now */
void     sim_tasks_drain(void);      /* run pending tasks until none */

/* Provided by fake_gnss.c */
void sim_gnss_set(const FS_GNSS_Data_t *data);

/* Provided by fake_audio.c */
void sim_audio_set_dir(const char *dir);
void sim_audio_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* SIM_H_ */
