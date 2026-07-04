/*
 * Fake STM32 sequencer (UTIL_SEQ_*) and timer server (HW_TS_*), backed by
 * the simulator clock.
 *
 * Sequencer: pending tasks are drained in ascending task-ID order, and the
 * drain loop keeps going until nothing is pending (a task may set another
 * task). Priorities are ignored (audio_control only uses CFG_SCH_PRIO_0).
 *
 * Timer server: hw_ts_Repeated timers re-arm at fire_time + period, which
 * matches hw_timerserver.c (it restarts the timer with CounterInit ticks
 * when the callback returns).
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "main.h"
#include "app_common.h"
#include "stm32_seq.h"
#include "sim.h"

#define MAX_TASKS  32
#define MAX_TIMERS 8

static void (*task_fn[MAX_TASKS])(void);
static uint32_t task_pending;

typedef struct
{
	int in_use;
	int running;
	HW_TS_Mode_t mode;
	HW_TS_pTimerCb_t cb;
	uint64_t period_ns;
	uint64_t next_ns;
} FakeTimer_t;

static FakeTimer_t timers[MAX_TIMERS];

void UTIL_SEQ_RegTask(uint32_t TaskId_bm, uint32_t Flags, void (*Task)(void))
{
	int i;

	(void) Flags;

	for (i = 0; i < MAX_TASKS; ++i)
	{
		if (TaskId_bm & (1U << i))
		{
			task_fn[i] = Task;
		}
	}
}

void UTIL_SEQ_SetTask(uint32_t TaskId_bm, uint32_t Task_Prio)
{
	(void) Task_Prio;
	task_pending |= TaskId_bm;
}

void sim_tasks_drain(void)
{
	while (task_pending)
	{
		int i;

		for (i = 0; i < MAX_TASKS; ++i)
		{
			if (task_pending & (1U << i))
			{
				task_pending &= ~(1U << i);
				if (task_fn[i]) task_fn[i]();
				break; /* restart scan: lowest ID always first */
			}
		}
	}
}

HW_TS_ReturnStatus_t HW_TS_Create(uint32_t TimerProcessID, uint8_t *pTimerId, HW_TS_Mode_t TimerMode, HW_TS_pTimerCb_t pTimerCallBack)
{
	int i;

	(void) TimerProcessID;

	for (i = 0; i < MAX_TIMERS; ++i)
	{
		if (!timers[i].in_use)
		{
			timers[i].in_use = 1;
			timers[i].running = 0;
			timers[i].mode = TimerMode;
			timers[i].cb = pTimerCallBack;
			*pTimerId = (uint8_t) i;
			return hw_ts_Successful;
		}
	}

	fprintf(stderr, "fake_seq_ts: out of timers\n");
	exit(2);
}

void HW_TS_Start(uint8_t TimerID, uint32_t timeout_ticks)
{
	FakeTimer_t *t = &timers[TimerID];

	t->period_ns = (uint64_t) timeout_ticks * SIM_RTC_TICK_NS;
	t->next_ns = sim_now_ns() + t->period_ns;
	t->running = 1;
}

void HW_TS_Stop(uint8_t TimerID)
{
	timers[TimerID].running = 0;
}

void HW_TS_Delete(uint8_t TimerID)
{
	timers[TimerID].running = 0;
	timers[TimerID].in_use = 0;
}

uint64_t sim_timers_next_ns(void)
{
	uint64_t next = UINT64_MAX;
	int i;

	for (i = 0; i < MAX_TIMERS; ++i)
	{
		if (timers[i].in_use && timers[i].running && timers[i].next_ns < next)
		{
			next = timers[i].next_ns;
		}
	}

	return next;
}

void sim_timers_fire_due(void)
{
	int i;

	for (i = 0; i < MAX_TIMERS; ++i)
	{
		FakeTimer_t *t = &timers[i];

		if (t->in_use && t->running && t->next_ns <= sim_now_ns())
		{
			if (t->mode == hw_ts_Repeated)
			{
				t->next_ns += t->period_ns;
			}
			else
			{
				t->running = 0;
			}

			t->cb();
		}
	}
}
