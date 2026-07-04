#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "main.h"
#include "app_common.h"
#include "sim.h" /* includes gnss.h, needed by audio_control.h */
#include "audio_control.h"

static uint64_t now_ns;
static FILE *trace_fp;

uint64_t sim_now_ns(void)
{
	return now_ns;
}

void sim_set_now_ns(uint64_t t_ns)
{
	now_ns = t_ns;
}

void sim_trace_open(const char *path)
{
	if (!path)
	{
		trace_fp = stdout;
		return;
	}

	trace_fp = fopen(path, "wb"); /* binary: LF line endings on all hosts */
	if (!trace_fp)
	{
		fprintf(stderr, "error: cannot open trace file '%s'\n", path);
		exit(2);
	}
}

void sim_trace_close(void)
{
	if (trace_fp && trace_fp != stdout)
	{
		fclose(trace_fp);
	}
	trace_fp = NULL;
}

void sim_trace_raw(const char *fmt, ...)
{
	va_list ap;

	if (!trace_fp) return;

	va_start(ap, fmt);
	vfprintf(trace_fp, fmt, ap);
	va_end(ap);
	fputc('\n', trace_fp);
}

void sim_trace_event(const char *fmt, ...)
{
	va_list ap;

	if (!trace_fp) return;

	fprintf(trace_fp, "%4llu.%06llu  ",
			(unsigned long long) (now_ns / 1000000000ULL),
			(unsigned long long) ((now_ns % 1000000000ULL) / 1000ULL));

	va_start(ap, fmt);
	vfprintf(trace_fp, fmt, ap);
	va_end(ap);
	fputc('\n', trace_fp);
}

void sim_run(const Sim_TrackPoint_t *points, size_t count, uint64_t tail_ns)
{
	size_t i = 0;
	uint64_t end_ns = (count > 0) ? points[count - 1].t_ns + tail_ns : tail_ns;

	for (;;)
	{
		uint64_t t_timer = sim_timers_next_ns();
		uint64_t t_gnss = (i < count) ? points[i].t_ns : UINT64_MAX;

		if (t_timer <= t_gnss)
		{
			if (t_timer > end_ns) break;
			sim_set_now_ns(t_timer);
			sim_timers_fire_due();
		}
		else
		{
			if (t_gnss > end_ns) break;
			sim_set_now_ns(t_gnss);
			sim_gnss_set(&points[i].data);
			FS_AudioControl_UpdateGNSS(&points[i].data);
			++i;
		}

		sim_tasks_drain();
	}

	sim_set_now_ns(end_ns);
}
