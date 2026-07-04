/*
 * Scenario runner for the FlySight audio-control simulator.
 *
 * Usage:
 *   audio_sim --config CONFIG.TXT --track TRACK.CSV
 *             [--audio-dir DIR] [--trace OUT.trace] [--tail SECONDS]
 *
 * Compiles the UNMODIFIED firmware sources (audio_control.c, config.c,
 * nav.c, common.c) against fake platform/driver layers, replays a GNSS
 * track through them on a deterministic simulated clock, and writes every
 * audible event (beeps, wav playback, stops) to a trace.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "app_common.h"
#include "sim.h" /* includes gnss.h, needed by audio_control.h */
#include "audio_control.h"
#include "config.h"
#include "track.h"

static const char *basename_of(const char *path)
{
	const char *p = path, *b = path;

	for (; *p; ++p)
	{
		if (*p == '/' || *p == '\\') b = p + 1;
	}

	return b;
}

static void usage(void)
{
	fprintf(stderr,
		"usage: audio_sim --config CONFIG.TXT --track TRACK.CSV\n"
		"                 [--audio-dir DIR] [--trace OUT] [--tail SECONDS]\n"
		"\n"
		"  --audio-dir  directory with the device's .wav assets (default TEMP/AUDIO)\n"
		"  --trace      trace output file (default stdout)\n"
		"  --tail       seconds to keep simulating after the last track point\n"
		"               (default 2.0; lets queued speech play out)\n");
}

int main(int argc, char **argv)
{
	const char *config_path = NULL;
	const char *track_path = NULL;
	const char *audio_dir = "TEMP/AUDIO";
	const char *trace_path = NULL;
	double tail_s = 2.0;
	int i;

	Sim_TrackPoint_t *points;
	size_t count;

	for (i = 1; i < argc; ++i)
	{
		if (!strcmp(argv[i], "--config") && i + 1 < argc)         config_path = argv[++i];
		else if (!strcmp(argv[i], "--track") && i + 1 < argc)     track_path = argv[++i];
		else if (!strcmp(argv[i], "--audio-dir") && i + 1 < argc) audio_dir = argv[++i];
		else if (!strcmp(argv[i], "--trace") && i + 1 < argc)     trace_path = argv[++i];
		else if (!strcmp(argv[i], "--tail") && i + 1 < argc)      tail_s = atof(argv[++i]);
		else
		{
			usage();
			return 2;
		}
	}

	if (!config_path || !track_path)
	{
		usage();
		return 2;
	}

	if (track_load(track_path, &points, &count) != 0)
	{
		return 2;
	}

	sim_audio_set_dir(audio_dir);
	sim_audio_reset();
	sim_trace_open(trace_path);

	sim_trace_raw("# fs-audio-sim v1");
	sim_trace_raw("# config=%s track=%s tail=%.3f",
			basename_of(config_path), basename_of(track_path), tail_s);
	sim_trace_raw("# rtc_tick_ns=%llu", (unsigned long long) SIM_RTC_TICK_NS);

	FS_Config_Init();
	if (FS_Config_Read(config_path) != FS_CONFIG_OK)
	{
		fprintf(stderr, "error: cannot read config '%s'\n", config_path);
		return 2;
	}

	/* t=0: mode activation (FS_AudioControl_Init on entering active mode) */
	sim_set_now_ns(0);
	FS_AudioControl_Init();
	sim_tasks_drain();

	sim_run(points, count, (uint64_t) (tail_s * 1e9));

	FS_AudioControl_DeInit();
	sim_trace_event("END");
	sim_trace_close();

	free(points);
	return 0;
}
