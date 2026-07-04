/*
 * Fake audio driver: mirrors the control-flow semantics of
 * FlySight/audio.c without DMA/I2C, and writes every audible event to the
 * trace.
 *
 * Semantics preserved from audio.c:
 *   - FS_Audio_Beep/FS_Audio_Play PREEMPT: if a sound is playing, it is
 *     stopped and the new sound starts immediately (no queueing).
 *   - A beep lasts exactly `duration` ms (duration*24000/1000 samples at
 *     24 kHz).
 *   - A file lasts data_bytes/2 samples at 24 kHz. Like the firmware
 *     (FS_Audio_PlayFile), the data-chunk length is read from the 32-bit
 *     little-endian word at offset 0x28 of the wav file -- the real wav
 *     assets are read from disk, so durations are exact.
 *   - If the file cannot be opened, the driver ends up IDLE (and any
 *     previous sound has already been cut). Logged as PLAYFAIL.
 *   - FS_Audio_IsIdle() becomes true the moment the last sample has
 *     played. (On target the DMA-complete interrupt triggers the update
 *     task immediately, so idle latency is sub-millisecond.)
 *   - FS_Audio_Stop() silences immediately; logged only if it actually
 *     cut a sound short (a Stop while idle is inaudible).
 *
 * Trace lines carry the current GNSS hMSL so events can be mapped to a
 * phase of the jump at a glance.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "app_common.h"
#include "audio.h"
#include "gnss.h"
#include "sim.h"

#define AUDIO_SAMPLE_RATE 24000

#define MAX_WAV_CACHE 128
#define MAX_NAME_LEN  32

typedef struct
{
	char name[MAX_NAME_LEN];
	long samples; /* -1 = file missing/unreadable */
} WavInfo_t;

static WavInfo_t wav_cache[MAX_WAV_CACHE];
static int wav_cache_len;

static const char *audio_dir = "TEMP/AUDIO";

static bool busy;
static uint64_t busy_until_ns;
static char cur_desc[MAX_NAME_LEN + 8];

void sim_audio_set_dir(const char *dir)
{
	audio_dir = dir;
}

void sim_audio_reset(void)
{
	busy = false;
	busy_until_ns = 0;
	cur_desc[0] = '\0';
	wav_cache_len = 0;
}

/* data-chunk length (bytes) via the u32 LE at 0x28, exactly like
 * FS_Audio_PlayFile; -1 if the file cannot be read */
static long wav_samples(const char *filename)
{
	char path[512];
	unsigned char hdr[0x2c];
	FILE *fp;
	uint32_t bytes;
	int i;

	for (i = 0; i < wav_cache_len; ++i)
	{
		if (!strcmp(wav_cache[i].name, filename))
		{
			return wav_cache[i].samples;
		}
	}

	snprintf(path, sizeof(path), "%s/%s", audio_dir, filename);

	long samples = -1;

	fp = fopen(path, "rb");
	if (fp)
	{
		if (fread(hdr, 1, sizeof(hdr), fp) == sizeof(hdr) &&
		    !memcmp(hdr, "RIFF", 4) && !memcmp(hdr + 8, "WAVE", 4))
		{
			bytes = (uint32_t) hdr[0x28] |
			        ((uint32_t) hdr[0x29] << 8) |
			        ((uint32_t) hdr[0x2a] << 16) |
			        ((uint32_t) hdr[0x2b] << 24);
			samples = (long) (bytes / 2);
		}
		fclose(fp);
	}

	if (wav_cache_len < MAX_WAV_CACHE && strlen(filename) < MAX_NAME_LEN)
	{
		strcpy(wav_cache[wav_cache_len].name, filename);
		wav_cache[wav_cache_len].samples = samples;
		++wav_cache_len;
	}

	return samples;
}

static uint64_t samples_to_ns(long samples)
{
	return ((uint64_t) samples * 125000ULL) / 3ULL; /* 1/24000 s = 125000/3 ns */
}

static bool update_idle(void)
{
	if (busy && sim_now_ns() >= busy_until_ns)
	{
		busy = false;
	}

	return !busy;
}

/* "%ld.%03ld" for a value in thousandths (mm -> m), sign-safe */
static const char *fmt_mm(int32_t mm)
{
	static char buf[24];
	const char *sign = (mm < 0) ? "-" : "";
	uint32_t a = (mm < 0) ? (uint32_t) -(int64_t) mm : (uint32_t) mm;

	snprintf(buf, sizeof(buf), "%s%lu.%03lu",
			sign, (unsigned long) (a / 1000), (unsigned long) (a % 1000));
	return buf;
}

static const char *fmt_dur_ns(uint64_t ns)
{
	static char buf[24];

	snprintf(buf, sizeof(buf), "%llu.%03llu",
			(unsigned long long) (ns / 1000000000ULL),
			(unsigned long long) ((ns % 1000000000ULL) / 1000000ULL));
	return buf;
}

/* ", cuts <sound>" when a new command preempts a playing one, else "" */
static const char *preempt_note(void)
{
	static char note[MAX_NAME_LEN + 24];

	if (update_idle())
	{
		note[0] = '\0';
	}
	else
	{
		snprintf(note, sizeof(note), " (cuts %s)", cur_desc);
	}

	return note;
}

void FS_Audio_Beep(uint32_t startFrequency, uint32_t endFrequency, uint32_t duration, uint8_t volume)
{
	const char *note = preempt_note();
	long samples = (long) ((duration * AUDIO_SAMPLE_RATE) / 1000);
	uint64_t dur_ns = samples_to_ns(samples);

	sim_trace_event("BEEP f0=%lu f1=%lu ms=%lu vol=%u hmsl=%s%s",
			(unsigned long) startFrequency, (unsigned long) endFrequency,
			(unsigned long) duration, (unsigned) volume,
			fmt_mm(FS_GNSS_GetData()->hMSL), note);

	busy = true;
	busy_until_ns = sim_now_ns() + dur_ns;
	snprintf(cur_desc, sizeof(cur_desc), "tone");
}

void FS_Audio_Play(const char *filename, uint8_t volume)
{
	const char *note = preempt_note();
	long samples = wav_samples(filename);

	if (samples < 0)
	{
		sim_trace_event("PLAYFAIL file=%s vol=%u hmsl=%s%s",
				filename, (unsigned) volume,
				fmt_mm(FS_GNSS_GetData()->hMSL), note);
		busy = false;
		return;
	}

	sim_trace_event("PLAY file=%s dur=%s vol=%u hmsl=%s%s",
			filename, fmt_dur_ns(samples_to_ns(samples)), (unsigned) volume,
			fmt_mm(FS_GNSS_GetData()->hMSL), note);

	busy = true;
	busy_until_ns = sim_now_ns() + samples_to_ns(samples);
	snprintf(cur_desc, sizeof(cur_desc), "%s", filename);
}

void FS_Audio_Stop(void)
{
	if (!update_idle())
	{
		sim_trace_event("STOP hmsl=%s (cuts %s)",
				fmt_mm(FS_GNSS_GetData()->hMSL), cur_desc);
		busy = false;
	}
}

bool FS_Audio_IsIdle(void)
{
	return update_idle();
}
