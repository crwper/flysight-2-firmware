/***************************************************************************
**                                                                        **
**  FlySight 2 firmware                                                   **
**  Copyright 2023 Bionic Avionics Inc.                                   **
**                                                                        **
**  This program is free software: you can redistribute it and/or modify  **
**  it under the terms of the GNU General Public License as published by  **
**  the Free Software Foundation, either version 3 of the License, or     **
**  (at your option) any later version.                                   **
**                                                                        **
**  This program is distributed in the hope that it will be useful,       **
**  but WITHOUT ANY WARRANTY; without even the implied warranty of        **
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         **
**  GNU General Public License for more details.                          **
**                                                                        **
**  You should have received a copy of the GNU General Public License     **
**  along with this program.  If not, see <http://www.gnu.org/licenses/>. **
**                                                                        **
****************************************************************************
**  Contact: Bionic Avionics Inc.                                         **
**  Website: http://flysight.ca/                                          **
****************************************************************************/

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "main.h"
#include "app_common.h"
#include "audio.h"
#include "audio_control.h"
#include "audio_speech.h"
#include "common.h"
#include "config.h"
#include "flight_params.h"
#include "nav.h"
#include "stm32_seq.h"

#define CONSUMER_TIMER_MSEC    10
#define CONSUMER_TIMER_TICKS   (CONSUMER_TIMER_MSEC*1000/CFG_TS_TICK_VAL)

#define ABS(a) (((a) < 0) ? -(a) : (a))

#define INVALID_VALUE   INT32_MAX

#define ALT_MIN         1500L // Minimum announced altitude (m)

#define TONE_MIN_PITCH 220
#define TONE_MAX_PITCH 1760

// ===========================================================================
//  Arbiter policy (data)
// ===========================================================================
//
// Audio channels, highest priority first. The arbiter grants at most one
// audible event at a time; a higher channel preempts / blocks lower ones.
// The precedence below is realized structurally by the source-request order
// (producer sample) and the grant order (consumer tick), NOT by iterating this
// enum -- it documents the emergent order the old `flags`/`toneHold`/
// `g_suppress_tone` code produced, which the goldens pin byte-for-byte.
//
// Known precedence special case (old code wins, see JOURNAL A5): the STARTUP
// first-fix beep is deferred until the speech queue is empty and the driver is
// idle (QUIRKS #3), so although STARTUP outranks SPEECH in the table, the beep
// effectively yields to any queued speech. The STARTUP rank only governs its
// precedence over ALT_GROUND_ELEV inside the empty-queue consumer branch.
typedef enum
{
	CH_ALARM,           // alarm crossings (beep/chirp/file) -- never suppressed
	CH_STARTUP,         // init speech + first-fix beep
	CH_ALT_GROUND_ELEV, // ground-elevation confirmation
	CH_ALT_STEP,        // altitude-step announcements
	CH_SPEECH,          // periodic rotational speech
	CH_TONE,            // continuous tone
	CH_COUNT
} AudioChannel_t;

#define SUP_TONE       (1u << CH_TONE)
#define SUP_ALT_STEP   (1u << CH_ALT_STEP)

// Suppression zones -> the channels they silence (policy as data). Entering a
// zone stops current playback + clears the speech queue
// (Arbiter_ApplySuppression); while inside, the masked channels cannot start.
// ZONE_ALT_STEP_WINDOW is a fourth zone beyond the brief's three; it is
// preserved verbatim from the old AltModeSource code (see JOURNAL A5).
// ZONE_SPEECH_ACTIVE (the old toneHold) is applied dynamically at the consumer
// tick via Arbiter_t.tone_hold rather than through this producer-side mask.
typedef enum
{
	ZONE_ALARM_WINDOW,    // near a configured alarm elevation
	ZONE_SILENCE_WINDOW,  // inside a configured silence window
	ZONE_ALT_STEP_WINDOW, // near an altitude-step announcement elevation
	ZONE_SPEECH_ACTIVE,   // a speech token is currently playing (old toneHold)
	ZONE_COUNT
} SuppressionZone_t;

static const uint8_t suppression_table[ZONE_COUNT] =
{
	[ZONE_ALARM_WINDOW]    = SUP_TONE,
	[ZONE_SILENCE_WINDOW]  = SUP_TONE | SUP_ALT_STEP,
	[ZONE_ALT_STEP_WINDOW] = SUP_TONE,
	[ZONE_SPEECH_ACTIVE]   = SUP_TONE,
};

// ===========================================================================
//  Sources + arbiter state
// ===========================================================================

// Altitude-mode source: silence + alt-step suppression zones, the Alt_Step
// announcement decision, and the say-altitude latch (old FLAG_SAY_ALTITUDE):
// the ground-elevation announcement is pending until vAcc is good and it plays,
// which blocks Alt_Step announcements AND all periodic speech (current
// behaviour; the QUIRKS #17 refinement is B4, not here).
typedef struct
{
	bool say_altitude;
} AltModeSource_t;

// Speech source: rotation index + speech-period accumulator (old cur_speech /
// sp_counter). Owns which periodic utterance speaks next and when.
typedef struct
{
	uint8_t  cur_speech;
	uint16_t sp_counter;
} SpeechSource_t;

// Startup source: first-fix beep bookkeeping (old FLAG_FIRST_FIX /
// FLAG_BEEP_DONE). Init speech/file queuing happens at FS_AudioControl_Init.
typedef struct
{
	bool first_fix_pending;
	bool beep_done;
} StartupSource_t;

// Arbiter state: the two edge/hold latches that used to be `g_suppress_tone`
// and `toneHold`.
typedef struct
{
	volatile uint8_t tone_hold;   // ISR-read: speech playing suppresses TONE
	uint8_t          tone_suppressed; // suppression-zone rising-edge latch
} Arbiter_t;

// Tone spec handed off producer-task -> consumerTimer ISR. The three fields
// (formerly the independent volatiles tonePitch/toneChirp/toneRate) form ONE
// coherent unit: a beep must use a pitch/chirp/rate that were computed
// together. Double-buffering (see tone_slots/tone_active below) publishes the
// whole struct atomically so the ISR can never observe a torn mix of an old
// and a new update. Field order/types match the old volatiles exactly, so the
// accumulator math (0x10000 - tone_timer <= rate) is bit-identical.
typedef struct
{
	uint32_t pitch;
	int32_t  chirp;
	uint16_t rate;
} ToneSpec_t;

typedef struct
{
	uint8_t timer_id;

	// Producer-level fix / accuracy tracking (old FLAG_HAS_FIX,
	// FLAG_VERTICAL_ACC, prev_flags). Feeds the sources and the arbiter.
	bool has_fix;
	bool prev_has_fix;
	bool vacc_good;

	int32_t prevHMSL;

	AltModeSource_t altmode;
	SpeechSource_t  speech_src;
	StartupSource_t startup;
	Arbiter_t       arb;

	FS_Speech_t speech;

	// Double-buffered tone spec (producer task -> consumerTimer ISR). The
	// producer builds the next spec in the inactive slot, then publishes it with
	// a single aligned 32-bit store to tone_active; the ISR reads slots[active].
	// See ToneSpec_t and the memory-ordering note at Arbiter_GrantTone.
	ToneSpec_t        tone_slots[2];
	volatile uint32_t tone_active;

	// CHANGE_IN_VALUE_1 history (formerly function-local statics in updateTones).
	// Integer value_1 in the rate path's scale (QUIRKS #5: the rate must stay
	// bit-identical, so this stays integer even though pitch value_1 is float).
	int32_t x0, x1, x2;

	// Tone-rate accumulator (formerly function-local static in consumerTimer)
	uint16_t tone_timer;
} FS_AudioControl_State_t;

static FS_AudioControl_State_t state;

// ===========================================================================
//  Tone source helpers
// ===========================================================================

// Producer-side draft slot: the inactive buffer the producer mutates before
// publishing it (the flip happens once, at the end of producerTask). Only the
// producer task ever calls the set* helpers, and the ISR never writes
// tone_active, so reading the volatile index here is race-free.
static ToneSpec_t *toneDraft(void)
{
	return &state.tone_slots[!state.tone_active];
}

static void setRate(uint16_t rate)
{
	toneDraft()->rate = rate;
}

static void setPitch(uint16_t pitch)
{
	toneDraft()->pitch = pitch;
}

static void setChirp(uint32_t chirp)
{
	toneDraft()->chirp = chirp;
}

// Tone emission. value_1 / value_2 keep the ORIGINAL integer metric so gating,
// rate and clamps are byte-identical to the pre-B1 code (a continuous-float
// metric would coarsen->smooth some pitches by far more than +/-1 Hz, e.g. the
// dive angle old-truncated to whole degrees, and would drift the rate). The
// ONLY B1 change on the tone path is the in-band pitch: the same integer ratio
// is evaluated in float and ROUNDED to Hz at the last step, so a pitch differs
// from the old truncation by at most +/-1 Hz and nothing else moves.
static void setTone(
	const FS_Config_Data_t *config,
	int32_t val_1,
	int32_t min_1,
	int32_t max_1,
	int32_t val_2,
	int32_t min_2,
	int32_t max_2)
{
	#define UNDER(val,min,max) ((min < max) ? (val <= min) : (val >= min))
	#define OVER(val,min,max)  ((min < max) ? (val >= max) : (val <= max))

	if (val_1 != INVALID_VALUE &&
	    val_2 != INVALID_VALUE)
	{
		if (UNDER(val_2, min_2, max_2))
		{
			if (config->flatline)
			{
				setRate(FS_CONFIG_RATE_FLATLINE);
			}
			else
			{
				setRate(config->min_rate);
			}
		}
		else if (OVER(val_2, min_2, max_2))
		{
			setRate(config->max_rate - 1);
		}
		else
		{
			// Rate accumulator stays integer AND bit-identical (QUIRKS #5): the
			// value_2 rate path keeps the original integer arithmetic, so the
			// free-running 0x10000 phase never drifts.
			setRate(config->min_rate + (config->max_rate - config->min_rate) * (val_2 - min_2) / (max_2 - min_2));
		}

		if (UNDER(val_1, min_1, max_1))
		{
			if (config->limits == 0)
			{
				setRate(0);
			}
			else if (config->limits == 1)
			{
				setPitch(TONE_MIN_PITCH);
				setChirp(0);
			}
			else if (config->limits == 2)
			{
				setPitch(TONE_MIN_PITCH);
				setChirp(TONE_MAX_PITCH - TONE_MIN_PITCH);
			}
			else
			{
				setPitch(TONE_MAX_PITCH);
				setChirp(TONE_MIN_PITCH - TONE_MAX_PITCH);
			}
		}
		else if (OVER(val_1, min_1, max_1))
		{
			if (config->limits == 0)
			{
				setRate(0);
			}
			else if (config->limits == 1)
			{
				setPitch(TONE_MAX_PITCH);
				setChirp(0);
			}
			else if (config->limits == 2)
			{
				setPitch(TONE_MAX_PITCH);
				setChirp(TONE_MIN_PITCH - TONE_MAX_PITCH);
			}
			else
			{
				setPitch(TONE_MIN_PITCH);
				setChirp(TONE_MAX_PITCH - TONE_MIN_PITCH);
			}
		}
		else
		{
			// In-band pitch: the ONLY float-derived driver output on the tone
			// path. Same integer ratio as before, evaluated in float and rounded
			// to Hz at the last step (was integer truncation).
			setPitch((uint16_t) lroundf((float) TONE_MIN_PITCH + (float) (TONE_MAX_PITCH - TONE_MIN_PITCH) * (float) (val_1 - min_1) / (float) (max_1 - min_1)));
			setChirp(0);
		}
	}
	else
	{
		setRate(0);
	}

	#undef OVER
	#undef UNDER
}

// ===========================================================================
//  Speech source
// ===========================================================================

// Periodic (rotational) speech: when the tone gate is open and the period has
// elapsed, queue the next non-skipped utterance and rotate. Skips Sp_Mode 12
// (altitude) entries below ALT_MIN, and (old behaviour) yields to a pending
// ground-elevation announcement via the say-altitude latch.
static void SpeechSource_MaybeSpeak(
	const FS_Config_Data_t *config,
	const FS_FlightData_t *fd,
	const FS_GNSS_Data_t *current)
{
	uint8_t i;

	if (config->sp_rate != 0 &&
	    config->num_speech != 0 &&
	    state.speech_src.sp_counter >= config->sp_rate &&
	    !FS_Speech_HasPending(&state.speech) &&
	    !state.altmode.say_altitude)
	{
		for (i = 0; i < config->num_speech; ++i)
		{
			if ((config->speech[state.speech_src.cur_speech].mode != FS_CONFIG_MODE_ALTITUDE) ||
				(current->hMSL - config->dz_elev >= ALT_MIN * 1000))
			{
				FS_Speech_BuildValue(&state.speech, config, fd, current, state.speech_src.cur_speech);
				state.speech_src.cur_speech = (state.speech_src.cur_speech + 1) % config->num_speech;
				break;
			}
			else
			{
				state.speech_src.cur_speech = (state.speech_src.cur_speech + 1) % config->num_speech;
			}
		}

		state.speech_src.sp_counter = 0;
	}
}

// Accumulate the speech-period counter (runs every producer sample regardless
// of suppression / thresholds, exactly as the old trailing block did).
static void SpeechSource_Tick(const FS_Config_Data_t *config)
{
	if (state.speech_src.sp_counter < config->sp_rate)
	{
		state.speech_src.sp_counter += config->rate;
	}
}

// ===========================================================================
//  Tone source
// ===========================================================================

static void updateTones(
	const FS_Config_Data_t *config,
	const FS_FlightData_t *fd,
	const FS_GNSS_Data_t *current)
{
	// Integer V/H-threshold gate kept byte-exact (cm/s), independent of the
	// float value path.
	const int32_t velD = current->velD / 10;

	// The tone value_1 (pitch + gating) and value_2 (rate) stay in the ORIGINAL
	// integer arithmetic (QUIRKS #5: the rate accumulator must be bit-identical
	// or the free-running 0x10000 phase drifts; the pitch is re-rounded from
	// this same integer value inside setTone). value_1 also feeds the
	// magnitude/change-in-value rate modes below.
	int32_t val_1 = INVALID_VALUE, min_1 = config->min,   max_1 = config->max;
	int32_t val_2 = INVALID_VALUE, min_2 = config->min_2, max_2 = config->max_2;

	FS_FlightParams_GetRateValue(config->mode, current, config, 10000, &val_1, &min_1, &max_1);

	if (config->mode_2 == FS_CONFIG_MODE_DIRECTION_TO_DESTINATION) // Direction to destination
	{
		if (config->mode == FS_CONFIG_MODE_DIRECTION_TO_DESTINATION)  //no need to re-calculate direction
		{
			val_2 = ABS(val_1);
		}
		else
		{
			val_2 = ABS(calcDirection(current->lat,current->lon,config->lat,config->lon,current->heading));
			val_2 = 180-val_2;  //make inverse so faster rate indicates closer to bearing
		}
		val_2 = pow(val_2, 3);
		min_2 = 0;
		max_2 = pow(180, 3);
	}
	else if (config->mode_2 == FS_CONFIG_MODE_DIRECTION_TO_BEARING) // Direction to bearing
	{
		if (config->mode == FS_CONFIG_MODE_DIRECTION_TO_BEARING)  //no need to re-calculate direction
		{
			val_2 = ABS(val_1);
		}
		else
		{
			// QUIRKS #16 preserved (B3 fixes it): raw deg*1e5 heading passed to
			// calcRelBearing, which expects plain degrees.
			val_2 = ABS(calcRelBearing(config->bearing,current->heading));
			val_2 = 180-val_2;  //make inverse so faster rate indicates closer to bearing
		}
		min_2 = 0;
		max_2 = 180;
	}
	else if (config->mode_2 == FS_CONFIG_MODE_MAGNITUDE_OF_VALUE_1)
	{
		FS_FlightParams_GetRateValue(config->mode, current, config, 10000, &val_2, &min_2, &max_2);
		if (val_2 != INVALID_VALUE)
		{
			val_2 = ABS(val_2);
		}
	}
	else if (config->mode_2 == FS_CONFIG_MODE_CHANGE_IN_VALUE_1)
	{
		state.x2 = state.x1;
		state.x1 = state.x0;
		state.x0 = val_1;

		if (state.x0 != INVALID_VALUE &&
			state.x1 != INVALID_VALUE &&
			state.x2 != INVALID_VALUE &&
			max_1 != min_1)
		{
			val_2 = (int32_t) 1000 * (state.x2 - state.x0) / (int32_t) (2 * config->rate);
			val_2 = (int32_t) 10000 * ABS(val_2) / ABS(max_1 - min_1);
		}
	}
	else
	{
		FS_FlightParams_GetRateValue(config->mode_2, current, config, 10000, &val_2, &min_2, &max_2);
	}

	if (!state.arb.tone_suppressed)
	{
		if (ABS(velD) >= config->threshold &&
			current->gSpeed >= config->hThreshold)
		{
			setTone(config, val_1, min_1, max_1, val_2, min_2, max_2);

			SpeechSource_MaybeSpeak(config, fd, current);
		}
		else
		{
			setRate(0);
		}
	}

	SpeechSource_Tick(config);
}

// ===========================================================================
//  Alarm + altitude-mode sources
// ===========================================================================

// Alarm source: alarm-window suppression (every sample) and the alarm-crossing
// scan ([min,max) interval, QUIRKS #8). ORs its suppression contribution into
// suppress_mask and, when the previous sample had a fix, reports the index of
// the alarm to fire (num_alarms = none); the actual sound / speech-cancel is
// applied by the arbiter AFTER the shared rising-edge stop, preserving
// Stop-before-Beep ordering.
static void AlarmSource_Update(
	const FS_Config_Data_t *config,
	FS_GNSS_Data_t *current,
	bool prev_had_fix,
	int32_t prevHMSL,
	uint8_t *suppress_mask,
	uint8_t *fired_index)
{
	uint8_t i;

	for (i = 0; i < config->num_alarms; ++i)
	{
		const int32_t alarm_elev = config->alarms[i].elev + config->dz_elev;

		if ((current->hMSL <= alarm_elev + config->alarm_window_above) &&
		    (current->hMSL >= alarm_elev - config->alarm_window_below))
		{
			*suppress_mask |= suppression_table[ZONE_ALARM_WINDOW];
			break;
		}
	}

	*fired_index = config->num_alarms;

	if (prev_had_fix)
	{
		int32_t min = MIN(prevHMSL, current->hMSL);
		int32_t max = MAX(prevHMSL, current->hMSL);

		for (i = 0; i < config->num_alarms; ++i)
		{
			const int32_t alarm_elev = config->alarms[i].elev + config->dz_elev;

			if (alarm_elev >= min && alarm_elev < max)
			{
				*fired_index = i;
				break;
			}
		}
	}
}

// Altitude-mode source: silence-window suppression (TONE + ALT_STEP), Alt_Step
// window suppression (TONE, with the ALT_MIN floor), and the Alt_Step
// announcement decision. want_alt_step folds in every self-contained guard,
// including the silence-window ALT_STEP suppression read back from
// suppress_mask (only ZONE_SILENCE_WINDOW sets SUP_ALT_STEP); the arbiter ANDs
// it with "no alarm fired" and "speech queue empty" before building.
static void AltModeSource_Update(
	const FS_Config_Data_t *config,
	FS_GNSS_Data_t *current,
	bool prev_had_fix,
	int32_t prevHMSL,
	uint8_t *suppress_mask,
	bool *want_alt_step,
	int32_t *step_out)
{
	const int32_t velD = current->velD / 10;

	uint8_t i;
	int32_t step_size, step = 0, step_elev = 0;

	for (i = 0; i < config->num_windows; ++i)
	{
		if ((config->windows[i].bottom + config->dz_elev <= current->hMSL) &&
		    (config->windows[i].top + config->dz_elev >= current->hMSL))
		{
			*suppress_mask |= suppression_table[ZONE_SILENCE_WINDOW];
			break;
		}
	}

	if (config->alt_step > 0)
	{
		if (config->alt_units == FS_CONFIG_UNITS_METERS)
		{
			step_size = 10000 * config->alt_step;
		}
		else
		{
			step_size = 3048 * config->alt_step;
		}

		step = ((current->hMSL - config->dz_elev) * 10 + step_size / 2) / step_size;
		step_elev = step * step_size / 10 + config->dz_elev;

		if ((current->hMSL <= step_elev + config->alarm_window_above) &&
		    (current->hMSL >= step_elev - config->alarm_window_below) &&
		    (current->hMSL - config->dz_elev >= ALT_MIN * 1000))
		{
			*suppress_mask |= suppression_table[ZONE_ALT_STEP_WINDOW];
		}
	}

	*want_alt_step = false;
	*step_out = step;

	if (prev_had_fix)
	{
		int32_t min = MIN(prevHMSL, current->hMSL);
		int32_t max = MAX(prevHMSL, current->hMSL);

		if ((config->alt_step > 0) &&
		    (prevHMSL - config->dz_elev >= ALT_MIN * 1000) &&
		    !state.altmode.say_altitude &&
		    !(*suppress_mask & SUP_ALT_STEP) &&
		    (step_elev >= min && step_elev < max) &&
		    ABS(velD) >= config->threshold &&
		    current->gSpeed >= config->hThreshold)
		{
			*want_alt_step = true;
		}
	}
}

// ===========================================================================
//  AUDIO ARBITER
//
//  The single owner of "what is audible": the ONLY caller of FS_Audio_* in the
//  audio-control module. Sources compute suppression zones and one-shot
//  requests; the arbiter turns them into driver calls in priority order.
// ===========================================================================

// Suppression-zone rising edge (old g_suppress_tone edge): entering any TONE
// suppression zone stops the playing sound and clears queued speech.
static void Arbiter_ApplySuppression(bool suppress)
{
	if (suppress && !state.arb.tone_suppressed)
	{
		FS_Speech_Clear(&state.speech);
		setRate(0);
		FS_Audio_Stop();
	}

	state.arb.tone_suppressed = suppress;
}

// CH_ALARM: emit the sound for a fired alarm crossing. Preempts unconditionally
// (no idle guard) so an alarm is always heard; never gated on vAcc.
static void Arbiter_FireAlarm(
	const FS_Config_Data_t *config,
	uint8_t index)
{
	char filename[13];

	switch (config->alarms[index].type)
	{
	case 1:	// beep
		FS_Audio_Beep(TONE_MAX_PITCH, TONE_MAX_PITCH, 125, config->volume * 5);
		break ;
	case 2:	// chirp up
		FS_Audio_Beep(TONE_MIN_PITCH, TONE_MAX_PITCH, 125, config->volume * 5);
		break ;
	case 3:	// chirp down
		FS_Audio_Beep(TONE_MAX_PITCH, TONE_MIN_PITCH, 125, config->volume * 5);
		break ;
	case 4:	// play file
		filename[0] = '\0';
		strncat(filename, config->alarms[index].filename, sizeof(filename) - 1);
		strncat(filename, ".wav", sizeof(filename) - 1);
		FS_Audio_Play(filename, config->sp_volume * 5);
		break;
	}
}

// CH_TONE: emit a tone beep on the consumer tick. Suppressed while a speech
// token is playing (ZONE_SPEECH_ACTIVE == tone_hold) or the driver is busy.
static void Arbiter_GrantTone(const FS_Config_Data_t *config)
{
	// Snapshot the published index ONCE, then read that whole slot into a local
	// so pitch/chirp/rate are mutually coherent for this tick.
	//
	// Memory ordering (single-writer producer task / single-reader consumer
	// ISR, one Cortex-M4 core):
	//   * The producer only ever mutates the INACTIVE slot and publishes it via
	//     a single naturally-aligned 32-bit store to tone_active, which is
	//     atomic on Cortex-M4 -- the ISR reads either the old index or the new
	//     one, never a partial value.
	//   * This ISR cannot be preempted by the producer task, so it observes the
	//     chosen slot atomically; and because both run on the same core, taking
	//     the exception orders the producer's slot writes before its tone_active
	//     store as seen here -- if we read the new index, the matching slot
	//     writes are already visible. On the producer side a compiler barrier
	//     ("" ::: "memory") sits immediately before the flip so the non-volatile
	//     draft-slot writes cannot be sunk past the volatile publish at compile
	//     time; this hardware note covers the run-time side of the same ordering.
	//   * No DMB is needed: a DMB would only matter for a second core or a
	//     DMA/peripheral observer. Same-core code + ISR need no barrier for this
	//     publish-then-flip pattern.
	const uint32_t idx = state.tone_active;
	const ToneSpec_t spec = state.tone_slots[idx];

	if (FS_Audio_IsIdle() && !state.arb.tone_hold && spec.rate > 0 && 0x10000 - state.tone_timer <= spec.rate)
	{
		FS_Audio_Beep(spec.pitch, spec.pitch + spec.chirp, 125, config->volume * 5);
	}

	state.tone_timer += spec.rate;
}

// Consumer-tick grant. Speech (any queued utterance -- init, ground-elev,
// alt-step, periodic) drains first; the empty-queue branch then grants STARTUP
// (first-fix beep) ahead of ALT_GROUND_ELEV (which re-queues speech). tone_hold
// tracks whether a speech token is currently playing.
static void Arbiter_ConsumerTick(const FS_Config_Data_t *config)
{
	if (FS_Speech_HasPending(&state.speech))
	{
		if (FS_Audio_IsIdle())
		{
			state.arb.tone_hold = 1;

			// Guarded by FS_Speech_HasPending above, so a queued token always
			// maps to a wav filename (or the raw-file payload) -- never NULL.
			FS_Audio_Play(FS_Speech_PlayNext(&state.speech), config->sp_volume * 5);
		}
	}
	else
	{
		state.arb.tone_hold = 0;

		// CH_STARTUP: first-fix beep. Deferred until the speech queue is empty
		// and the driver idle (QUIRKS #3) -- so it yields to any queued speech
		// even though STARTUP outranks SPEECH in the priority table.
		if (state.startup.first_fix_pending && FS_Audio_IsIdle())
		{
			state.startup.first_fix_pending = false;
			FS_Audio_Beep(TONE_MAX_PITCH, TONE_MAX_PITCH, 125, config->volume * 5);
			state.startup.beep_done = true;
		}

		// CH_ALT_GROUND_ELEV: ground-elevation confirmation, once vAcc is good.
		if (state.altmode.say_altitude &&
			state.has_fix &&
			state.vacc_good &&
			FS_Audio_IsIdle())
		{
			state.altmode.say_altitude = false;

			FS_Speech_BuildGroundElev(&state.speech, config, state.prevHMSL);
		}
	}
}

// ===========================================================================
//  Task glue
// ===========================================================================

static void producerTask(void)
{
	// Producer-private config snapshot. QUIRKS #7: config is READ-ONLY in the
	// audio module -- nothing writes to it (the old direction-mode `decimals = 0`
	// override now lives in a local inside FS_Speech_BuildValue). The copy is
	// kept only for producer/ISR isolation and is `const` so the read-only
	// contract is enforced at compile time end-to-end.
	const FS_Config_Data_t config = *FS_Config_Get();
	FS_GNSS_Data_t current;

	// Seed the draft slot from the currently-published one. The old code kept
	// tonePitch/toneChirp/toneRate as persistent volatiles, so a pass that
	// touches only a subset (e.g. setRate(0)) carried the rest forward. All
	// set* helpers below mutate this draft; nothing is published until the flip
	// at the end of this task, so the ISR keeps seeing the old coherent spec
	// throughout the pass.
	*toneDraft() = state.tone_slots[state.tone_active];

	// Copy current GNSS sample to a local snapshot.
	memcpy(&current, FS_GNSS_GetData(), sizeof(FS_GNSS_Data_t));

	// The one and only GNSS -> SI conversion. valid3d / vAccGood fold the old
	// FLAG_HAS_FIX / FLAG_VERTICAL_ACC tests; the float metric path works from
	// `fd`, while the alarm/alt-step crossings keep the integer `current` so
	// their threshold timing stays byte-exact.
	const FS_FlightData_t fd = FS_FlightData_FromGNSS(&current);

	if (fd.valid3d)
	{
		state.has_fix = true;

		{
			const bool prev_had_fix = state.prev_has_fix;

			uint8_t suppress_mask = 0;
			uint8_t fired_index;
			bool want_alt_step;
			int32_t step;

			AlarmSource_Update(&config, &current, prev_had_fix, state.prevHMSL,
			                   &suppress_mask, &fired_index);
			AltModeSource_Update(&config, &current, prev_had_fix, state.prevHMSL,
			                     &suppress_mask, &want_alt_step, &step);

			// Rising-edge stop: entering a TONE suppression zone stops the sound.
			Arbiter_ApplySuppression((suppress_mask & SUP_TONE) != 0);

			if (prev_had_fix)
			{
				if (fired_index != config.num_alarms)
				{
					Arbiter_FireAlarm(&config, fired_index);
					FS_Speech_Clear(&state.speech);
				}
				else if (want_alt_step && !FS_Speech_HasPending(&state.speech))
				{
					FS_Speech_BuildAltStep(&state.speech, &config, step);
				}
			}
		}

		updateTones(&config, &fd, &current);

		if (!state.startup.beep_done)
		{
			state.startup.first_fix_pending = true;
		}
	}
	else
	{
		state.has_fix = false;
		setRate(0);
	}

	state.vacc_good = fd.vAccGood;

	state.prev_has_fix = state.has_fix;
	state.prevHMSL = current.hMSL;

	// Publish the freshly-built tone spec: a single aligned 32-bit store to the
	// volatile index, atomic on Cortex-M4. Only this task writes tone_active, so
	// the read-modify-write cannot be lost. After this line the ISR reads the
	// new slot; the just-vacated slot becomes the next pass's draft.
	//
	// Compiler barrier: the draft-slot writes above (the seed copy plus every
	// set*/setRate/setPitch/setChirp) are NON-volatile, and the C standard only
	// orders volatile accesses relative to one another -- so without this the
	// optimizer could legally sink those slot stores PAST the volatile flip,
	// re-introducing a same-core torn read (an ISR seeing the new index but a
	// half-written slot). This "" ::: "memory" clobber is a pure COMPILE-TIME
	// barrier (NOT a DMB -- no hardware barrier is wanted): it forbids the
	// compiler from moving memory accesses across it. Paired with the single-
	// core hardware ordering argued at Arbiter_GrantTone (same-core exception
	// entry orders the store-buffer drain), it makes the publish-then-flip
	// airtight against both compile-time and run-time reordering.
	__asm__ volatile ("" ::: "memory");   // compiler barrier
	state.tone_active = !state.tone_active;
}

static void consumerTimer(void)
{
	const FS_Config_Data_t *config = FS_Config_Get();

	Arbiter_GrantTone(config);

	// Call consumer task
	UTIL_SEQ_SetTask(1<<CFG_TASK_FS_AUDIO_CONTROL_CONSUMER_ID, CFG_SCH_PRIO_0);
}

static void consumerTask(void)
{
	const FS_Config_Data_t *config = FS_Config_Get();

	Arbiter_ConsumerTick(config);
}

void FS_AudioControl_Init(void)
{
	const FS_Config_Data_t *config = FS_Config_Get();
	uint8_t i;

	// Initialize state
	memset(&state, 0, sizeof(state));
	state.x0 = INVALID_VALUE;

	// Initialize producer task
	UTIL_SEQ_RegTask(1<<CFG_TASK_FS_AUDIO_CONTROL_PRODUCER_ID, UTIL_SEQ_RFU, producerTask);

	// Initialize consumer task
	UTIL_SEQ_RegTask(1<<CFG_TASK_FS_AUDIO_CONTROL_CONSUMER_ID, UTIL_SEQ_RFU, consumerTask);

	// Initialize consumer timer
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &state.timer_id, hw_ts_Repeated, consumerTimer);
	HW_TS_Start(state.timer_id, CONSUMER_TIMER_TICKS);

	// AltModeSource say-altitude latch: pending ground-elevation announcement.
	if (config->alt_step > 0)
	{
		state.altmode.say_altitude = true;
	}

	for (i = 0; i < config->num_speech; ++i)
	{
		if (config->speech[i].mode == FS_CONFIG_MODE_ALTITUDE)
		{
			state.altmode.say_altitude = true;
		}
	}

	// StartupSource: queue init speech / file.
	if (config->init_mode == 1)
	{
		FS_Speech_BuildInitDigits(&state.speech);
	}
	else if (config->init_mode == 2)
	{
		if (strlen(config->init_filename))
		{
			FS_Speech_BuildInitFile(&state.speech, config->init_filename);
		}
	}
}

void FS_AudioControl_DeInit(void)
{
	// Delete update timer
	HW_TS_Delete(state.timer_id);
}

void FS_AudioControl_UpdateGNSS(const FS_GNSS_Data_t *current)
{
	// Call update task
	UTIL_SEQ_SetTask(1<<CFG_TASK_FS_AUDIO_CONTROL_PRODUCER_ID, CFG_SCH_PRIO_0);
}
