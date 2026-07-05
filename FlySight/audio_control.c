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

#define FLAG_HAS_FIX         0x01
#define FLAG_FIRST_FIX       0x02
#define FLAG_BEEP_DONE       0x04
#define FLAG_SAY_ALTITUDE    0x08
#define FLAG_VERTICAL_ACC    0x10

#define TONE_MIN_PITCH 220
#define TONE_MAX_PITCH 1760

// Alarm crossing / alarm-window source (split out of updateAlarms in A4).
// prevHMSL is shared producer input passed explicitly, not stored here; the
// per-source private fields are added by later cards (A5).
typedef struct
{
	uint8_t _reserved;
} AlarmSource_t;

// Altitude-mode source: silence windows + Alt_Step suppression and the
// Alt_Step announcement (split out of updateAlarms in A4). Step tracking and
// the ground-elevation latch move here in later cards (A5).
typedef struct
{
	uint8_t _reserved;
} AltModeSource_t;

typedef struct
{
	uint8_t timer_id;

	uint8_t cur_speech;

	uint16_t sp_counter;

	uint8_t flags;
	uint8_t prev_flags;

	int32_t prevHMSL;

	uint8_t g_suppress_tone;

	AlarmSource_t   alarm;
	AltModeSource_t altmode;

	FS_Speech_t speech;

	volatile uint32_t tonePitch;
	volatile int32_t  toneChirp;
	volatile uint16_t toneRate;
	volatile uint8_t  toneHold;

	// CHANGE_IN_VALUE_1 history (formerly function-local statics in updateTones)
	int32_t x0, x1, x2;

	// Tone-rate accumulator (formerly function-local static in consumerTimer)
	uint16_t tone_timer;
} FS_AudioControl_State_t;

static FS_AudioControl_State_t state;

static void setRate(uint16_t rate)
{
	state.toneRate = rate;
}

static void setPitch(uint16_t pitch)
{
	state.tonePitch = pitch;
}

static void setChirp(uint32_t chirp)
{
	state.toneChirp = chirp;
}

static void setTone(
	FS_Config_Data_t *config,
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
			setPitch(TONE_MIN_PITCH + (TONE_MAX_PITCH - TONE_MIN_PITCH) * (val_1 - min_1) / (max_1 - min_1));
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

static void getValues(
	FS_GNSS_Data_t *current,
	FS_Config_Data_t *config,
	uint8_t mode,
	int32_t *val,
	int32_t *min,
	int32_t *max)
{
	// Tone path: glide/inverse-glide use a scale of 10000 (see flight_params).
	FS_FlightParams_GetValue(mode, current, config, 10000, val, min, max);
}

static void speakValue(
	FS_Config_Data_t *config,
	FS_GNSS_Data_t *current)
{
	FS_Speech_BuildValue(&state.speech, config, current, state.cur_speech);
}

// Alarm source: alarm-window suppression (every sample) and the alarm-crossing
// scan ([min,max) interval, QUIRKS #8). Emits its suppression contribution and,
// when prev sample had a fix, the index of the alarm to fire (num_alarms = none)
// as a request; the actual sound / speech-cancel is applied by the caller AFTER
// the shared rising-edge stop, preserving Stop-before-Beep ordering.
static void AlarmSource_Update(
	FS_Config_Data_t *config,
	FS_GNSS_Data_t *current,
	bool prev_had_fix,
	int32_t prevHMSL,
	uint8_t *suppress_tone,
	uint8_t *fired_index)
{
	uint8_t i;

	for (i = 0; i < config->num_alarms; ++i)
	{
		const int32_t alarm_elev = config->alarms[i].elev + config->dz_elev;

		if ((current->hMSL <= alarm_elev + config->alarm_window_above) &&
		    (current->hMSL >= alarm_elev - config->alarm_window_below))
		{
			*suppress_tone = 1;
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
// announcement decision. want_alt_step folds in every self-contained guard
// condition; the caller ANDs it with "no alarm fired" and "speech queue empty"
// (read after the rising-edge stop) before building, matching the old order.
static void AltModeSource_Update(
	FS_Config_Data_t *config,
	FS_GNSS_Data_t *current,
	bool prev_had_fix,
	int32_t prevHMSL,
	uint8_t *suppress_tone,
	uint8_t *suppress_alt,
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
			*suppress_tone = 1;
			*suppress_alt = 1;
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
			*suppress_tone = 1;
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
		    !(state.flags & FLAG_SAY_ALTITUDE) &&
		    !*suppress_alt &&
		    (step_elev >= min && step_elev < max) &&
		    ABS(velD) >= config->threshold &&
		    current->gSpeed >= config->hThreshold)
		{
			*want_alt_step = true;
		}
	}
}

// Emit the sound for a fired alarm crossing (AlarmSource request). Applied by
// producerTask after the rising-edge stop so a suppression-zone Stop cannot
// swallow the alarm beep.
static void fireAlarm(
	FS_Config_Data_t *config,
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

static void updateTones(
	FS_Config_Data_t *config,
	FS_GNSS_Data_t *current)
{
	const int32_t velD = current->velD / 10;

	int32_t val_1 = INVALID_VALUE, min_1 = config->min, max_1 = config->max;
	int32_t val_2 = INVALID_VALUE, min_2 = config->min_2, max_2 = config->max_2;

	uint8_t i;

	getValues(current, config, config->mode, &val_1, &min_1, &max_1);

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
			val_2 = ABS(calcRelBearing(config->bearing,current->heading));
			val_2 = 180-val_2;  //make inverse so faster rate indicates closer to bearing
		}
		min_2 = 0;
		max_2 = 180;
	}
	else if (config->mode_2 == FS_CONFIG_MODE_MAGNITUDE_OF_VALUE_1)
	{
		getValues(current, config, config->mode, &val_2, &min_2, &max_2);
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
		getValues(current, config, config->mode_2, &val_2, &min_2, &max_2);
	}

	if (!state.g_suppress_tone)
	{
		if (ABS(velD) >= config->threshold &&
			current->gSpeed >= config->hThreshold)
		{
			setTone(config, val_1, min_1, max_1, val_2, min_2, max_2);

			if (config->sp_rate != 0 &&
			    config->num_speech != 0 &&
			    state.sp_counter >= config->sp_rate &&
				!FS_Speech_HasPending(&state.speech) &&
				!(state.flags & FLAG_SAY_ALTITUDE))
			{
				for (i = 0; i < config->num_speech; ++i)
				{
					if ((config->speech[state.cur_speech].mode != FS_CONFIG_MODE_ALTITUDE) ||
						(current->hMSL - config->dz_elev >= ALT_MIN * 1000))
					{
						speakValue(config, current);
						state.cur_speech = (state.cur_speech + 1) % config->num_speech;
						break;
					}
					else
					{
						state.cur_speech = (state.cur_speech + 1) % config->num_speech;
					}
				}

				state.sp_counter = 0;
			}
		}
		else
		{
			setRate(0);
		}
	}

	if (state.sp_counter < config->sp_rate)
	{
		state.sp_counter += config->rate;
	}
}

static void producerTask(void)
{
	FS_Config_Data_t config;
	FS_GNSS_Data_t current;

	// Copy to local variable
	memcpy(&config, FS_Config_Get(), sizeof(FS_Config_Data_t));
	memcpy(&current, FS_GNSS_GetData(), sizeof(FS_GNSS_Data_t));

	if (current.gpsFix == 3)
	{
		state.flags |= FLAG_HAS_FIX;

		{
			const bool prev_had_fix = (state.prev_flags & FLAG_HAS_FIX) != 0;

			uint8_t suppress_tone = 0;
			uint8_t suppress_alt = 0;
			uint8_t fired_index;
			bool want_alt_step;
			int32_t step;

			AlarmSource_Update(&config, &current, prev_had_fix, state.prevHMSL,
			                   &suppress_tone, &fired_index);
			AltModeSource_Update(&config, &current, prev_had_fix, state.prevHMSL,
			                     &suppress_tone, &suppress_alt, &want_alt_step, &step);

			// Rising-edge stop: entering a suppression zone stops the sound.
			if (suppress_tone && !state.g_suppress_tone)
			{
				FS_Speech_Clear(&state.speech);
				setRate(0);
				FS_Audio_Stop();
			}

			state.g_suppress_tone = suppress_tone;

			if (prev_had_fix)
			{
				if (fired_index != config.num_alarms)
				{
					fireAlarm(&config, fired_index);
					FS_Speech_Clear(&state.speech);
				}
				else if (want_alt_step && !FS_Speech_HasPending(&state.speech))
				{
					FS_Speech_BuildAltStep(&state.speech, &config, step);
				}
			}
		}

		updateTones(&config, &current);

		if (!(state.flags & FLAG_BEEP_DONE))
		{
			state.flags |= FLAG_FIRST_FIX;
		}
	}
	else
	{
		state.flags &= ~FLAG_HAS_FIX;
		setRate(0);
	}

	if (current.vAcc < 10000)
	{
		state.flags |= FLAG_VERTICAL_ACC;
	}
	else
	{
		state.flags &= ~FLAG_VERTICAL_ACC;
	}

	state.prev_flags = state.flags;
	state.prevHMSL = current.hMSL;
}

static void consumerTimer(void)
{
	const FS_Config_Data_t *config = FS_Config_Get();

	if (FS_Audio_IsIdle() && !state.toneHold && state.toneRate > 0 && 0x10000 - state.tone_timer <= state.toneRate)
	{
		FS_Audio_Beep(state.tonePitch, state.tonePitch + state.toneChirp, 125, config->volume * 5);
	}

	state.tone_timer += state.toneRate;

	// Call consumer task
	UTIL_SEQ_SetTask(1<<CFG_TASK_FS_AUDIO_CONTROL_CONSUMER_ID, CFG_SCH_PRIO_0);
}

static void consumerTask(void)
{
	const FS_Config_Data_t *config = FS_Config_Get();

	if (FS_Speech_HasPending(&state.speech))
	{
		if (FS_Audio_IsIdle())
		{
			state.toneHold = 1;

			FS_Speech_PlayNext(&state.speech, config);
		}
	}
	else
	{
		const FS_Config_Data_t *config = FS_Config_Get();

		state.toneHold = 0;

		if ((state.flags & FLAG_FIRST_FIX) && FS_Audio_IsIdle())
		{
			state.flags &= ~FLAG_FIRST_FIX;
			FS_Audio_Beep(TONE_MAX_PITCH, TONE_MAX_PITCH, 125, config->volume * 5);
			state.flags |= FLAG_BEEP_DONE;
		}

		if ((state.flags & FLAG_SAY_ALTITUDE) &&
			(state.flags & FLAG_HAS_FIX) &&
		    (state.flags & FLAG_VERTICAL_ACC) &&
			FS_Audio_IsIdle())
		{
			state.flags &= ~FLAG_SAY_ALTITUDE;

			FS_Speech_BuildGroundElev(&state.speech, config, state.prevHMSL);
		}
	}
}

void FS_AudioControl_Init(void)
{
	const FS_Config_Data_t *config = FS_Config_Get();
	uint8_t i;

	// Initialize state
	memset(&state, 0, sizeof(state));
	state.cur_speech = 0;
	state.sp_counter = 0;
	state.flags = 0;
	state.prev_flags = 0;
	state.g_suppress_tone = 0;
	state.tonePitch = 0;
	state.toneChirp = 0;
	state.toneRate = 0;
	state.toneHold = 0;
	state.x0 = INVALID_VALUE;

	// Initialize producer task
	UTIL_SEQ_RegTask(1<<CFG_TASK_FS_AUDIO_CONTROL_PRODUCER_ID, UTIL_SEQ_RFU, producerTask);

	// Initialize consumer task
	UTIL_SEQ_RegTask(1<<CFG_TASK_FS_AUDIO_CONTROL_CONSUMER_ID, UTIL_SEQ_RFU, consumerTask);

	// Initialize consumer timer
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &state.timer_id, hw_ts_Repeated, consumerTimer);
	HW_TS_Start(state.timer_id, CONSUMER_TIMER_TICKS);

	if (config->alt_step > 0)
	{
		state.flags |= FLAG_SAY_ALTITUDE;
	}

	for (i = 0; i < config->num_speech; ++i)
	{
		if (config->speech[i].mode == FS_CONFIG_MODE_ALTITUDE)
		{
			state.flags |= FLAG_SAY_ALTITUDE;
		}
	}

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
