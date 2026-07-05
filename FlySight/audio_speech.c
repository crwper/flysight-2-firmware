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
#include <stdint.h>
#include <string.h>

#include "main.h"
#include "app_common.h"
#include "audio.h"
#include "audio_speech.h"
#include "config.h"
#include "flight_params.h"
#include "gnss.h"
#include "nav.h"

#define ABS(a) (((a) < 0) ? -(a) : (a))

// Token -> filename. Indexed by FS_SpeechToken_t; TOK_END and TOK_RAW_FILE
// carry no static name (RAW_FILE plays FS_Speech_t.raw_file instead).
static const char *const token_files[FS_SPEECH_TOKEN_COUNT] =
{
	[TOK_NUM_0]  = "0.wav",  [TOK_NUM_1]  = "1.wav",  [TOK_NUM_2]  = "2.wav",
	[TOK_NUM_3]  = "3.wav",  [TOK_NUM_4]  = "4.wav",  [TOK_NUM_5]  = "5.wav",
	[TOK_NUM_6]  = "6.wav",  [TOK_NUM_7]  = "7.wav",  [TOK_NUM_8]  = "8.wav",
	[TOK_NUM_9]  = "9.wav",  [TOK_NUM_10] = "10.wav", [TOK_NUM_11] = "11.wav",
	[TOK_NUM_12] = "12.wav", [TOK_NUM_13] = "13.wav", [TOK_NUM_14] = "14.wav",
	[TOK_NUM_15] = "15.wav", [TOK_NUM_16] = "16.wav", [TOK_NUM_17] = "17.wav",
	[TOK_NUM_18] = "18.wav", [TOK_NUM_19] = "19.wav",

	[TOK_TENS_2] = "20.wav", [TOK_TENS_3] = "30.wav", [TOK_TENS_4] = "40.wav",
	[TOK_TENS_5] = "50.wav", [TOK_TENS_6] = "60.wav", [TOK_TENS_7] = "70.wav",
	[TOK_TENS_8] = "80.wav", [TOK_TENS_9] = "90.wav",

	[TOK_HUNDRED]  = "00.wav",
	[TOK_THOUSAND] = "000.wav",
	[TOK_MINUS]    = "minus.wav",
	[TOK_DOT]      = "dot.wav",

	[TOK_UNIT_METERS] = "meters.wav",
	[TOK_UNIT_FEET]   = "feet.wav",
	[TOK_UNIT_KM]     = "km.wav",
	[TOK_UNIT_MILES]  = "miles.wav",
	[TOK_UNIT_KNOTS]  = "knots.wav",

	[TOK_LABEL_HORZ]     = "horz.wav",
	[TOK_LABEL_VERT]     = "vert.wav",
	[TOK_LABEL_GLIDE]    = "glide.wav",
	[TOK_LABEL_IGLIDE]   = "iglide.wav",
	[TOK_LABEL_SPEED]    = "speed.wav",
	[TOK_LABEL_DIRECTN]  = "directn.wav",
	[TOK_LABEL_DISTANCE] = "distance.wav",
	[TOK_LABEL_BEARING]  = "bearing.wav",
	[TOK_LABEL_DIVE]     = "dive.wav",
	[TOK_LABEL_ALT]      = "alt.wav",

	[TOK_SUFFIX_LEFT]  = "left.wav",
	[TOK_SUFFIX_RIGHT] = "right.wav",
};

// Speech label for a config Sp_Mode. Only modes the renderer voices (0..7, 11,
// 12) are reachable; anything else yields TOK_END so num_speech>1 emits no
// label wav, matching the legacy '>' switch's silent default.
static uint8_t labelToken(uint8_t mode)
{
	switch (mode)
	{
	case FS_CONFIG_MODE_HORIZONTAL_SPEED:         return TOK_LABEL_HORZ;
	case FS_CONFIG_MODE_VERTICAL_SPEED:           return TOK_LABEL_VERT;
	case FS_CONFIG_MODE_GLIDE_RATIO:              return TOK_LABEL_GLIDE;
	case FS_CONFIG_MODE_INVERSE_GLIDE_RATIO:      return TOK_LABEL_IGLIDE;
	case FS_CONFIG_MODE_TOTAL_SPEED:              return TOK_LABEL_SPEED;
	case FS_CONFIG_MODE_DIRECTION_TO_DESTINATION: return TOK_LABEL_DIRECTN;
	case FS_CONFIG_MODE_DISTANCE_TO_DESTINATION:  return TOK_LABEL_DISTANCE;
	case FS_CONFIG_MODE_DIRECTION_TO_BEARING:     return TOK_LABEL_BEARING;
	case FS_CONFIG_MODE_DIVE_ANGLE:               return TOK_LABEL_DIVE;
	case FS_CONFIG_MODE_ALTITUDE:                 return TOK_LABEL_ALT;
	default:                                      return TOK_END;
	}
}

// Digit-string encoder: writes a 2-decimal fixed-point value BACKWARD into buf
// ending at index idx, returning the new start index. Byte-for-byte the token
// analogue of common.c writeInt32ToBuf(ptr, val, 2, 1, 0) (delimiter '\0' ==
// TOK_END). Keeping this arithmetic identical is what makes the QUIRKS #12
// truncation pathology fall out naturally (see FS_Speech_BuildValue).
static int writeValueTokens(uint8_t *buf, int idx, int32_t val)
{
	int32_t value = val > 0 ? val : -val;
	int8_t dec = 2;

	buf[--idx] = TOK_END;
	while (value > 0 || dec > 0)
	{
		buf[--idx] = TOK_NUM_0 + (value % 10);
		value /= 10;
		if (--dec == 0)
		{
			buf[--idx] = TOK_DOT;
		}
	}
	if (buf[idx] == TOK_DOT || buf[idx] == TOK_END)
	{
		buf[--idx] = TOK_NUM_0;
	}
	if (val < 0)
	{
		buf[--idx] = TOK_MINUS;
	}

	return idx;
}

// Number-word encoder: writes teens/tens/hundreds/thousands FORWARD from index
// i, returning the new end index. Token analogue of the legacy numberToSpeech.
static int numberToTokens(uint8_t *buf, int i, int32_t number)
{
	if (number == 0)
	{
		buf[i++] = TOK_NUM_0;
		return i;
	}

	if (number < 0)
	{
		buf[i++] = TOK_MINUS;
		return numberToTokens(buf, i, -number);
	}

	if ((number / 1000) > 0)
	{
		i = numberToTokens(buf, i, number / 1000);
		buf[i++] = TOK_THOUSAND;
		number %= 1000;
	}

	if ((number / 100) > 0)
	{
		i = numberToTokens(buf, i, number / 100);
		buf[i++] = TOK_HUNDRED;
		number %= 100;
	}

	if (number > 0)
	{
		if (number < 10)
		{
			buf[i++] = TOK_NUM_0 + number;
		}
		else if (number < 20)
		{
			buf[i++] = TOK_NUM_0 + number;      // teens: "1X.wav"
		}
		else
		{
			buf[i++] = TOK_TENS_2 + (number / 10 - 2);
			if ((number % 10) > 0)
			{
				buf[i++] = TOK_NUM_0 + (number % 10);
			}
		}
	}

	return i;
}

bool FS_Speech_HasPending(const FS_Speech_t *sp)
{
	return sp->buf[sp->pos] != TOK_END;
}

void FS_Speech_Clear(FS_Speech_t *sp)
{
	sp->buf[sp->pos] = TOK_END;
}

bool FS_Speech_PlayNext(FS_Speech_t *sp, const FS_Config_Data_t *config)
{
	uint8_t t = sp->buf[sp->pos];

	if (t == TOK_END)
	{
		return false;
	}

	if (t == TOK_RAW_FILE)
	{
		FS_Audio_Play(sp->raw_file, config->sp_volume * 5);
	}
	else if (t < FS_SPEECH_TOKEN_COUNT && token_files[t])
	{
		FS_Audio_Play(token_files[t], config->sp_volume * 5);
	}

	++sp->pos;
	return true;
}

void FS_Speech_BuildValue(
	FS_Speech_t *sp,
	const FS_Config_Data_t *config,
	const FS_GNSS_Data_t *current,
	uint8_t cur_speech)
{
	const int32_t velD = current->velD / 10;

	const uint8_t  mode  = config->speech[cur_speech].mode;
	const uint8_t  units = config->speech[cur_speech].units;
	int32_t decimals = config->speech[cur_speech].decimals;

	uint16_t speed_mul = FS_FlightParams_GetSpeedMul(config, current->hMSL);
	int32_t step_size, step;

	uint8_t *buf = sp->buf;
	int ptr, end;

	// Direction suffix (modes 5/7): only voiced when the value is actually
	// computed. The legacy code read an uninitialized tVal here when the
	// nav gates skipped the calculation (QUIRKS #13); that garbage was
	// written below the read cursor and never played in any golden, so
	// omitting it is byte-identical for the goldens.
	bool dir_valid = false;
	int32_t dir_val = 0;

	switch (units)
	{
	case FS_CONFIG_UNITS_KMH:
		speed_mul = (uint16_t) (((uint32_t) speed_mul * 18204) / 65536);
		break;
	case FS_CONFIG_UNITS_MPH:
		speed_mul = (uint16_t) (((uint32_t) speed_mul * 29297) / 65536);
		break;
	case FS_CONFIG_UNITS_KNOTS:
		speed_mul = (uint16_t) (((uint32_t) speed_mul * 33713) / 65536);
		break;
	}

	// Step 0: cursor and end index start one past the buffer's last slot's
	// value region, mirroring speech_ptr / end_ptr = speech_buf + size - 1.
	ptr = FS_SPEECH_QUEUE_SIZE - 1;
	end = ptr;

	// Step 1: Encode the value with 2 decimal places (or leave empty).
	switch (mode)
	{
	case FS_CONFIG_MODE_HORIZONTAL_SPEED:
		ptr = writeValueTokens(buf, ptr, (current->gSpeed * 1024) / speed_mul);
		break;
	case FS_CONFIG_MODE_VERTICAL_SPEED:
		ptr = writeValueTokens(buf, ptr, (velD * 1024) / speed_mul);
		break;
	case FS_CONFIG_MODE_GLIDE_RATIO:
		if (velD != 0)
		{
			ptr = writeValueTokens(buf, ptr, 100 * (int32_t) current->gSpeed / velD);
		}
		else
		{
			buf[--ptr] = TOK_END;
		}
		break;
	case FS_CONFIG_MODE_INVERSE_GLIDE_RATIO:
		if (current->gSpeed != 0)
		{
			ptr = writeValueTokens(buf, ptr, 100 * (int32_t) velD / current->gSpeed);
		}
		else
		{
			buf[--ptr] = TOK_END;
		}
		break;
	case FS_CONFIG_MODE_TOTAL_SPEED:
		ptr = writeValueTokens(buf, ptr, (current->speed * 1024) / speed_mul);
		break;
	case FS_CONFIG_MODE_DIRECTION_TO_DESTINATION:
		if ((calcDistance(current->lat, current->lon, config->lat, config->lon) < config->max_dist) ||
		    (config->max_dist == 0))
		{
			if ((current->hMSL > (config->end_nav + config->dz_elev)) || (config->end_nav == 0))
			{
				decimals = 0;
				dir_val = calcDirection(current->lat, current->lon, config->lat, config->lon, current->heading);
				dir_valid = true;
				ptr = writeValueTokens(buf, ptr, ABS(dir_val) * 100);
			}
		}
		break;
	case FS_CONFIG_MODE_DISTANCE_TO_DESTINATION:
	{
		int32_t tVal;
		decimals = 1;
		tVal = calcDistance(current->lat, current->lon, config->lat, config->lon);
		switch (units)
		{
		case FS_CONFIG_UNITS_METERS:
			tVal = tVal / 10;
			break;
		case FS_CONFIG_UNITS_FEET:
			tVal = (tVal * 100) / 1609;
			break;
		case FS_CONFIG_UNITS_NM:
			tVal = (tVal * 100) / 1852;
			break;
		}
		tVal = tVal + 5; // for correct rounding when reducing to one decimal place
		ptr = writeValueTokens(buf, ptr, tVal);
		break;
	}
	case FS_CONFIG_MODE_DIRECTION_TO_BEARING:
		if ((current->hMSL > (config->end_nav + config->dz_elev)) || (config->end_nav == 0))
		{
			decimals = 0;
			dir_val = calcRelBearing(config->bearing, current->heading / 100000);
			dir_valid = true;
			ptr = writeValueTokens(buf, ptr, ABS(dir_val) * 100);
		}
		break;
	case FS_CONFIG_MODE_DIVE_ANGLE:
		ptr = writeValueTokens(buf, ptr, 100 * atan2(velD, current->gSpeed) / M_PI * 180);
		break;
	case FS_CONFIG_MODE_ALTITUDE:
		if (units == FS_CONFIG_UNITS_METERS)
		{
			step_size = 10000 * decimals;
		}
		else
		{
			step_size = 3048 * decimals;
		}
		step = ((current->hMSL - config->dz_elev) * 10 + step_size / 2) / step_size;
		end = numberToTokens(buf, 2, step * decimals);
		ptr = 2;
		break;
	}

	// Step 1.5: Include label (one token, was '>' + mode+1).
	if (config->num_speech > 1)
	{
		buf[--ptr] = labelToken(mode);
	}

	// Step 2: Truncate to the desired number of decimal places. Same index
	// arithmetic as the legacy end_ptr adjustment; for an empty value the
	// terminator can land on the label token (QUIRKS #12): Sp_Dec 1 silences
	// the whole utterance, Sp_Dec 0/2 leave the label alone.
	if (mode != FS_CONFIG_MODE_ALTITUDE)
	{
		if (decimals == 0) end -= 4;
		else               end -= 3 - decimals;
	}

	// Step 3: Units / left-right suffix.
	switch (mode)
	{
	case FS_CONFIG_MODE_HORIZONTAL_SPEED:
	case FS_CONFIG_MODE_VERTICAL_SPEED:
	case FS_CONFIG_MODE_GLIDE_RATIO:
	case FS_CONFIG_MODE_INVERSE_GLIDE_RATIO:
	case FS_CONFIG_MODE_TOTAL_SPEED:
	case FS_CONFIG_MODE_DIVE_ANGLE:
		break;
	case FS_CONFIG_MODE_DIRECTION_TO_DESTINATION:
	case FS_CONFIG_MODE_DIRECTION_TO_BEARING:
		if (dir_valid)
		{
			if (dir_val < 0)      buf[end++] = TOK_SUFFIX_LEFT;
			else if (dir_val > 0) buf[end++] = TOK_SUFFIX_RIGHT;
		}
		break;
	case FS_CONFIG_MODE_DISTANCE_TO_DESTINATION:
		switch (units)
		{
		case FS_CONFIG_UNITS_METERS:
			buf[end++] = TOK_UNIT_KM;
			break;
		case FS_CONFIG_UNITS_FEET:
			buf[end++] = TOK_UNIT_MILES;
			break;
		case FS_CONFIG_UNITS_NM:
			buf[end++] = TOK_UNIT_KNOTS;
			break;
		}
		break;
	case FS_CONFIG_MODE_ALTITUDE:
		buf[end++] = (units == FS_CONFIG_UNITS_METERS) ? TOK_UNIT_METERS : TOK_UNIT_FEET;
		break;
	}

	// Step 4: Terminate and rewind the cursor to the utterance start.
	buf[end++] = TOK_END;
	sp->pos = (uint8_t) ptr;
}

void FS_Speech_BuildAltStep(
	FS_Speech_t *sp,
	const FS_Config_Data_t *config,
	int32_t step)
{
	int i = 0;

	i = numberToTokens(sp->buf, i, step * config->alt_step);
	sp->buf[i++] = (config->alt_units == FS_CONFIG_UNITS_METERS) ? TOK_UNIT_METERS : TOK_UNIT_FEET;
	sp->buf[i++] = TOK_END;
	sp->pos = 0;
}

void FS_Speech_BuildGroundElev(
	FS_Speech_t *sp,
	const FS_Config_Data_t *config,
	int32_t prevHMSL)
{
	int i = 0;

	if (config->alt_units == FS_CONFIG_UNITS_METERS)
	{
		i = numberToTokens(sp->buf, i, (prevHMSL - config->dz_elev) / 1000);
		sp->buf[i++] = TOK_UNIT_METERS;
	}
	else
	{
		i = numberToTokens(sp->buf, i, (prevHMSL - config->dz_elev) * 10 / 3048);
		sp->buf[i++] = TOK_UNIT_FEET;
	}
	sp->buf[i++] = TOK_END;
	sp->pos = 0;
}

void FS_Speech_BuildInitDigits(FS_Speech_t *sp)
{
	uint8_t i = 0;
	uint8_t d;

	for (d = 0; d < 10; ++d)
	{
		sp->buf[i++] = TOK_NUM_0 + d;
	}
	sp->buf[i++] = TOK_DOT;
	sp->buf[i++] = TOK_MINUS;
	sp->buf[i++] = TOK_END;
	sp->pos = 0;
}

void FS_Speech_BuildInitFile(FS_Speech_t *sp, const char *init_filename)
{
	sp->raw_file[0] = '\0';
	strncat(sp->raw_file, init_filename, sizeof(sp->raw_file) - 1);
	strncat(sp->raw_file, ".wav", sizeof(sp->raw_file) - strlen(sp->raw_file) - 1);

	sp->buf[0] = TOK_RAW_FILE;
	sp->buf[1] = TOK_END;
	sp->pos = 0;
}
