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

// Speech utterance builder + token->wav renderer for audio_control.
//
// Replaces the legacy sentinel-character bytecode (speech_buf/speech_ptr,
// numberToSpeech, the consumerTask if/else ladder). An utterance is an array
// of enum tokens terminated by TOK_END; the renderer starts ONE wav per idle
// consumer tick while tokens remain, matching the old one-char-or-two-per-tick
// interpreter exactly (byte-identical traces are the contract in Phase A).

#ifndef AUDIO_SPEECH_H
#define AUDIO_SPEECH_H

#include <stdbool.h>
#include <stdint.h>

#include "config.h"
#include "gnss.h"

// Utterance token alphabet. TOK_END == 0 is the queue terminator (mirrors the
// legacy '\0'): a freshly zeroed queue is empty, and writing TOK_END anywhere
// truncates playback at that point.
typedef enum
{
	TOK_END = 0,

	// Cardinal numbers 0..19 -> "0.wav".."19.wav". Digit strings use 0..9;
	// number words (altitude) reuse 10..19 for the teens ("13" -> "13.wav").
	TOK_NUM_0,  TOK_NUM_1,  TOK_NUM_2,  TOK_NUM_3,  TOK_NUM_4,
	TOK_NUM_5,  TOK_NUM_6,  TOK_NUM_7,  TOK_NUM_8,  TOK_NUM_9,
	TOK_NUM_10, TOK_NUM_11, TOK_NUM_12, TOK_NUM_13, TOK_NUM_14,
	TOK_NUM_15, TOK_NUM_16, TOK_NUM_17, TOK_NUM_18, TOK_NUM_19,

	// Tens 20..90 -> "20.wav".."90.wav".
	TOK_TENS_2, TOK_TENS_3, TOK_TENS_4, TOK_TENS_5,
	TOK_TENS_6, TOK_TENS_7, TOK_TENS_8, TOK_TENS_9,

	TOK_HUNDRED,        // "00.wav"
	TOK_THOUSAND,       // "000.wav"
	TOK_MINUS,          // "minus.wav"
	TOK_DOT,            // "dot.wav"

	TOK_UNIT_METERS,    // "meters.wav"
	TOK_UNIT_FEET,      // "feet.wav"
	TOK_UNIT_KM,        // "km.wav"
	TOK_UNIT_MILES,     // "miles.wav"
	TOK_UNIT_KNOTS,     // "knots.wav"

	TOK_LABEL_HORZ,     // "horz.wav"     (mode 0)
	TOK_LABEL_VERT,     // "vert.wav"     (mode 1)
	TOK_LABEL_GLIDE,    // "glide.wav"    (mode 2)
	TOK_LABEL_IGLIDE,   // "iglide.wav"   (mode 3)
	TOK_LABEL_SPEED,    // "speed.wav"    (mode 4)
	TOK_LABEL_DIRECTN,  // "directn.wav"  (mode 5)
	TOK_LABEL_DISTANCE, // "distance.wav" (mode 6)
	TOK_LABEL_BEARING,  // "bearing.wav"  (mode 7)
	TOK_LABEL_DIVE,     // "dive.wav"     (mode 11)
	TOK_LABEL_ALT,      // "alt.wav"      (mode 12)

	TOK_SUFFIX_LEFT,    // "left.wav"
	TOK_SUFFIX_RIGHT,   // "right.wav"

	TOK_RAW_FILE,       // plays FS_Speech_t.raw_file (Init_Mode 2)

	FS_SPEECH_TOKEN_COUNT
} FS_SpeechToken_t;

// Queue capacity. Mirrors the legacy speech_buf[16]: the utterance-building
// index arithmetic (and the QUIRKS #12 truncation pathology) is expressed
// against this size, so it must stay 16.
#define FS_SPEECH_QUEUE_SIZE 16

typedef struct
{
	uint8_t buf[FS_SPEECH_QUEUE_SIZE]; // token stream, TOK_END-terminated
	uint8_t pos;                       // read cursor (mirrors speech_ptr)
	char    raw_file[13];              // payload for TOK_RAW_FILE
} FS_Speech_t;

// True while tokens remain to be played from the current cursor.
bool FS_Speech_HasPending(const FS_Speech_t *sp);

// Drop everything queued from the cursor onward (mirrors *speech_ptr = '\0').
// A wav already handed to the driver keeps playing; nothing further starts.
void FS_Speech_Clear(FS_Speech_t *sp);

// Play the token under the cursor and advance. Returns false (no-op) when the
// queue is empty. One call per idle consumer tick reproduces the old cadence.
bool FS_Speech_PlayNext(FS_Speech_t *sp, const FS_Config_Data_t *config);

// Utterance builders. Each fills the queue and rewinds the cursor to the start.
void FS_Speech_BuildValue(FS_Speech_t *sp,
                          const FS_Config_Data_t *config,
                          const FS_GNSS_Data_t *current,
                          uint8_t cur_speech);
void FS_Speech_BuildAltStep(FS_Speech_t *sp,
                            const FS_Config_Data_t *config,
                            int32_t step);
void FS_Speech_BuildGroundElev(FS_Speech_t *sp,
                               const FS_Config_Data_t *config,
                               int32_t prevHMSL);
void FS_Speech_BuildInitDigits(FS_Speech_t *sp);
void FS_Speech_BuildInitFile(FS_Speech_t *sp, const char *init_filename);

#endif // AUDIO_SPEECH_H
