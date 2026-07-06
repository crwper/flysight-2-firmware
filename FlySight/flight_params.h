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

#ifndef FLIGHT_PARAMS_H_
#define FLIGHT_PARAMS_H_

#include <stdbool.h>
#include <stdint.h>

#include "config.h"
#include "gnss.h"

// SI flight sample. Retires the AVR-era integer scaling: speeds/altitude/angles
// are float SI (m, m/s, degrees), position is double degrees. Built from a raw
// FS_GNSS_Data_t in EXACTLY ONE place (FS_FlightData_FromGNSS); every metric
// consumer works from this struct. valid3d / vAccGood fold the old FLAG_HAS_FIX
// / FLAG_VERTICAL_ACC tests.
//
// nav.c (calcDistance/calcDirection/calcRelBearing) is unchanged in B1 and still
// takes the raw integer geo/heading scales, so the *_Raw helpers reconstruct
// those exact integers from the SI fields at the nav boundary (lossless: the
// round-trip recovers the original int32 for |value| < 2^52). heading keeps the
// raw deg*1e5 scale available so the Mode_2==7 recalc can preserve QUIRKS #16
// (that fix is B3).
typedef struct
{
	bool    valid3d;    // gpsFix == 3
	bool    vAccGood;   // vAcc < 10 m

	double  lat;        // deg
	double  lon;        // deg

	float   alt;        // m    (hMSL)
	float   velD;       // m/s  (down, +down)
	float   gSpeed;     // m/s  (ground speed)
	float   speed;      // m/s  (3D speed)
	float   heading;    // deg  (2D heading)
} FS_FlightData_t;

// Build the SI sample from a raw GNSS fix. The one and only conversion point.
FS_FlightData_t FS_FlightData_FromGNSS(const FS_GNSS_Data_t *g);

// SAS speed-scaling multiplier (base 1024), the one SAS implementation for the
// module. Kept integer: shared by the tone RATE path (must stay bit-identical,
// QUIRKS #5) and the speech speed path (whose value the renderer rounds at the
// last step, so the spoken digit shifts by at most +/-1). The 1024 constants
// are the sanctioned rate-path / driver-boundary survivors.
uint16_t FS_FlightParams_GetSpeedMul(const FS_Config_Data_t *config, int32_t hMSL);

// Integer metric for the tone value_1 (pitch base + gating) and value_2 (rate).
// The tone-rate accumulator must stay bit-identical (QUIRKS #5): a float rate
// value drifts the free-running 0x10000 phase and changes beep counts/timing,
// and a continuous-float value_1 would coarsen->smooth pitches (e.g. the dive
// angle, old-truncated to whole degrees) by far more than +/-1 Hz. So the tone
// metric is computed here with the original integer arithmetic, verbatim from
// the pre-B1 code -- the scale parameter is the glide scale (10000). setTone
// re-rounds the in-band pitch from this integer value; speech glide/dive values
// use the float FS_FlightData path, speech speeds this integer SAS.
void FS_FlightParams_GetRateValue(
	uint8_t mode,
	const FS_GNSS_Data_t *current,
	const FS_Config_Data_t *config,
	int32_t scale,
	int32_t *val,
	int32_t *min,
	int32_t *max);

// develop-branch-compatible signature: the SAS correction factor as a double
// (interpolated from the same table, /1024.0). Unused on this branch; it
// exists so the develop merge deletes their copy instead of conflicting.
double FS_FlightParams_GetSASCorrectionFactor(int32_t hMSL);

#endif /* FLIGHT_PARAMS_H_ */
