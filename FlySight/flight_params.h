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

#include <stdint.h>

#include "config.h"
#include "gnss.h"

// Speed-scaling correction factor (SAS) as an integer multiplier (base 1024).
// Returns 1024 when SAS is disabled. Shared by the tone and speech paths so
// the interpolation lives in exactly one place.
uint16_t FS_FlightParams_GetSpeedMul(const FS_Config_Data_t *config, int32_t hMSL);

// Shared flight-metric computation. Integer math is preserved exactly from
// the original tone path; the glide/inverse-glide scale differs between the
// tone path (10000) and the speech path (100), so it is a parameter rather
// than a constant -- the two computations must stay separate (integer
// division is not associative under a common scale).
void FS_FlightParams_GetValue(
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
