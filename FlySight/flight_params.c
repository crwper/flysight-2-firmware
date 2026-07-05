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

#include "flight_params.h"
#include "config.h"
#include "gnss.h"
#include "nav.h"

#define ABS(a) (((a) < 0) ? -(a) : (a))

#define INVALID_VALUE   INT32_MAX

static const uint16_t sas_table[] =
{
	1024, 1077, 1135, 1197,
	1265, 1338, 1418, 1505,
	1600, 1704, 1818, 1944
};

uint16_t FS_FlightParams_GetSpeedMul(const FS_Config_Data_t *config, int32_t hMSL)
{
	uint16_t speed_mul = 1024;

	if (config->use_sas)
	{
		if (hMSL < 0)
		{
			speed_mul = sas_table[0];
		}
		else if (hMSL >= 11534336L)
		{
			speed_mul = sas_table[11];
		}
		else
		{
			int32_t h = hMSL / 1024;
			uint16_t i = h / 1024;
			uint16_t j = h % 1024;
			uint16_t y1 = sas_table[i];
			uint16_t y2 = sas_table[i + 1];
			speed_mul = y1 + ((y2 - y1) * j) / 1024;
		}
	}

	return speed_mul;
}

void FS_FlightParams_GetValue(
	uint8_t mode,
	const FS_GNSS_Data_t *current,
	const FS_Config_Data_t *config,
	int32_t scale,
	int32_t *val,
	int32_t *min,
	int32_t *max)
{
	const int32_t velD = current->velD / 10;

	uint16_t speed_mul = FS_FlightParams_GetSpeedMul(config, current->hMSL);

	int32_t tVal;

	switch (mode)
	{
	case FS_CONFIG_MODE_HORIZONTAL_SPEED:
		*val = (current->gSpeed * 1024) / speed_mul;
		break;
	case FS_CONFIG_MODE_VERTICAL_SPEED:
		*val = (velD * 1024) / speed_mul;
		break;
	case FS_CONFIG_MODE_GLIDE_RATIO:
		if (velD != 0)
		{
			*val = scale * (int32_t) current->gSpeed / velD;
			*min *= 100;
			*max *= 100;
		}
		break;
	case FS_CONFIG_MODE_INVERSE_GLIDE_RATIO:
		if (current->gSpeed != 0)
		{
			*val = scale * velD / (int32_t) current->gSpeed;
			*min *= 100;
			*max *= 100;
		}
		break;
	case FS_CONFIG_MODE_TOTAL_SPEED:
		*val = (current->speed * 1024) / speed_mul;
		break;
	case FS_CONFIG_MODE_DIRECTION_TO_DESTINATION:
		//check if too far from destination for Nav, would indicate user error with Lat & Lon
		if ((calcDistance(current->lat,current->lon,config->lat,config->lon) < config->max_dist) || (config->max_dist == 0))
		{
			//check if above height tone should be silenced
			if ((current->hMSL > (config->end_nav+config->dz_elev)) || (config->end_nav == 0))
			{
				tVal=calcDirection(current->lat,current->lon,config->lat,config->lon,current->heading);
				//check if heading not within UBX_min_angle deg of bearing or tones needed for other measurement
				if ((ABS(tVal) > config->min_angle) || (config->mode_2 != FS_CONFIG_MODE_DIRECTION_TO_DESTINATION) || (config->min_angle==0))
				{
					*min = -180;
					*max = 180;
					//manipulate tone so biggest change is at desired heading
					if(tVal < 0)
					{
						*val = -180-tVal;
					}
					else
					{
						*val = 180-tVal;
					}
				}
			}
		}
		break;
	case FS_CONFIG_MODE_DISTANCE_TO_DESTINATION:
		*min = 0;
		if(config->max_dist != 0 )
		{
			*max = config->max_dist;
		}
		else
		{
			*max = 10000; //set a default maximum value
		}
		*val = calcDistance(current->lat,current->lon,config->lat,config->lon);
		if(*val < *max)
		{
			*val = *max-*val;  //make inverse so higher pitch indicates shorter distance
		}
		else
		{
			*val = 0;  //set to lowest pitch/Hz
		}
		break;
	case FS_CONFIG_MODE_DIRECTION_TO_BEARING: // Direction to bearing
		//check if above height tone should be silenced
		if ((current->hMSL > (config->end_nav+config->dz_elev)) || (config->end_nav == 0))
		{
			tVal=calcRelBearing(config->bearing,current->heading/100000);
			//check if heading not within UBX_min_angle deg of bearing or tones needed for other measurement
			if ((ABS(tVal) > config->min_angle) || (config->mode_2 != FS_CONFIG_MODE_DIRECTION_TO_BEARING) || (config->min_angle==0))
			{
				*min = -180;
				*max = 180;
				//manipulate tone so biggest change is at desired heading
				if(tVal < 0)
				{
					*val = -180-tVal;
				}
				else
				{
					*val = 180-tVal;
				}
			}
		}
		break;
	case FS_CONFIG_MODE_LEFT_RIGHT:
		//check if too far from destination for Nav, would indicate user error with Lat & Lon
		if ((calcDistance(current->lat,current->lon,config->lat,config->lon) < config->max_dist) || (config->max_dist == 0))
		{
			//check if above height tone should be silenced
			if ((current->hMSL > (config->end_nav+config->dz_elev)) || (config->end_nav == 0))
			{
				tVal=calcDirection(current->lat,current->lon,config->lat,config->lon,current->heading);
				*min = 0;
				*max = 10;
				if(ABS(tVal) > config->min_angle)
				{
					if(tVal < 0)   //left turn required  - low pitch tone
					{
						*val = *min;
					}
					else           //right turn required - high pitch tone
					{
						*val = *max;
					}
				}
				else              //mid tone
				{
					*val = (*max-*min)/2;
				}
			}
		}
		break;
	case FS_CONFIG_MODE_DIVE_ANGLE:
		*val = atan2(velD, current->gSpeed) / M_PI * 180;
		break;
	}
}

double FS_FlightParams_GetSASCorrectionFactor(int32_t hMSL)
{
	uint16_t speed_mul;

	if (hMSL < 0)
	{
		speed_mul = sas_table[0];
	}
	else if (hMSL >= 11534336L)
	{
		speed_mul = sas_table[11];
	}
	else
	{
		int32_t h = hMSL / 1024;
		uint16_t i = h / 1024;
		uint16_t j = h % 1024;
		uint16_t y1 = sas_table[i];
		uint16_t y2 = sas_table[i + 1];
		speed_mul = y1 + ((y2 - y1) * j) / 1024;
	}

	return speed_mul / 1024.0;
}
