/*
 * Fake GNSS data source. The run loop stores the current track point here;
 * audio_control's producer task reads it back via FS_GNSS_GetData(), same
 * as on target.
 */

#include <string.h>

#include "main.h"
#include "gnss.h"
#include "sim.h"

static FS_GNSS_Data_t current;

void sim_gnss_set(const FS_GNSS_Data_t *data)
{
	memcpy(&current, data, sizeof(current));
}

const FS_GNSS_Data_t *FS_GNSS_GetData(void)
{
	return &current;
}
