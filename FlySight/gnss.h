/*
 * gnss.h
 *
 *  Created on: Mar 3, 2020
 *      Author: Michael
 */

#ifndef GNSS_H_
#define GNSS_H_

#include <stdbool.h>

#define GNSS_RX_BUF_LEN		512						// Circular buffer for UART
#define GNSS_RAW_BUF_LEN	(GNSS_RX_BUF_LEN / 2)	// Circular buffer for raw output

typedef struct
{
	uint16_t year;     // Year                         (1999..2099)
	uint8_t  month;    // Month                        (1..12)
	uint8_t  day;      // Day of month                 (1..31)
	uint8_t  hour;     // Hour of day                  (0..23)
	uint8_t  min;      // Minute of hour               (0..59)
	uint8_t  sec;      // Second of minute             (0..59)
	int32_t  nano;     // Nanoseconds of second        (ns)

	int32_t  lon;      // Longitude                    (deg)
	int32_t  lat;      // Latitude                     (deg)
	int32_t  hMSL;     // Height above mean sea level  (mm)

	int32_t  velN;     // North velocity               (cm/s)
	int32_t  velE;     // East velocity                (cm/s)
	int32_t  velD;     // Down velocity                (cm/s)

	int32_t  speed;    // 3D speed                     (cm/s)
	int32_t  gSpeed;   // Ground speed                 (cm/s)

	uint32_t tAcc;     // Time accuracy estimate       (ns)
	uint32_t hAcc;     // Horizontal accuracy estimate (mm)
	uint32_t vAcc;     // Vertical accuracy estimate   (mm)
	uint32_t sAcc;     // Speed accuracy estimate      (cm/s)

	uint8_t  gpsFix;   // GPS fix type
	uint8_t  numSV;    // Number of SVs in solution
} FS_GNSS_Data_t;

typedef struct
{
	uint32_t time;		// ms
	uint32_t towMS;     // Time pulse time of week     (ms)
	uint16_t week;      // Time pulse week number
} FS_GNSS_Time_t;

typedef struct
{
	unsigned char buf[GNSS_RAW_BUF_LEN];
} FS_GNSS_Raw_t;

void FS_GNSS_Init(void);
void FS_GNSS_DeInit(void);

void FS_GNSS_Start(void);
void FS_GNSS_Stop(void);

const FS_GNSS_Data_t *FS_GNSS_GetData(void);
void FS_GNSS_DataReady_Callback(void);

void FS_GNSS_Timepulse(void);
const FS_GNSS_Time_t *FS_GNSS_GetTime(void);
void FS_GNSS_TimeReady_Callback(bool validTime);

const FS_GNSS_Raw_t *FS_GNSS_GetRaw(void);
void FS_GNSS_RawReady_Callback(void);

#endif /* GNSS_H_ */
