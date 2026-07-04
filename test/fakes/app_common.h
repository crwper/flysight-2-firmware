/*
 * Host-build stand-in for Core/Inc/app_common.h (+ the parts of app_conf.h
 * and hw_if.h that the simulated firmware sources use).
 *
 * Values below mirror the target build:
 *   - CFG_TS_TICK_VAL = DIVR(16 * 1000000, 32774) = 488 us per RTC tick
 *     (CFG_RTCCLK_DIV = 16, LSE_VALUE = 32774; see Core/Inc/app_conf.h and
 *     Core/Inc/stm32wbxx_hal_conf.h). audio_control.c uses this to convert
 *     its 10 ms consumer period into RTC ticks (10000/488 = 20 ticks).
 *   - Task IDs: only relative order matters (the fake sequencer drains
 *     pending tasks in ascending ID order, like the target sequencer within
 *     one priority). PRODUCER < CONSUMER matches app_conf.h.
 */

#ifndef FAKE_APP_COMMON_H_
#define FAKE_APP_COMMON_H_

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FALSE 0
#define TRUE  (!0)

#define MAX( x, y )  (((x)>(y))?(x):(y))
#define MIN( x, y )  (((x)<(y))?(x):(y))

#define DIVR( x, y ) (((x)+((y)/2))/(y))

#define CFG_TS_TICK_VAL 488 /* us per RTC tick, matches target */

typedef enum
{
	CFG_TASK_FS_AUDIO_CONTROL_PRODUCER_ID,
	CFG_TASK_FS_AUDIO_CONTROL_CONSUMER_ID,
	CFG_TASK_NBR /* keep last */
} CFG_Task_Id_With_HCI_Cmd_t;

typedef enum
{
	CFG_SCH_PRIO_0,
	CFG_SCH_PRIO_1,
	CFG_SCH_PRIO_NBR
} CFG_SCH_Prio_t;

#define CFG_TIM_PROC_ID_ISR 0

/* ---- Timer server API (hw_if.h) ---- */

typedef enum
{
	hw_ts_SingleShot,
	hw_ts_Repeated
} HW_TS_Mode_t;

typedef enum
{
	hw_ts_Successful,
	hw_ts_Failed
} HW_TS_ReturnStatus_t;

typedef void (*HW_TS_pTimerCb_t)(void);

HW_TS_ReturnStatus_t HW_TS_Create(uint32_t TimerProcessID, uint8_t *pTimerId, HW_TS_Mode_t TimerMode, HW_TS_pTimerCb_t pTimerCallBack);
void HW_TS_Start(uint8_t TimerID, uint32_t timeout_ticks);
void HW_TS_Stop(uint8_t TimerID);
void HW_TS_Delete(uint8_t TimerID);

#ifdef __cplusplus
}
#endif

#endif /* FAKE_APP_COMMON_H_ */
