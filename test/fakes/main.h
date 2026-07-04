/*
 * Host-build stand-in for Core/Inc/main.h.
 *
 * Provides just enough HAL surface for the firmware sources compiled into
 * the audio simulator (audio_control.c, config.c, common.c). No hardware
 * access; peripheral handles are opaque dummies.
 */

#ifndef FAKE_MAIN_H_
#define FAKE_MAIN_H_

#include <stdint.h>

typedef enum
{
	HAL_OK      = 0x00,
	HAL_ERROR   = 0x01,
	HAL_BUSY    = 0x02,
	HAL_TIMEOUT = 0x03
} HAL_StatusTypeDef;

/* Opaque dummy handles (common.c declares `extern RNG_HandleTypeDef hrng`) */
typedef struct { int dummy; } RNG_HandleTypeDef;

HAL_StatusTypeDef HAL_RNG_GenerateRandomNumber(RNG_HandleTypeDef *hrng, uint32_t *random32bit);
uint32_t HAL_GetTick(void);

/* RNG/semaphore plumbing used by FS_Common_GetRandomBytes (not exercised
 * by audio paths); no-ops on the host */
#define HSEM 0
#define CFG_HW_RNG_SEMID 0
#define RCC_RNGCLKSOURCE_CLK48 0
#define LL_HSEM_1StepLock(hsem, sem)        ((void) 0)
#define LL_HSEM_ReleaseLock(hsem, sem, pid) ((void) 0)
#define LL_RCC_SetRNGClockSource(src)       ((void) 0)
#define MX_RNG_Init()                       ((void) 0)
#define HAL_RNG_DeInit(h)                   ((void) 0)

void Error_Handler(void);

#endif /* FAKE_MAIN_H_ */
