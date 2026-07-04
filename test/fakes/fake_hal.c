/* Stubs for the few HAL symbols referenced by compiled firmware sources */

#include <stdio.h>
#include <stdlib.h>

#include "main.h"
#include "sim.h"

HAL_StatusTypeDef HAL_RNG_GenerateRandomNumber(RNG_HandleTypeDef *hrng, uint32_t *random32bit)
{
	(void) hrng;
	*random32bit = 0x12345678; /* deterministic; not used by audio paths */
	return HAL_OK;
}

uint32_t HAL_GetTick(void)
{
	return (uint32_t) (sim_now_ns() / 1000000ULL);
}

RNG_HandleTypeDef hrng;

void Error_Handler(void)
{
	fprintf(stderr, "Error_Handler called\n");
	exit(2);
}
