/*
 * Host-build stand-in for the STM32 sequencer (stm32_seq.h).
 * Implemented in fake_seq_ts.c on top of the simulator core.
 */

#ifndef FAKE_STM32_SEQ_H_
#define FAKE_STM32_SEQ_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UTIL_SEQ_RFU 0

void UTIL_SEQ_RegTask(uint32_t TaskId_bm, uint32_t Flags, void (*Task)(void));
void UTIL_SEQ_SetTask(uint32_t TaskId_bm, uint32_t Task_Prio);

#ifdef __cplusplus
}
#endif

#endif /* FAKE_STM32_SEQ_H_ */
