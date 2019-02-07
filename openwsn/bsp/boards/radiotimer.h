#ifndef __RADIOTIMER_H
#define __RADIOTIMER_H

/**
\addtogroup BSP
\{
\addtogroup radiotimer
\{

\brief Cross-platform declaration "radiotimer" bsp module.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, February 2012.
*/

#include "stdint.h"
#include "board.h"

//=========================== define ==========================================
#define MTOVF_CMP1 0x03
#define MTOVF_CMP2 0x04

#define MACTIMER_BASE 0.32
//=========================== typedef =========================================

typedef void (*radiotimer_compare_cbt)(void);
typedef void (*radiotimer_capture_cbt)(PORT_TIMER_WIDTH timestamp);

//=========================== variables =======================================

//=========================== prototypes ======================================

// admin
void     radiotimer_init(void);
void     radiotimer_setOverflowCb(radiotimer_compare_cbt cb);
void     radiotimer_setCompareCb(radiotimer_compare_cbt cb);
void     radiotimer_setStartFrameCb(radiotimer_capture_cbt cb);
void     radiotimer_setEndFrameCb(radiotimer_capture_cbt cb);
void     radiotimer_start(PORT_RADIOTIMER_WIDTH period);
// direct access
PORT_RADIOTIMER_WIDTH radiotimer_getValue(void);
void radiotimer_clearValue(void);
void     radiotimer_setPeriod(PORT_RADIOTIMER_WIDTH period);
PORT_RADIOTIMER_WIDTH radiotimer_getPeriod(void);
uint8_t checkinterrupt(void);
// compare
void     radiotimer_schedule(PORT_RADIOTIMER_WIDTH offset);
void     radiotimer_schedule_us(PORT_RADIOTIMER_WIDTH offset);

void     radiotimer_cancel(void);
// capture
PORT_RADIOTIMER_WIDTH radiotimer_getCapturedTime(void);
uint32_t radiotimer_getfreerunning(void);
// interrupt handlers
kick_scheduler_t   radiotimer_isr(void);

uint32_t getdeltaslotperiod_ms(void);

//CSMA = 1
void RITTimer_Init(void);
/**
\}
\}
*/

#endif
