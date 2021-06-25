/*
 * Temperature_Control.cpp
 *
 *  Created on: Jun 25, 2021
 *      Author: 16126
 */
#include "stdbool.h"
#include "stdint.h"

#include "PID.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "inc/hw_memmap.h"


extern CNTL_2P2Z_Terminal_t Steam_CNTL, HotWater_CNTL;
void Temperature_Control(void)
{
    if (TimerIntStatus(TIMER3_BASE, true) == TIMER_TIMA_TIMEOUT)
    {
        TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
        CNTL_2P2Z(&Steam_CNTL);
        CNTL_2P2Z(&HotWater_CNTL);

    }

}
