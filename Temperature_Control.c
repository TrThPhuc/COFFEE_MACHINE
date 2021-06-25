/*
 * Temperature_Control.c
 *
 *  Created on: Jun 25, 2021
 *      Author: 16126
 */
#include "stdbool.h"
#include "stdint.h"

#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"
#include "inc/hw_memmap.h"
/******************************************************************************/
#include "ADS1118.h"
#include "PID.h"
#define Shutdown_Temp_Steam 120
#define Shutdown_Temp_HotWater 95

#define PID_method

extern CNTL_2P2Z_Terminal_t Steam_CNTL, HotWater_CNTL;
extern void ADS1118_Cal(ADS1118_t*);

extern ADS1118_t Steam, Hot_Water;
// True task 125ms
void Temperature_Control(void)
{
    if (TimerIntStatus(TIMER3_BASE, true) == TIMER_TIMA_TIMEOUT)
    {
        static uint16_t scale = 0;
        TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);

        if (scale > 4)  // task 0.5s
        {
            scale = 0;
#ifdef PID_method

            CNTL_2P2Z(&Steam_CNTL);
            CNTL_2P2Z(&HotWater_CNTL);
#endif
        }
        else
        {
            ADS1118_Cal(&Steam);
            ADS1118_Cal(&Hot_Water);

            scale++;

        }
        // Shutdow if overshoot termperature
        if (Steam.Actual_temperature >= Shutdown_Temp_Steam)
            PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, false);
        if (Steam.Actual_temperature >= Shutdown_Temp_Steam)
            PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, false);

    }

}
