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
#include "TCA9539.h"
#include "PID.h"

#define Shutdown_Temp_Steam 120
#define Shutdown_Temp_HotWater 95

#define PID_method
extern TCA9539Regs TCA9539_IC1, TCA9539_IC2, TCA9539_IC3;
extern CNTL_2P2Z_Terminal_t Steam_CNTL, HotWater_CNTL;
extern void ADS1118_Cal(ADS1118_t*);
extern float Steam_Vout, HotWater_Vout;
extern void Led_Display();
extern volatile uint8_t In_TxBrust;

extern ADS1118_t Steam, Hot_Water;
ADS1118_t *Temp_ptr;
// True task 125ms
extern uint8_t counttest;
void Temperature_Control(void)
{

    if (TimerIntStatus(TIMER3_BASE, true) == TIMER_TIMA_TIMEOUT)
    {

        static uint16_t scale = 0;
        TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
        counttest = scale;
        if (scale >= 4)  // task 0.5s
        {
            scale = 0;
#ifdef PID_method

            CNTL_2P2Z(&Steam_CNTL);
            CNTL_2P2Z(&HotWater_CNTL);
#endif

        }
        else
        {
            switch (scale)
            {
            case 0:
            case 1:
                Temp_ptr = &Steam;
                break;
            case 2:
            case 3:
                Temp_ptr = &Hot_Water;
                break;

            }
            ADS1118_Cal(Temp_ptr); // request hot data of steam, read cold data of preveous
                                   // request cold data of steam, read hot data of steam
            //  ADS1118_Cal(&Hot_Water);     // request hot data of Hotwater, read cold data of preveous
            // request cold data of Hotwater, read hot data of Hotwater
            scale++;

        }
        // Shutdow if overshoot termperature
/*
        if (Steam.Actual_temperature >= Shutdown_Temp_Steam)
            PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, false);
        else
            PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, true);
        if (Steam.Actual_temperature >= Shutdown_Temp_Steam)
            PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, false);
        else
            PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3_BIT, (uint32_t) Steam_Vout);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4_BIT, (uint32_t) HotWater_Vout);
*/

        TCA9539_IC1.updateOutputFlag = 1;
        TCA9539_IC2.updateOutputFlag = 1;
        TCA9539_IC3.updateOutputFlag = 1;
        // Sub fuction
        // Led_Display();
    }

}

