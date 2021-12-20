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
#include "driverlib/gpio.h"
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
_Bool PWMSSR1Enable, PWMSSR2Enable, PWMSSR3Enable;

ADS1118_t *Temp_ptr;
// True task 125ms
extern uint16_t counttest;
uint16_t dutyCount_SSR1 = 0, dutyCount_SSR2 = 0, dutyCount_SSR3 = 0;
uint16_t activeDuty_SRR1 = 0, activeDuty_SRR2 = 0, activeDuty_SRR3 = 0;
void LowFreqPWM(void);

void Temperature_Control(void)
{

    if (TimerIntStatus(TIMER3_BASE, true) == TIMER_TIMA_TIMEOUT)
    {

        static uint16_t scale = 0;
        TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
        counttest = scale;
        if (scale >= 4)  // Sacle to task 0.5s
        {
            scale = 0;
#ifdef PID_method
            if (PWMSSR1Enable)
                CNTL_2P2Z(&Steam_CNTL);
            // CNTL_2P2Z(&HotWater_CNTL);
#endif

        }
        else
        {
            switch (scale)
            {
            case 0:     // request hot data of steam, read cold data of preveous
            case 1:
                Temp_ptr = &Steam; // request cold data of steam, read hot data of steam
                break;
            case 2:     // request hot data of steam, read cold data of preveous
            case 3:
                Temp_ptr = &Hot_Water;
                break;

            }
            ADS1118_Cal(Temp_ptr);

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

        TCA9539_IC1.ReadCmdFlag = 1;
        TCA9539_IC2.ReadCmdFlag = 2;
        TCA9539_IC3.ReadCmdFlag = 1;

    }

}
void LowFreqPWM(void)
{
    TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
    //--------------------SSR1---------------------------------------
    if (PWMSSR1Enable)
    {

        if (dutyCount_SSR1 == activeDuty_SRR1)
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0);
        if (dutyCount_SSR1 < 100)
            dutyCount_SSR1++;
        else
        {
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_PIN_5);
            activeDuty_SRR1 = 100 - Steam_Vout;   //Shadow Update to active
            dutyCount_SSR1 = 0;
        }

    }
    else
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_PIN_5);

    //--------------------SSR2---------------------------------------
    if (PWMSSR2Enable)
    {
        if (dutyCount_SSR2 == activeDuty_SRR2)
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0); // turn on ssr out
        if (dutyCount_SSR2 < 100)
        {
            dutyCount_SSR2++;

        }
        else
        {
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4); // turn off ssr out
            activeDuty_SRR2 = 100 - HotWater_Vout; //Shadow Update to active when counter match period
            dutyCount_SSR2 = 0;
        }

    }
    else
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4); // turn off ssr out

    //--------------------SSR3---------------------------------------
    /*    if (PWMSSR3Enable)
     {
     if (dutyCount_SSR3 < (uint32_t) Steam_CNTL.Out)
     GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5);
     else
     GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0);
     }*/

}
