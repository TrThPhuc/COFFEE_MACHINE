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
#include "DCL_PID.h"

#define Shutdown_Temp_Steam 125.0f
#define Shutdown_Temp_HotWater 125.0f

#define PID_method
extern TCA9539Regs TCA9539_IC1, TCA9539_IC2, TCA9539_IC3;
extern CNTL_2P2Z_Terminal_t Steam_CNTL, HotWater_CNTL;
extern CNTL_PID_DCL_Terminal_t HotWater_PID_DCL;
extern void ADS1118_Cal(ADS1118_t*);
extern float Steam_Vout, HotWater_Vout, Extrude_Vout;
extern void Led_Display();
extern volatile uint8_t In_TxBrust;

extern ADS1118_t Steam, Hot_Water;
_Bool PWMSSR1Enable, PWMSSR2Enable, PWMSSR3Enable;
_Bool SteamMask, HotWaterMask, ExtrudeMask;
_Bool ErSteam, ErHotWater, ErExtrude;
extern volatile _Bool Hyteresis, ErPumpSteam;
volatile _Bool HeatingSteam, HeatingHotwater;

ADS1118_t *Temp_ptr;
// True task 125ms
extern uint16_t counttest;
float SteamTempBuffer[4], HotWaterTempBuffer[4];
float tempvalue;
extern float Gui_TempSteam, Gui_HotWaterSteam;

uint16_t dutyCount_SSR1 = 0, dutyCount_SSR2 = 0, dutyCount_SSR3 = 0;
uint16_t activeDuty_SRR1 = 0, activeDuty_SRR2 = 0, activeDuty_SRR3 = 0;
extern void CNTL_Extrude();
void LowFreqPWM(void);
void Temperature_Control(void)
{

    if (TimerIntStatus(TIMER3_BASE, true) == TIMER_TIMA_TIMEOUT)    // 0.125s
    {

        static uint16_t scale = 0;
        TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);

        if (scale >= 4)  // Sacle to task 0.5s
        {
            scale = 0;
            counttest++;
            if (counttest >= 4)
                counttest = 0;
#ifdef PID_method
            if (PWMSSR1Enable)
                CNTL_2P2Z(&Steam_CNTL);
            if (PWMSSR2Enable)
                CNTL_PID_DCL(&HotWater_PID_DCL);
            if (PWMSSR3Enable)
                CNTL_Extrude();
#endif
            SteamTempBuffer[counttest] = Steam.Actual_temperature;
            tempvalue = 0;
            int i;
            for (i = 0; i < 4; i++)
                tempvalue += SteamTempBuffer[i];
            Gui_TempSteam = tempvalue / 4;

            HotWaterTempBuffer[counttest] = Hot_Water.Actual_temperature;
            tempvalue = 0;
            for (i = 0; i < 4; i++)
                tempvalue += HotWaterTempBuffer[i];
            Gui_HotWaterSteam = tempvalue / 4;
        }
        else
        {
            switch (scale)
            {
            case 0: // request hot data of steam, read cold data of preveous    //0.25
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

        ErHotWater = (Hot_Water.Actual_temperature == 0xA5A5) ? true : false;
        if (Hot_Water.Actual_temperature >= Shutdown_Temp_HotWater
                || ErHotWater)
        {
            HotWaterMask = 1;   // Mask Hot water control
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);  // turn off

        }
        else
            HotWaterMask = 0;
        ErSteam = (Steam.Actual_temperature == 0xA5A5) ? true : false;
        if (Steam.Actual_temperature >= Shutdown_Temp_Steam || ErSteam)
        {
            SteamMask = 1;     // MasK steam water control
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_PIN_5);

        }
        else
            SteamMask = 0;

        TCA9539_IC1.updateOutputFlag = 1;
        TCA9539_IC2.updateOutputFlag = 1;
        TCA9539_IC3.updateOutputFlag = 1;

        TCA9539_IC1.ReadCmdFlag = 1;
        TCA9539_IC2.ReadCmdFlag = 1;
        TCA9539_IC3.ReadCmdFlag = 1;

    }

}

void LowFreqPWM(void)
{
    uint32_t tempduty;
    TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
//---------------------------------------------SSR1 - Steam -----------------------------------------
    bool LevelWaterEnough = Hyteresis || ErPumpSteam;
    if (PWMSSR1Enable && !SteamMask && !LevelWaterEnough)
    {
        HeatingSteam = true;
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
    {
        HeatingSteam = false;
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_PIN_5);
    }

//-----------------------------------------------SSR2 - Hotwater -------------------------------------
    if (PWMSSR2Enable && !HotWaterMask)
    {
        HeatingHotwater = true;
        if (dutyCount_SSR2 == activeDuty_SRR2)
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0); // turn on ssr out
        if (dutyCount_SSR2 < 100)
        {
            dutyCount_SSR2++;

        }
        else
        {
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4); // turn off ssr out
            if (HotWater_Vout < 0)
                tempduty = 0;
            else
                tempduty = (uint16_t) HotWater_Vout;
            activeDuty_SRR2 = 100 - tempduty; //Shadow Update to active when counter match period
            dutyCount_SSR2 = 0;
        }

    }
    else
    {
        HeatingHotwater = false;
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4); // turn off ssr out
    }

//------------------------------------------SSR3 - Heating compress ------------------------------------
    if (PWMSSR3Enable && !ExtrudeMask)
    {
        if (dutyCount_SSR3 == activeDuty_SRR3)
        {
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0); // turn on ssr out
        }

        if (dutyCount_SSR3 < 400)
        {
            dutyCount_SSR3++;

        }
        else
        {
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5); // turn off ssr out
            activeDuty_SRR3 = 400 - Extrude_Vout * 4; //Shadow Update to active when counter match period
            dutyCount_SSR3 = 0;
        }

    }
    else
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5); // turn off ssr out

}
