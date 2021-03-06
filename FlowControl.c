/*
 * FlowControl.c
 *
 *  Created on: Jun 26, 2021
 *      Author: 16126
 */

#include "stdint.h"
#include "stdbool.h"

#include "driverlib/pwm.h"
#include "driverlib/interrupt.h"
#include "driverlib/qei.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "inc/hw_types.h"

#include "inc/hw_memmap.h"
#include "inc/hw_qei.h"
#include "TCA9539.h"

#include "PID.h"
#include "TCA9539_hw_memmap.h"
#include "Coffee_Machine.h"

#ifndef Null
#define Null 0
#endif

#define QEIVelTimer10m 800000

extern volatile float MilliLitresBuffer;
float FlowMeter;
uint32_t FlowMeter_Pulse, VrTimeVel;

extern float SetVolume;
extern volatile bool FinishPumpEvent;
extern CNTL_2P2Z_Terminal_t Steam_CNTL;
extern _Bool InPumping;
TCA9539Regs TCA9539_IC3;
extern void Pumping_Process_Stop(void *PrPtr);
extern float Gui_CoffeeExtractionTime;
void FlowMeterCal(void);
extern _Bool InCleanning;
//static float v1, v2;
float Dec = 0.8, Decspeed = 0.4;
_Bool posspeed;
_Bool ErNoPumpPulse;
extern uint32_t wExtract_MaxTime, wExtract_MinTime;
void QEP_CoffeeMachine_cnf(void)
{
    QEIDisable(QEI0_BASE);
    QEIIntDisable(QEI0_BASE,
    QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);
// GPIO configure mux
    GPIOUnlockPin(GPIO_PORTD_BASE, GPIO_PIN_7); // Unlock pin
    GPIOPinConfigure(GPIO_PD6_PHA0);        // Clock
    GPIOPinConfigure(GPIO_PD7_PHB0);        // Dir
// GPIO configure type
    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA, // Mode Pul/Dir
                     GPIO_PIN_TYPE_STD_WPD);
//////////////////////////////////////////////////////////////////////////////
// Confiure QEI
    QEIConfigure(QEI0_BASE,
    QEI_CONFIG_NO_RESET | QEI_CONFIG_CLOCK_DIR | QEI_CONFIG_NO_SWAP, // PD7 Dir, PD6 Clock
                 0xFFFFFFFF);
    HWREG(QEI0_BASE + QEI_O_CTL) = ((HWREG(QEI0_BASE + QEI_O_CTL)
            & ~(QEI_CTL_INVI)) | QEI_CTL_INVI);     // Invert Index Pulse
    QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, QEIVelTimer10m);  // 10ms
    QEIFilterConfigure(QEI0_BASE, QEI_FILTCNT_17);
    QEIFilterEnable(QEI0_BASE);
    QEIVelocityEnable(QEI0_BASE);
    QEIIntRegister(QEI0_BASE, &FlowMeterCal);
    uint32_t status = QEIIntStatus(QEI0_BASE, true);
    QEIIntClear(QEI0_BASE, status);
    QEIEnable(QEI0_BASE);

}
void FlowMeterCal()
{
    if (QEIIntStatus(QEI0_BASE, true) == QEI_INTTIMER)
    {
        QEIIntClear(QEI0_BASE, QEI_INTTIMER);
        MilliLitresBuffer = (float) QEIPositionGet(QEI0_BASE);
        // Calculate the vollume pumping
        if (Gui_CoffeeExtractionTime > wExtract_MaxTime)
        {
            Pumping_Process_Stop(Null);
            return;
        }
        if ((Gui_CoffeeExtractionTime > wExtract_MinTime) || InCleanning)
        {
            if ((MilliLitresBuffer >= SetVolume * Dec) && (posspeed == false))
            {
                uint32_t countDuty = (uint32_t) (Decspeed
                        * PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1));
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, countDuty);
                posspeed = true;

            }
            if ((MilliLitresBuffer >= SetVolume) && (FinishPumpEvent == false))

            {
                // Stop pumping direct not through swept function
                Pumping_Process_Stop(Null);
            }
        }
        if (VrTimeVel >= 100)
        {
            VrTimeVel = 0;
            FlowMeter = FlowMeter_Pulse * 0.25;
            FlowMeter_Pulse = 0;

        }
        else
        {
            FlowMeter_Pulse += QEIVelocityGet(QEI0_BASE);
            VrTimeVel++;
        }
        /*
         if (!InPumping)
         return;

         if (eVrTimer[eVrPumpingPulse] >= 500)
         {   // 2s
         eVrTimer[eVrPumpingPulse] = 0;
         v1 = MilliLitresBuffer;
         // ErNoPumpPulse = (v1 > v2) ? false : true;
         v2 = v1;

         }
         else if (PWMPulseWidthGet(PWM0_BASE, PWM_OUT_2) >= 2000)
         eVrTimer[eVrPumpingPulse]++;*/

    }
}

void InitPumpingEvent(void)

{
    QEIPositionSet(QEI0_BASE, 0x0000);        // Initalize
    MilliLitresBuffer = 0;
    VrTimeVel = 0;
    posspeed = false;
    // v1 = v2 = 0;
    uint32_t status = QEIIntStatus(QEI0_BASE, true);
    QEIIntClear(QEI0_BASE, status);   // Clear any interrupt flag
    QEIIntEnable(QEI0_BASE, QEI_INTTIMER);  // Init interrupt timer out

}
