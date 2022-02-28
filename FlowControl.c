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

#ifndef Null
#define Null 0
#endif

#define QEIVelTimer10m 800000

extern volatile float MilliLitresBuffer;
extern float SetVolume;
extern volatile bool FinishPumpEvent;
extern float Calibration;
extern CNTL_2P2Z_Terminal_t Steam_CNTL;
TCA9539Regs TCA9539_IC3;
extern void Pumping_Process_Stop(void *PrPtr);
void FlowMeterCal(void);
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
    QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, QEIVelTimer10m);
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
        ; // Calculate the vollume pumping
        if ((MilliLitresBuffer >= SetVolume) && (FinishPumpEvent == false))

        {
            // Stop pumping direct not through swept function
            Pumping_Process_Stop(Null);
        }

    }
}

void InitPumpingEvent(void)

{
    QEIPositionSet(QEI0_BASE, 0x0000);        // Initalize
    MilliLitresBuffer = 0;
    uint32_t status = QEIIntStatus(QEI0_BASE, true);
    QEIIntClear(QEI0_BASE, status);   // Clear any interrupt flag
    QEIIntEnable(QEI0_BASE, QEI_INTTIMER);  // Init interrupt timer out

}
