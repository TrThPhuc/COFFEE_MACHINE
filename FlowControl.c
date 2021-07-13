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

#ifndef Null
#define Null 0
#endif

extern uint32_t totalMilliLitres, MilliLitresBuffer;
extern uint32_t SetVolume;
extern volatile bool FinishPumpEvent;
extern float Calibration;

extern void Pumping_Process_Stop(void *PrPtr);
void FlowMeterCal(void);
void QEP_CoffeeMachine_cnf(void)
{
    QEIDisable(QEI0_BASE);
    QEIIntDisable(QEI0_BASE,
    QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);
// GPIO configure mux
    GPIOUnlockPin(GPIO_PORTD_BASE, GPIO_PIN_7); // Unlock pin
    GPIOPinConfigure(GPIO_PD6_PHA0);
    GPIOPinConfigure(GPIO_PD7_PHB0);
// GPIO configure type
    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA, // Mode Pul/Dir
                     GPIO_PIN_TYPE_STD_WPD);
// Confiure QEI
    QEIConfigure(QEI0_BASE,
    QEI_CONFIG_NO_RESET | QEI_CONFIG_CLOCK_DIR | QEI_CONFIG_NO_SWAP,
                 0xFFFFFFFF);
    HWREG(QEI0_BASE + QEI_O_CTL) = ((HWREG(QEI0_BASE + QEI_O_CTL)
            & ~(QEI_CTL_INVI)) | QEI_CTL_INVI);
    QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_16, 500000);
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
        uint32_t temp;
        temp = QEIPositionGet(QEI0_BASE);
        // MilliLitresBuffer = (float) temp * Calibration;
        if ((MilliLitresBuffer >= SetVolume) && (FinishPumpEvent == false))
        {
            // Stop pumping(direct call for accuracy)
            Pumping_Process_Stop(Null);
        }
    }
}

void InitPumpingEvent(void)

{
    MilliLitresBuffer = 0;
    QEIPositionSet(QEI0_BASE, 0x0000);
    ;
    QEIIntClear(QEI0_BASE, QEI_INTTIMER);
    QEIIntEnable(QEI0_BASE, QEI_INTTIMER);  // Init interrupt timer out

}
