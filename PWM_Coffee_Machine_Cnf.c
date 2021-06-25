/*
 * ADS1118.c
 *
 *  Created on: Jun 21, 2021
 *      Author: 16126
 */

/******************************************************************************
 * function: Configure PWMs for coffe machine
 * introduction: read the ADC result and tart a new conversion.
 * parameters:
 ******************************************************************************/
#include "stdbool.h"
#include "stdint.h"

#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"
#include "inc/hw_pwm.h"
//                      SyncIn = 80Mhz
//                         |
//              ___________|___________
//              |                     |
//              |~~~~~~~~~~~~~~~~~~~~~|
//              |                     |
//           -->|                     |
//           -->|              M0PWM0 |-->  Motor cum xay
//              |              M0PWM1 |-->  Motor cum ep
//              |              M0PWM2 |-->  Motor bom nuoc
//              |_____________________|
void PWMDRV_Coffee_machine_cnf(void)
{

#ifndef PWM_CLOK
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

#endif
#ifndef GPIOB_CLOK
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
#endif

    GPIOPinConfigure(GPIO_PB6_M0PWM0);  // Grinding motor - Generation 0
    GPIOPinConfigure(GPIO_PB7_M0PWM1);  // Pressing Motor - Generation 0
    GPIOPinConfigure(GPIO_PB4_M0PWM2);  // Pumping motor  - Generation 1

    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);
//-------------------------Generation 1-----------------------------------
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1,
    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 8000);    // Freq = 10Khz
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);      // PB4 - Intialize Duty = 0%

    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, false);
    PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, false);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
//-------------------------Generation 1-----------------------------------
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0,
    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 8000);    // Freq = 10Khz
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 0);      // PB6 - Intialize Duty = 0%
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);      // PB6 - Intialize Duty = 0%

    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, false);
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, false);

    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
}
