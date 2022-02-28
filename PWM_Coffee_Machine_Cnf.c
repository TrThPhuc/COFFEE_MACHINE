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
#include "Coffee_Machine.h"
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

extern void PulseStepCount(void);
void PWMDRV_Coffee_machine_cnf(void)
{

#ifndef PWM_CLOCK
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

#endif
#ifndef GPIOB_CLOCK
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
#endif
// Configure Pin mux
#if(Grind == pwm)
    GPIOPinConfigure(GPIO_PB6_M0PWM0);  // Grinding motor - Generation 0
#endif

    GPIOPinConfigure(GPIO_PB7_M0PWM1);  // Comress Motor - Generation 0
//------------------------------------------------------------------------------
    GPIOUnlockPin(GPIO_PORTB_BASE, GPIO_PIN_4); // Unclock pin
    GPIOUnlockPin(GPIO_PORTB_BASE, GPIO_PIN_5);

    GPIOPinConfigure(GPIO_PB4_M0PWM2);  // Pumping motor  - Generation 1

// GPIOPinConfigure(GPIO_PB5_M0PWM3);  // Out SSR1 Steam -  Generation 1(Not used PWM module)
//------------------------------------------------------------------------------
// GPIOPinConfigure(GPIO_PE4_M0PWM4);  // Out SSR2 Hot water - Generation 2 (Not used PWM module)
// GPIOPinConfigure(GPIO_PE5_M0PWM5);  // Out SSR3 Compress - Generation 2 (Not used PWM module)

// Configure Pin type
#if(Grind == pwm)
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6); // Grinding motor - Generation 0
#else

    GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_DIR_MODE_OUT);
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_PIN_6);

#endif
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7); // Comress Motor   - Generation 0
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7,
    GPIO_STRENGTH_8MA,
                     GPIO_PIN_TYPE_STD);

    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4); // Pumping motor - Generation 1

// Used for low frequency pwm generation
    GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_DIR_MODE_OUT);
    GPIODirModeSet(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_DIR_MODE_OUT);
    GPIODirModeSet(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_DIR_MODE_OUT);

    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_STRENGTH_8MA,
    GPIO_PIN_TYPE_STD);
    GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5,
    GPIO_STRENGTH_8MA,
                     GPIO_PIN_TYPE_STD);

    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);

//-------------------------Generation 2 - f = 1Khz-----------------------------------
// SSR 2 - SSR3 (Not used - used low pwm timer trigger)

//-------------------------Generation 1 - f = 10Kh-----------------------------------
    //pumping motor & SSR
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1,
    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC | PWM_GEN_MODE_DBG_RUN);

    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 8000);     // Freq = 10Khz
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0); // PB4 - Intialize Duty = 0%    //pumping moto

    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, false);    // Disable output

    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
//-------------------------Generation 0 - f = 10Kh------------------------------------
    // Grinding motor & Comress Motor
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0,
    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 80000);    // Freq = 1Khz
#if(Grind == pwm)
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 0); // PB6 - Intialize Duty = 0%    Grinding motor
#endif
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0); // PB7 - Intialize Duty = 0%    Comress Motor
#if(Grind == pwm)
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, false);    // Disable output
#endif
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, false);    // Disable output
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);

    PWMGenIntRegister(PWM0_BASE, PWM_GEN_0, PulseStepCount);

    //------------------------------------------------------------

#if(Grind == pwm)
    PWMOutputInvert(PWM0_BASE, PWM_OUT_0_BIT, true);
#endif
    PWMOutputInvert(PWM0_BASE, PWM_OUT_1_BIT, true);
    PWMOutputInvert(PWM0_BASE, PWM_OUT_2_BIT, true);

#if(Grindmotor == scooter)
    SysCtlPWMClockSet(SYSCTL_PWMDIV_16);
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
#else
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
#endif
}
