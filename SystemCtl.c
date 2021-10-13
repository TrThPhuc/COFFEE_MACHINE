/*
 * SystemCtl.c
 *
 *  Created on: Jun 23, 2021
 *      Author: 16126
 */
#include "stdbool.h"
#include "stdint.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "inc/hw_memmap.h"
#define mSec2   160000
#define mSec5   400000
#define mSec10  800000
#define mSec50  4000000
#define mSec125 10000000

#define Configure_TCA9539_IC1   0xFF30
#define Configure_TCA9539_IC2   0x89A0
#define Configure_TCA9539_IC3   0x7C20
extern void Temperature_Control(void);
extern void TimmingPorcess(void);
extern void LowFreqPWM(void);
void InitSysClt(void)
{
// Configurate lock system run at 80 Mhz
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_OSC_INT | SYSCTL_USE_PLL);
// Pheripheral enable clock

    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);

}
void TimerSysClt(void)
{
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
    TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);
    TimerConfigure(TIMER4_BASE, TIMER_CFG_PERIODIC);
    TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC);

    TimerLoadSet(TIMER0_BASE, TIMER_A, mSec2);
    TimerLoadSet(TIMER1_BASE, TIMER_A, mSec5);
    TimerLoadSet(TIMER2_BASE, TIMER_A, mSec50);
    TimerLoadSet(TIMER3_BASE, TIMER_A, mSec125);
    TimerLoadSet(TIMER4_BASE, TIMER_A, mSec125);
    TimerLoadSet(TIMER5_BASE, TIMER_A, mSec5);

// Interrrupt timer configure
    TimerIntRegister(TIMER3_BASE, TIMER_A, &Temperature_Control); // Temperature control
    TimerIntRegister(TIMER4_BASE, TIMER_A, &TimmingPorcess); // Interrupt handler for making coffee
    TimerIntRegister(TIMER5_BASE, TIMER_A, &LowFreqPWM); // PWM low freqency generate

    TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);

    // TimerInt of Timmingprocess will enable when initalizing task which need to timming

}
void MachineGpioConfigure(void)
{

}
