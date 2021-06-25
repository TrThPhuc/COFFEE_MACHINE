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
#define mSec50  4000000
#define Sec0_25 20000000

#define Configure_TCA9539_IC1   0xFD30
#define Configure_TCA9539_IC2   0x89A0
#define Configure_TCA9539_IC3   0x7C20
extern void Temperature_Control(void);
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

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

}
void TimerSysClt(void)
{
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
    TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);

    TimerLoadSet(TIMER0_BASE, TIMER_A, mSec2);
    TimerLoadSet(TIMER1_BASE, TIMER_A, mSec5);
    TimerLoadSet(TIMER2_BASE, TIMER_A, mSec50);
    TimerLoadSet(TIMER3_BASE, TIMER_A, Sec0_25);

// Interrrupt timer configure
    TimerIntRegister(TIMER3_BASE, TIMER_A, &Temperature_Control);
    TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    IntMasterEnable();
}
void MachineGpioConfigure(void)
{

}
