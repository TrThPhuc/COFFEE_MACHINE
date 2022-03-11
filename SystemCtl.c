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
#include "driverlib/rom_map.h"
#include "driverlib/rom.h"
#define mSec2   160000
#define mSec5   400000
#define mSec10  800000
#define mSec20  1600000
#define mSec50  4000000
#define mSec125 10000000
#define mSec500 16000000

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
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);

    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);

    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);

    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);

    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);

}
void TimerSysClt(void)
{
    MAP_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    MAP_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    MAP_TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
    MAP_TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);
    MAP_TimerConfigure(TIMER4_BASE, TIMER_CFG_PERIODIC);
    MAP_TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC);
    MAP_TimerConfigure(WTIMER0_BASE,
    TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC);

    MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, mSec2);
    MAP_TimerLoadSet(TIMER1_BASE, TIMER_A, mSec5);
    MAP_TimerLoadSet(TIMER2_BASE, TIMER_A, mSec50);
    MAP_TimerLoadSet(TIMER3_BASE, TIMER_A, mSec125);   // Temperature control
    MAP_TimerLoadSet(TIMER4_BASE, TIMER_A, mSec20);    // Timming pocess
    MAP_TimerLoadSet(TIMER5_BASE, TIMER_A, mSec5);  // PWM low freqency generate
    MAP_TimerLoadSet(WTIMER0_BASE, TIMER_A, mSec500);

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
