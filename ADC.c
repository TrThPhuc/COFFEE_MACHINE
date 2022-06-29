/*
 * LevelSensor.c
 *
 *  Created on: Dec 18, 2021
 *      Author: 16126
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/udma.h"
#include "driverlib/adc.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/timer.h"

#include "inc/hw_memmap.h"
#include "inc/hw_adc.h"
#include "inc/hw_ints.h"
#define ADC_SAMPLE_BUF_SIZE 2

uint32_t ui32ADCBuffer[8];
//static enum BUFFER_STATUS ui32BufferStatus[2];
float fADC_LevelSensor, fADC_TempWarming;
static float K_Level;
volatile uint8_t range = 0;
float VRT, VR, RT, TX, ln, Group_Temp;
extern float Extrude_Vout;
extern uint32_t wGroupDuty, wGroupShutdownDuty, wGroupTempSet;
#define RT0 10000
#define B 3977
#define T0 298.15
#define HighT 600
#define LowT 700
enum ADC_bufer
{
    levelsensor, temperaturePressurebrew,
};
void ADCSeqHander()
{
    ADCIntClear(ADC0_BASE, 0);
    ADCSequenceDataGet(ADC0_BASE, 0, ui32ADCBuffer);
}

void ADC_Cfg()
{

// GPIO config
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

//-----------------------------------------------------------

    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_HALF, 1);

    SysCtlDelay(10);

    IntDisable(INT_ADC0SS0);
    ADCIntDisable(ADC0_BASE, 0);
    ADCSequenceDisable(ADC0_BASE, 0);
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_TIMER, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0,
    ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 1,
    ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);

    ADCHardwareOversampleConfigure(ADC0_BASE, 64);  // Oversample 64 times

    ADCIntRegister(ADC0_BASE, 0, &ADCSeqHander);

    ADCIntClear(ADC0_BASE, 0);

    ADCIntEnable(ADC0_BASE, 0);
    ADCSequenceEnable(ADC0_BASE, 0);
    IntEnable(INT_ADC0SS0);
    TimerControlTrigger(TIMER4_BASE, TIMER_A, true);
    TimerControlStall(TIMER4_BASE, TIMER_A, true);
    K_Level = (float) 3.3 / 4096;

}
void ADC_READ(void)
{
    fADC_TempWarming = ui32ADCBuffer[temperaturePressurebrew] * K_Level; // ex temp
    // fADC_LevelSensor = (ui32ADCBuffer[levelsensor] * K_Level  - 0.6) * 10.16;   //10 // used for pressure sensor
    fADC_LevelSensor = ui32ADCBuffer[levelsensor] * K_Level;     // level sensor

}
void CNTL_Extrude()
{
    VRT = fADC_TempWarming;      //Conversion to voltage
    if (VRT > 0)
    {
        VR = (float) 5 - VRT;
        RT = VR / (VRT / 1000);               //Resistance of RT
        ln = log(RT / RT0);
        TX = (1 / ((ln / B) + (1 / T0))); //Temperature from thermistor
        Group_Temp = TX - 273.15;
    }
    if (Group_Temp > wGroupTempSet)
        Extrude_Vout = wGroupShutdownDuty;
    else
        Extrude_Vout = wGroupDuty;
}
