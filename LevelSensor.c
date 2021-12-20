/*
 * LevelSensor.c
 *
 *  Created on: Dec 18, 2021
 *      Author: 16126
 */

#include <stdint.h>
#include <stdbool.h>

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/udma.h"
#include "driverlib/adc.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

#include "inc/hw_memmap.h"
#include "inc/hw_adc.h"
#include "inc/hw_ints.h"
enum BUFFER_STATUS
{
    EMPTY, FILLING, FULL,

};
#define ADC_SAMPLE_BUF_SIZE 32
uint16_t ui16ADCBuffer1[ADC_SAMPLE_BUF_SIZE];
uint16_t ui16ADCBuffer2[ADC_SAMPLE_BUF_SIZE];
static enum BUFFER_STATUS ui32BufferStatus[2];
float ui32AverageResult;
float K_Level;

void ADCSeqHander()
{
    ADCIntClear(ADC0_BASE, 0);
    if ((uDMAChannelModeGet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT)
            == UDMA_MODE_STOP) && (ui32BufferStatus[0] == FILLING))
    {
        ui32BufferStatus[0] = FULL;
        ui32BufferStatus[1] = FILLING;

    }
    else if ((uDMAChannelModeGet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT)
            == UDMA_MODE_STOP) && (ui32BufferStatus[1] == FILLING))
    {
        ui32BufferStatus[0] = FILLING;
        ui32BufferStatus[1] = FULL;
    }
}

void ADC_uDMA_Cfg()
{
    ui32BufferStatus[0] = FILLING;
    ui32BufferStatus[1] = EMPTY;
    uDMAEnable();

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
    uDMAChannelAttributeDisable(UDMA_CHANNEL_ADC0,
    UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY |
    UDMA_ATTR_REQMASK);

    uDMAChannelControlSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT, UDMA_SIZE_16 |
    UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);
    uDMAChannelControlSet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT, UDMA_SIZE_16 |
    UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);

    uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT,
    UDMA_MODE_PINGPONG,
                           (void*) (ADC0_BASE + ADC_O_SSFIFO0), &ui16ADCBuffer1,
                           ADC_SAMPLE_BUF_SIZE);
    uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT,
    UDMA_MODE_PINGPONG,
                           (void*) (ADC0_BASE + ADC_O_SSFIFO0), &ui16ADCBuffer2,
                           ADC_SAMPLE_BUF_SIZE);

    uDMAChannelAttributeEnable(UDMA_CHANNEL_ADC0, UDMA_ATTR_USEBURST);
    uDMAChannelEnable(UDMA_CHANNEL_ADC0);

    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_HALF, 1);
    SysCtlDelay(10);

    IntDisable(INT_ADC0SS0);
    ADCIntDisable(ADC0_BASE, 0);
    ADCSequenceDisable(ADC0_BASE, 0);

    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_TIMER, 0);

    ADCSequenceStepConfigure(ADC0_BASE, 0, 0,
    ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);

    // ADCComparatorConfigure(ADC0_BASE, 0, );

    ADCSequenceEnable(ADC0_BASE, 0);
    ADCIntRegister(ADC0_BASE, 0, &ADCSeqHander);

    ADCIntClear(ADC0_BASE, 0);
    ADCSequenceDMAEnable(ADC0_BASE, 0);

    ADCIntEnable(ADC0_BASE, 0);
    IntEnable(INT_ADC0SS0);
    K_Level = (float) 3.3 / 4096;

}
void ADC_uDMA_READ(void)
{
    if (ui32BufferStatus[0] == FULL)
    {
        uint16_t ui16Count;
        uint32_t ui32temp = 0;

        for (ui16Count = 0; ui16Count < ADC_SAMPLE_BUF_SIZE; ui16Count++)
        {
            ui32temp += ui16ADCBuffer1[ui16Count];
            ui16ADCBuffer1[ui16Count] = 0;
        }
        ui32BufferStatus[0] = EMPTY;
        uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT,
        UDMA_MODE_PINGPONG,
                               (void*) (ADC0_BASE + ADC_O_SSFIFO0),
                               &ui16ADCBuffer1, ADC_SAMPLE_BUF_SIZE);

        uDMAChannelEnable(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT);
        ui32AverageResult = (ui32temp * K_Level) / ADC_SAMPLE_BUF_SIZE;
    }
    if (ui32BufferStatus[1] == FULL)
    {
        uint16_t ui16Count;
        uint32_t ui32temp = 0;
        for (ui16Count = 0; ui16Count < ADC_SAMPLE_BUF_SIZE; ui16Count++)
        {
            ui32temp += ui16ADCBuffer2[ui16Count];
            ui16ADCBuffer2[ui16Count] = 0;
        }
        ui32BufferStatus[1] = EMPTY;
        uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT,
        UDMA_MODE_PINGPONG,
                               (void*) (ADC0_BASE + ADC_O_SSFIFO0),
                               &ui16ADCBuffer2, ADC_SAMPLE_BUF_SIZE);
        uDMAChannelEnable(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT);
        ui32AverageResult = (ui32temp * K_Level) / ADC_SAMPLE_BUF_SIZE;
    }

}

