/*
 * uDMA_SPI.c
 *
 *  Created on: Jun 28, 2021
 *      Author: 16126
 */
#include <stdint.h>
#include <stdbool.h>

#include "driverlib/ssi.h"
#include "driverlib/pin_map.h"
#include "driverlib/udma.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"

#include "inc/hw_ssi.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"

//*****************************************************************************
#if defined(ewarm)
#pragma data_alignment=1024
uint8_t ui8ControlTable[1024];
#elif defined(ccs)
#pragma DATA_ALIGN(ui8ControlTable, 1024)
uint8_t ui8ControlTable[1024];
#else
uint8_t ui8ControlTable[1024] __attribute__ ((aligned(1024)));
#endif
//*****************************************************************************
#define TXBUF_SIZE 10
#define MEM_BUFFER_SIZE         64
uint8_t ui8TxBuf[MEM_BUFFER_SIZE];
#define page 8
#define col 128
//uint8_t LCD_IMAGE_Pri[1024];
//uint8_t LCD_IMAGE_Sec[1024];
uint8_t *LCD_IMAGE_Ptr;
volatile uint8_t currentBuffer;
const uint32_t ui32SrcBuf[1] = { 0x00 };
bool Write_ready;
volatile uint16_t tx_tail, tx_head = 0;

void uDMAIntHandler(void)
{
    uint32_t ui32Mode;
    ui32Mode = uDMAChannelModeGet(UDMA_CHANNEL_SW);
    if (ui32Mode == UDMA_MODE_STOP)
    {
        Write_ready = 1;
    }

}
UART1IntHandler(void)
{
    // ui32Status = MAP_UARTIntStatus(UART1_BASE, 1);
    // MAP_UARTIntClear(UART1_BASE, ui32Status);

}

void Init_SW_DMA(void)
{   // Use to clean image lcd
//IntEnable(INT_UDMA);
    uDMAChannelAttributeDisable(UDMA_CHANNEL_SW, UDMA_ATTR_ALL);

    uDMAChannelControlSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT,
    UDMA_SIZE_32 | UDMA_SRC_INC_NONE | UDMA_DST_INC_32 |
    UDMA_ARB_32);

    uDMAChannelDisable(UDMA_CHANNEL_SW);
    //  uDMAChannelRequest(UDMA_CHANNEL_SW);
}
void Init_SPI_DMA(void)
{

    SSIDMAEnable(SSI0_BASE, SSI_DMA_TX);    // Peripheral level

    uDMAControlBaseSet(ui8ControlTable);
    // Configure channel attribute
    uDMAChannelAttributeDisable(UDMA_CHANNEL_SSI0TX, UDMA_ATTR_ALL);

    // Configure channel control stucture
    uDMAChannelControlSet(      //  Primary Control word
            UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT,
            UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_4);

}
void clearBuffer(void *ptr)
{

    uDMAChannelControlSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT,
    UDMA_SIZE_32 | UDMA_SRC_INC_NONE | UDMA_DST_INC_32 |
    UDMA_ARB_32);
    uDMAChannelTransferSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT, UDMA_MODE_AUTO,
                           (void*) ui32SrcBuf, ptr, 256);
    uDMAChannelEnable(UDMA_CHANNEL_SW);
    uDMAChannelRequest(UDMA_CHANNEL_SW);
}
void WriteImageToDriverLCD(void *bufferPtr)
{

    uDMAChannelControlSet(      //  Primary Control word
            UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT,
            UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_4);
    uDMAChannelTransferSet( UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT,
    UDMA_MODE_BASIC,
                           bufferPtr, (void*) (SSI0_BASE + SSI_O_DR), 8);

    uDMAChannelEnable(UDMA_CHANNEL_SSI0TX);

}
void WriteTxFiFO(uint8_t c)
{
    ui8TxBuf[tx_tail] = c;
    // wait for fifo empty
    while ((tx_tail == tx_head - 1)
            || (tx_head == 0 && (tx_tail == MEM_BUFFER_SIZE - 1)))
        ;

    if (tx_tail == MEM_BUFFER_SIZE - 1)
        tx_tail = 0;
    else
        tx_tail++;
    SSIIntEnable(SSI0_BASE, SSI_TXFF);
}
void ReadTxFiFO(void)
{
    uint32_t status = SSIIntStatus(SSI0_BASE, true);
    SSIIntClear(SSI0_BASE, status);
    uint8_t i = 0;
    for (i = 0; i <= 4; i++)
    {
        if (tx_head != tx_tail)
        {
            SSIDataPut(SSI0_BASE, ui8TxBuf[tx_head]);

            if (tx_head == (MEM_BUFFER_SIZE - 1))
                tx_head = 0;
            else
                tx_head++;
        }
        else
            SSIIntDisable(SSI0_BASE, SSI_TXFF);
        break;
    }

}
