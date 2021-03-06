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
#include "driverlib/gpio.h"

#include "inc/hw_ssi.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"

#include "ADS1118.h"

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
//#define uDma_SSI0
extern uint16_t volatile Settingconfig;
extern ADS1118_t Steam, Hot_Water;
#define TXBUF_SIZE 10
#define MEM_BUFFER_SIZE         1024

uint8_t ui8TxBuf[MEM_BUFFER_SIZE];

uint16_t ui16TxBuf[2];
uint16_t ui16RxBuf[2];
uint8_t *currentBuffer;
extern uint32_t datas, fb_config;

uint16_t SteamBuf[MEM_BUFFER_SIZE];
uint16_t HotWaterBuf[MEM_BUFFER_SIZE];
volatile uint8_t pagelcd = 0;
#define col 128
extern uint8_t LCD_IMAGE[1024];

#ifdef uDma_SSI0
uint8_t LCD_IMAGE_Pri[1024];
uint8_t LCD_IMAGE_Sec[1024];
uint8_t *LCD_IMAGE_Ptr;
#endif
//volatile uint8_t currentBuffer;
uint32_t ui32SrcBuf[1] = { 0x00 };
volatile bool Write_ready;
volatile uint16_t tx_tail, tx_head = 0;
extern void LCD_Address_Set(uint8_t page, uint8_t column);
extern void LCD_Write_Cmd(uint8_t cmd);
////////////////////////////////////////////////////////////////////
#ifdef uDma_SSI0
void uDMAIntHandler(void)
{
    uint32_t ui32Mode;
    //uDMAIntClear();
    ui32Mode = uDMAChannelModeGet(UDMA_CHANNEL_SW);
    if (ui32Mode == UDMA_MODE_STOP)
    {
        Write_ready = 1;
    }

}

void Init_SW_DMA(void)
{   // Use to clean image lcd
    IntEnable(INT_UDMA);
    uDMAIntRegister(INT_UDMA, uDMAIntHandler);

    uDMAChannelAttributeDisable(
            UDMA_CHANNEL_SW,
            UDMA_ATTR_USEBURST | UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY
                    | UDMA_ATTR_REQMASK);

    uDMAChannelControlSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT,
    UDMA_SIZE_32 | UDMA_SRC_INC_NONE | UDMA_DST_INC_32 |
    UDMA_ARB_256);

}
void Init_LCDSPI_DMA(void)
{
    SSIDMAEnable(SSI0_BASE, SSI_DMA_TX);
    uDMAControlBaseSet(ui8ControlTable);
    uDMAChannelAttributeDisable(
            UDMA_CHANNEL_SSI0TX,
            UDMA_ATTR_USEBURST | UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY
                    | UDMA_ATTR_REQMASK);

    // Configure channel control structure

    // Transmit
    uDMAChannelControlSet(      //  Primary Control word
            UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT,
            UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_4);

}
void WriteImageToDriverLCD(void *bufferPtr)
{
    if (pagelcd == 0)
    {
        currentBuffer = bufferPtr;
        uDMAChannelTransferSet( UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT,
        UDMA_MODE_BASIC,
                               currentBuffer, (void*) (SSI0_BASE + SSI_O_DR),
                               128);

        uDMAChannelEnable(UDMA_CHANNEL_SSI0TX);
    }

}
#endif
////////////////////////////////////////////////////////////////////
void clearBuffer(void *ptr)
{
    Write_ready = 0;
    uDMAChannelControlSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT,
    UDMA_SIZE_32 | UDMA_SRC_INC_NONE | UDMA_DST_INC_32 |
    UDMA_ARB_32);
    uDMAChannelTransferSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT, UDMA_MODE_AUTO,
                           (void*) ui32SrcBuf, ptr, 256);
    uDMAChannelEnable(UDMA_CHANNEL_SW);
    uDMAChannelRequest(UDMA_CHANNEL_SW);
}
void Copy_bitExImage(void *ptr)
{

    uDMAChannelControlSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT,
    UDMA_SIZE_32 | UDMA_SRC_INC_32 | UDMA_DST_INC_32 |
    UDMA_ARB_32);
    uDMAChannelTransferSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT, UDMA_MODE_AUTO,
                           (void*) ptr, LCD_IMAGE, 256);
    uDMAChannelEnable(UDMA_CHANNEL_SW);
    uDMAChannelRequest(UDMA_CHANNEL_SW);

}
//============================================================================
// DMA: DMA SPI1
// Interface with: ADS11178
//============================================================================
void Init_SPI_DMA(void)
{

    SSIDMAEnable(SSI1_BASE, SSI_DMA_TX);    // Peripheral level
    SSIDMAEnable(SSI1_BASE, SSI_DMA_RX);
    uDMAControlBaseSet(ui8ControlTable);
    // Assign control table for dma
    // Configure channel attribute
    // uDMAChannelAttributeEnable(UDMA_CHANNEL_SSI1TX, UDMA_ATTR_REQMASK); // Disable dma- mask request
    // uDMAChannelAttributeEnable(UDMA_CHANNEL_SSI1RX, UDMA_ATTR_REQMASK); // Disable dma- mask reques

    uDMAChannelAttributeDisable(
            UDMA_CHANNEL_SSI1TX,
            UDMA_ATTR_USEBURST | UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY);
    uDMAChannelAttributeDisable(
            UDMA_CHANNEL_SSI1RX,
            UDMA_ATTR_USEBURST | UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY);

    // Configure channel control structure

    // Transmit
    uDMAChannelControlSet(      //  Primary Control word
            UDMA_CHANNEL_SSI1TX | UDMA_PRI_SELECT,
            UDMA_SIZE_16 | UDMA_SRC_INC_16 | UDMA_DST_INC_NONE | UDMA_ARB_2);
    // Receive
    uDMAChannelControlSet(
            UDMA_CHANNEL_SSI1RX | UDMA_PRI_SELECT,
            UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_2);

    // Transmit
    uDMAChannelTransferSet( UDMA_CHANNEL_SSI1TX | UDMA_PRI_SELECT,
    UDMA_MODE_BASIC,
                           ui16TxBuf, (void*) (SSI1_BASE + SSI_O_DR), 2);

    // Receive
    uDMAChannelTransferSet( UDMA_CHANNEL_SSI1RX | UDMA_PRI_SELECT,
    UDMA_MODE_BASIC,
                           (void*) (SSI1_BASE + SSI_O_DR), ui16RxBuf, 2);

}
void SSI1_IntHandler(void)
{
    uint32_t ui32Status;
    //static uint16_t config;
    ui32Status = SSIIntStatus(SSI1_BASE, true);
    SSIIntClear(SSI1_BASE, ui32Status);

    if (!uDMAChannelIsEnabled(UDMA_CHANNEL_SSI1TX))
    {
        uDMAChannelTransferSet( UDMA_CHANNEL_SSI1TX | UDMA_PRI_SELECT,
        UDMA_MODE_BASIC,
                               ui16TxBuf, (void*) (SSI1_BASE + SSI_O_DR), 2);
        // config = ((ui16TxBuf[1] & 0x7FFF) | 0x01);

    }
    if (!uDMAChannelIsEnabled(UDMA_CHANNEL_SSI1RX))
    {
        uDMAChannelTransferSet( UDMA_CHANNEL_SSI1RX | UDMA_PRI_SELECT,
        UDMA_MODE_BASIC,
                               (void*) (SSI1_BASE + SSI_O_DR), ui16RxBuf, 2);

        datas = ui16RxBuf[0];
        fb_config = ui16RxBuf[1];
        switch (fb_config)
        {
        case 0xB3B:
            Steam.hot_data = datas;
            eCom_Ads1118 = 0;
            break;
        case 0xB2B:
            Steam.cold_data = datas;
            eCom_Ads1118 = 0;
            break;
        case 0x3B3B:
            Hot_Water.hot_data = datas;
            eCom_Ads1118 = 0;
            break;
        case 0x3B2B:
            Hot_Water.cold_data = datas;
            eCom_Ads1118 = 0;
            break;
        default:
            eCom_Ads1118 = 1;
            break;

        }

    }
}
//============================================================================
void WriteConfigture()
{
    uDMAChannelControlSet(      //  Primary Control word
            UDMA_CHANNEL_SSI1TX | UDMA_PRI_SELECT,
            UDMA_SIZE_16 | UDMA_SRC_INC_16 | UDMA_DST_INC_NONE | UDMA_ARB_2);
    uDMAChannelControlSet(      //  Primary Control word
            UDMA_CHANNEL_SSI1TX | UDMA_ALT_SELECT,
            UDMA_SIZE_16 | UDMA_SRC_INC_16 | UDMA_DST_INC_NONE | UDMA_ARB_2);

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
#ifndef uDma_SSI0
    uint8_t i = 0;
    if (status == SSI_TXFF)
    {
        for (i = 0; i <= 4; i++)
        {
            if (tx_head != tx_tail)
            {
                uint32_t ui32Data = ui8TxBuf[tx_head];
                SSIDataPut(SSI0_BASE, ui32Data);

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
#endif
#ifdef uDma_SSI0
    if (!uDMAChannelIsEnabled(UDMA_CHANNEL_SSI0TX))

    {
        if (pagelcd < 7)
        {
            pagelcd++;
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);
            while (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7) != 0)
                ;
            LCD_Write_Cmd(0xb0 | pagelcd);
            LCD_Write_Cmd(0x10);
            LCD_Write_Cmd(0X00);

            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);
            while (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7) == 0)
                ;
            uDMAChannelTransferSet( UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT,
            UDMA_MODE_BASIC,
                                   (void*) (currentBuffer + (128 * pagelcd)),
                                   (void*) (SSI0_BASE + SSI_O_DR), 128);
            uDMAChannelEnable(UDMA_CHANNEL_SSI0TX);

        }
        else
        {
            pagelcd = 0;
            uDMAChannelDisable(UDMA_CHANNEL_SSI0TX);

        }

    }
#endif
}
