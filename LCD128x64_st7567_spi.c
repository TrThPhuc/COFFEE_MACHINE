/*
 * LCD128x64_st7567_spi.c
 *
 *  Created on: Mar 28, 2022
 *      Author: 16126
 */
#include "Coffee_Machine.h"
#include "TCA9539_hw_memmap.h"
#include "TCA9539.h"

#define SET_RST     GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6)
#define CLR_RST     GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0)
// A0/RS for Cmd or data - PA7
#define  SET_RS     GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7)
#define  CLR_RS     GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0)

// LCD back light
#define   LIGHT_ON   TCA9539_IC2.TCA9539_Onput.all = ((TCA9539_IC2.TCA9539_Onput.all\
                     & ~(LCD_back_Light)) | LCD_back_Light);
#define   LIGHT_OFF   TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all\
                     & ~(LCD_back_Light));

extern const unsigned char ascii_table_5x8[95][5]; // Bit Map for ASCII table (ASCII_Font.c)
extern const unsigned char ascii_table_8x16[95][16];
extern void ReadTxFiFO(void);
extern void WriteTxFiFO(uint8_t c);

// Low Level driver
void LCD_Write_Cmd(uint8_t cmd);    // Write command to LCD
void LCD_Write_Dat(uint8_t dat);    // Write data to LCD
void LCD_Address_Set(uint8_t page, uint8_t column); // Set cursor
static void LCD_Disp_Clr(uint8_t dat);
// Display ASCII string
void LCD_Disp_Clr(uint8_t dat);     // Clear LCD

extern uint8_t LCD_IMAGE[1024];
extern TCA9539Regs TCA9539_IC2;
void Spi0_LCD_Interface_Cnf()
{
// Configurate pin mux for SSI(SPI) peripheral function
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    SSIClockSourceSet(SSI0_BASE, SSI_CLOCK_SYSTEM);
// Confige Mode 0 SSI, Freq = 15Mhz, 8Bit
    SSIConfigSetExpClk(SSI0_BASE, 80000000, SSI_FRF_MOTO_MODE_0,
    SSI_MODE_MASTER,
                       15000000, 8);
#ifdef Int_SSI0
    SSIIntRegister(SSI0_BASE, &ReadTxFiFO);
    SSIIntEnable(SSI0_BASE, SSI_TXFF);
#endif
#ifdef uDma_SSI0
    SSIIntRegister(SSI0_BASE, &ReadTxFiFO);
    SSIIntEnable(SSI0_BASE, SSI_DMATX);
    IntEnable(INT_SSI0);
    Init_LCDSPI_DMA();
    Init_SW_DMA();
#endif

}
void LCD_Interface_Cnf()
{
    Spi0_LCD_Interface_Cnf();   // Used interrupt method for transmit
// Configurate Pin for proper ssi
    GPIOPinTypeSSI(GPIO_PORTA_BASE,
    GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
    SSIEnable(SSI0_BASE);

// Configurate Pin for other
    // Pin RST vs Pin A0/Rs
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_STRENGTH_12MA, //PA6 - LCD_RST
                     GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_STRENGTH_12MA, // PA7 - LCD_RS-A0
                     GPIO_PIN_TYPE_STD_WPU);

    GPIODirModeSet(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_DIR_MODE_OUT);
    GPIODirModeSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_DIR_MODE_OUT);

    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);

}
void LCD_Write_Cmd(uint8_t cmd)
{
    CLR_RS;
#if (defined Polling_SSI0) || (defined uDma_SSI0)
    SSIDataPut(SSI0_BASE, (uint32_t) cmd);
    while (SSIBusy(SSI0_BASE))  // Polling method
    {
    }
#endif
#if (defined Int_SSI0)
    SSIIntEnable(SSI0_BASE, SSI_TXFF);
    WriteTxFiFO(cmd);
#endif

}
void LCD_Write_Dat(uint8_t cmd)
{

#ifdef Polling_SSI0
    SET_RS;
    SSIDataPut(SSI0_BASE, (uint32_t) cmd);
    while (SSIBusy(SSI0_BASE))  // polling method
    {
    }
#endif
#ifdef uDma_SSI0
    if ((!uDMAChannelIsEnabled(UDMA_CHANNEL_SSI0TX)) && (pagelcd == 0)){
   // LCD_Address_Set(1, 1);
    SET_RS;
    WriteImageToDriverLCD(LCD_IMAGE_Send);
    }
#endif
#ifdef Int_SSI0
    SET_RS;
    SSIIntEnable(SSI0_BASE, SSI_TXFF);
    WriteTxFiFO(cmd);
#endif
}
void LCD_Address_Set(uint8_t page, uint8_t column)
{
    column = (column - 1) & 0x00FF;
    page = (page - 1) & 0x00FF;
    LCD_Write_Cmd(0xb0 + page);
    LCD_Write_Cmd(((column >> 4) & 0x0f) + 0x10);
    LCD_Write_Cmd(column & 0x0f);
}
void Disp_Str_5x8_Image(volatile uint8_t page, volatile uint8_t column,
                        uint8_t *text, uint8_t *Image)
{
    uint8_t i = 0, j, k;
    page = page - 1;
    column = column - 1;
    while (text[i] > 0x00)
    {

        if ((text[i] >= 0x20) && (text[i] <= 0x7e))
        {
            j = text[i] - 0x20;
            for (k = 0; k < 5; k++)
            {
                Image[128 * page + (column + k)] = ascii_table_5x8[j][k];

            }
            i++;
            (j == 0) ? (column += 5) : (column += 6);

        }
        else
            i++;

    }

}
void Disp_Str_8x16_Image(uint8_t page, uint8_t column, uint8_t *text)
{
    uint8_t i = 0, j, k, n;
    page = page - 1;
    column = column - 1;
    while (text[i] > 0x00)
    {
        if ((text[i] >= 0x20) && (text[i] <= 0x7E))
        {
            j = text[i] - 0x20;
            for (n = 0; n < 2; n++)
            {
                // LCD_Address_Set(page + n, column);
                for (k = 0; k < 8; k++)
                {
                    LCD_IMAGE[128 * (page + n) + (column + k)] =
                            ascii_table_8x16[j][k + 8 * n];
                }
            }
            i++;
            (j == 0) ? (column += 4) : (column += 8);
        }
        else
        {
            i++;
        }
    }
}
void Disp_20x20_Image(uint8_t page, uint8_t column, uint8_t *iPtr,
                      uint8_t *Image)
{
    page = page - 1;
    column = column - 1;
    uint8_t k, n;
    for (n = 0; n < 2; n++)
    {
        for (k = 0; k < 20; k++)
        {
            LCD_IMAGE[128 * (page + n) + (column + k)] = iPtr[20 * n + k];
        }

    }
}
void disp_bitmap(uint8_t page, uint8_t col, uint8_t *obj)
{
// LCD_Address_Set(page, col);
    uint8_t i, j;
    for (i = 0; i < 8; i++)
    {
        LCD_Write_Cmd(0xb0 | i);
        LCD_Write_Cmd(0x10);
        LCD_Write_Cmd(0X00);
        for (j = 0; j < 128; j++)
        {
            LCD_Write_Dat(obj[128 * i + j]);
        }
    }

}
void LCD_ST7567_Init()
{
    SET_RST;
    SysCtlDelay(533333);    // 20ms

    CLR_RST;
    SysCtlDelay(5333333);   // 200ms

    SET_RST;
    SysCtlDelay(533333);    // 20ms

    LCD_Write_Cmd(0xE2);
    SysCtlDelay(800000);

    LCD_Write_Cmd(0x2C);
    SysCtlDelay(133333);
    LCD_Write_Cmd(0x2E);
    SysCtlDelay(133333);
    LCD_Write_Cmd(0x2F);    //
    SysCtlDelay(133333);
    LCD_Write_Cmd(0xA6);

    LCD_Write_Cmd(0x23);
    LCD_Write_Cmd(0x82);

    LCD_Write_Cmd(0xA3);
    LCD_Write_Cmd(0xA4);

    LCD_Write_Cmd(0xAD);
    LCD_Write_Cmd(0x03);

    LCD_Write_Cmd(0x40);

    LCD_Write_Cmd(0xC8);    //
    LCD_Write_Cmd(0xA0);

    LCD_Write_Cmd(0xAF);    //

    SysCtlDelay(2666666);

    LIGHT_ON
    LCD_Disp_Clr(0x00);
    ;
}
static void LCD_Disp_Clr(uint8_t dat)
{
    uint8_t i, j; /* Use this, set the address at each row */
    for (i = 0; i < 8; i++)
    {
        LCD_Write_Cmd(0xb0 | i);
        LCD_Write_Cmd(0x10);
        LCD_Write_Cmd(0X00);
        for (j = 0; j < 128; j++)
        {
            LCD_Write_Dat(dat);
        }
    }
}

