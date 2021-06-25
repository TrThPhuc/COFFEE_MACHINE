//----------------------------------------------------------------------------------
//  FILE:           Coffee_Machine-Main.C
//
//  Description:    Automatic Coffee Machine
//
//  Version:        1.0
//
//  Target:         TM4C123(ARM M4)
//
//----------------------------------------------------------------------------------
//  Copyright Davi-Engineering
//----------------------------------------------------------------------------------
//  Revision History:
//----------------------------------------------------------------------------------
//  Date      | Description / Status
//----------------------------------------------------------------------------------
// 22 June 2021 - Coffee machine firmware
//==================================================================================
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "inc/hw_memmap.h"
#include "inc/hw_qei.h"
#include "inc/hw_types.h"
#include "inc/hw_pwm.h"

#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/ssi.h"
#include "driverlib/pin_map.h"
#include "driverlib/qei.h"
#include "driverlib/i2c.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"

#include "ADS1118.h"
#include "TCA9539.h"
#include "PID.h"
#include "TCA9539_hw_memmap.h"
#include "Coffee_Machine.h"
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//////////////////////////////////PROTOTYPES///////////////////////////////////////////////
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// SYSTEM FUNCTION
//---------------------------------------------------------------
extern void InitSysClt(void);      // Initialize system & peripheral clock
void defaultISR(void);      // Default interrupt handler
void GpioConfigure(void);   //
void TimerSysClt(void);
void (*Ptr_Task)(void);     // Pointer task
// The error routine that is called if the driver library encounters an error.
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    while(1);
}
#endif
// Machine Defines
// -------------------------------------------------------------------------------------------
#define Lifespan_Of_Blade               1000
#define Lifespan_Of_Ron                 1000
// Period count = 800 corresponding to 10 KHz @ 80 MHz (Down count mode)
#define PWM_PRD_Motor_PressModule       8000
#define PWM_PRD_Motor_PumpModule        8000
// Status machine
#define Idle_Process                    0
#define Epresso_Process_1               1
#define Epresso_Process_2               2
#define Decatt_Process_1                3
#define Decatt_Process_2                4
#define Reset                           5
#define Reset_All                       6
// Constant string
const char *Producer = "Davi Engineering";
const char *SerialProduct = "xx";
const char *Model = "xx";
uint32_t clockrate; // System clock
// ---------------------------------- USER --------------------------------------------------
// Temperature controll
// PID coeficient translate to zeros and poles of heting process
// PID Steam
uint16_t Pgain_Steam, Igain_Steam, Dgain_Steam, Dmax_Steam;
// PID Hot water
uint16_t Pgain_HotWater, Igain_HotWater, Dgain_HotWater, Dmax_HotWater;
// Vaiables
uint8_t coef_change;
float Steam_Temperature_Ref, Steam_Vout;
float HotWater_Temperature_Ref, HotWater_Vout;
CNTL_2P2Z_Terminal_t Steam_CNTL, HotWater_CNTL;
uint16_t Process_status;
uint16_t SumOfCupInUsed = 0;
uint16_t SumOfCupInUsed_day = 0;
uint16_t Blade, Ron;

Mode_Parameter_t Espresso_1, Espresso_2, Decatt_1, Decatt_2;  // Mode parameter
Mode_Parameter_t *ModeSelected;
// Temperature monitor/GUI
float Steam_Temp_Gui;
float HotWater_Temp_Gui;
float PressHeating_Temp_Gui;
// ---------------------------------- LCD Interface -----------------------------------------
// Reset LCD - PA6
#define SET_RST     GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6)
#define CLR_RST     GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0)
// A0/RS for Cmd or data - PA7
#define  SET_RS     GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7)
#define  CLR_RS     GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0)

//#define  LIGHT_ON   GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1)
//#define  LIGHT_OFF  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0)
// Initialize and configure LCD through SPI interface
void LCD_Interface_Cnf(void);
void LCD_ST7567_Init(void);
void Spi0_LCD_Interface_Cnf(void);
//Kernel for Communicate Host to LCD
void SerialCommsInit(void); // Initialize task
void SerialHostComms(void); // Task proceessed in period
// Low Level driver
void LCD_Write_Cmd(uint8_t cmd);    // Write command to LCD
void LCD_Write_Dat(uint8_t dat);    // Write data to LCD
void LCD_Address_Set(uint8_t page, uint8_t column); // Set cursor
// Display ASCII string
void Disp_Str_5x8(volatile uint8_t page, volatile uint8_t column, uint8_t *text);
void LCD_Disp_Clr(uint8_t dat);     // Clear LCD
//Variable
uint16_t *dataSentList[10]; // Terminal connect to monitor variable
uint16_t *Pr_Packet[16];    // Parameter
uint8_t coeff_change;   // Flag for change parameter in mode
int16_t VrTimer1[4];    // Virtual timer
extern const unsigned char ascii_table_5x8[95][5]; // Bit Map for ASCII table (ASCII_Font.c)
// ---------------------------- ADS1118 Temperature Sensor ------------------------------------
ADS1118_t Steam, Hot_Water;
void Spi1_ADS1118_Interface_Cnf(void); // Configurate Spi for communicate ADS1118
void ADS1118_Cal(ADS1118_t *ADS); // Calculate temperature
int32_t dummyTemp;
// ---------------------------- TCA9539 IO Expander Module -------------------------------------
TCA9539Regs TCA9539_IC1, TCA9539_IC2, TCA9539_IC3;
extern void I2C0_TCA9539_Configuration(void);
extern void I2C0_TCA9539_IterruptTrigger_Cnf();
// ------------------------------------- Flow Meter --------------------------------------------
float Calibration; // Coeficient for calculate vollume pump
uint32_t totalMilliLitres, MilliLitresBuffer;
uint32_t SetVolume;
volatile bool FinishPumpEvent = false;
void FlowMeterCal(void);
void InitPumpingEvent(void);
// -------------------------------------Driver BLDC Motor ---------------------------------------
// Configure 3 pwm chanel for 3 BLDC motor
#define I2C0
#define SPI0 #define SPI1
#define QEI0
#define All_Of_GPIO
void PWMDRV_Coffee_machine_cnf(void);
// -------------------------------------- Group Task ---------------------------------------------
void (*A_Group_Task)(void); // 2ms Task
void (*B_Group_Task)(void); // 5ms Task
void (*C_Group_Task)(void); // 100ms Task

void A_Base(void);
void B_Base(void);  // Monitor machine
void C_Base(void);

void A1(void);
void Default_B(void);
void Default_C(void);
uint16_t data[10] = { };
uint16_t parameter[10] = { };
extern void MakeCoffee(void);
extern void MakeCoffeProcess(void);
extern bool InProcess;
uint32_t duty = 100;
void main()
{
    // Initialize Device/board include:
    // + Disable Wdog timer
    // + System clk, Peripheral clock
    // + GPIO init
    InitSysClt();
    TimerSysClt();
    Ptr_Task = &A_Base;
    A_Group_Task = &A1;
    B_Group_Task = &Default_B;
    B_Group_Task = &Default_C;
    // C_Group_Task = &C1;
    clockrate = SysCtlClockGet();

// ---------------------------------- USER -----------------------------------------
//=================================================================================
//  Temperature Controll terminal assign
    CNTL_2P2Z_DBUFF_t Default = { 0, 0, 0, 0, 0 };
    Steam_CNTL.Ref = &Steam_Temperature_Ref;
    Steam_CNTL.Fdbk = &Steam.Actual_temperature;
    Steam_CNTL.Out = &Steam_Vout;
    Steam_CNTL.DBUFF = Default;
    CNTL_Pole_Zero_Cal(&Steam_CNTL, Pgain_Steam, Igain_Steam, Dgain_Steam,
                       Dmax_Steam, 0, -0.9);

    HotWater_CNTL.Ref = &HotWater_Temperature_Ref;
    HotWater_CNTL.Fdbk = &Hot_Water.Actual_temperature;
    HotWater_CNTL.Out = &HotWater_Vout;
    HotWater_CNTL.DBUFF = Default;
    CNTL_Pole_Zero_Cal(&HotWater_CNTL, Pgain_HotWater, Igain_HotWater,
                       Dgain_HotWater, Dmax_Steam, 0, -0.9);

//=================================================================================

    PWMDRV_Coffee_machine_cnf();
//=================================================================================
//  INITIALISATION - LCD-Display connections
//=================================================================================
    LCD_Interface_Cnf(); // SPI & I/O configruation
    LCD_ST7567_Init();   // LCD initialize
    LCD_Disp_Clr(0x00);
    // Initialize GUI interface
    SerialCommsInit();
    // Assign data stream to Gui variable display LCD - Display on Page 0 LCD
    dataSentList[0] = &SumOfCupInUsed;
    dataSentList[1] = &SumOfCupInUsed_day;
    dataSentList[2] = &Blade;
    dataSentList[3] = &Ron;
    //"Set" variables
    //---------------------------------------
    // Assign GUI parameter  to desired  parameter setting addresses
    Pr_Packet[0] = &Espresso_1.Water;
    Pr_Packet[1] = &Espresso_1.GrindingDuration;
    Pr_Packet[2] = &Espresso_1.AmountOfWaterPumping.stage_1;
    Pr_Packet[3] = &Espresso_1.AmountOfWaterPumping.stage_2;

    Pr_Packet[4] = &Espresso_2.Water;
    Pr_Packet[5] = &Espresso_2.GrindingDuration;
    Pr_Packet[6] = &Espresso_2.AmountOfWaterPumping.stage_1;
    Pr_Packet[7] = &Espresso_2.AmountOfWaterPumping.stage_2;

    Pr_Packet[8] = &Decatt_1.Water;
    Pr_Packet[9] = &Decatt_1.GrindingDuration;
    Pr_Packet[10] = &Decatt_1.AmountOfWaterPumping.stage_1;
    Pr_Packet[11] = &Decatt_1.AmountOfWaterPumping.stage_2;

    Pr_Packet[12] = &Decatt_2.Water;
    Pr_Packet[13] = &Decatt_2.GrindingDuration;
    Pr_Packet[14] = &Decatt_2.AmountOfWaterPumping.stage_1;
    Pr_Packet[15] = &Decatt_2.AmountOfWaterPumping.stage_2;

    Calibration = (float) 2 / 15.0;
    // Assign direction motor of grind module
    Espresso_1.DirGrinding = Espresso_2.DirGrinding = true;
    Decatt_1.DirGrinding = Decatt_2.DirGrinding = false;

//=================================================================================
//  ADS1118 - Termperature Sensor Configuration - 2 channel
//=================================================================================
    Spi1_ADS1118_Interface_Cnf();   // Configurate spi1
    ADS_Config(0);
//=================================================================================
//  TCA9539 - I/O Expander Configuration - 2 channel
//=================================================================================
    TCA9539_IC1._Id = 0x74;
    TCA9539_IC2._Id = 0x75;
    TCA9539_IC3._Id = 0x77;
    I2C0_TCA9539_Configuration();
    I2C0_TCA9539_IterruptTrigger_Cnf();

// ----------------------------------  Configure QEI -----------------------------------------

    GPIOUnlockPin(GPIO_PORTD_BASE, GPIO_PIN_7);
    GPIOPinConfigure(GPIO_PD6_PHA0);
    GPIOPinConfigure(GPIO_PD7_PHB0);
    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA,
    GPIO_PIN_TYPE_STD_WPD);
    QEIDisable(QEI0_BASE);
    QEIIntDisable(QEI0_BASE,
    QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

    QEIConfigure(
            QEI0_BASE,
            QEI_CONFIG_NO_RESET | QEI_CONFIG_CLOCK_DIR | QEI_CONFIG_NO_SWAP,
            0xFFFFFFFF);
    HWREG(QEI0_BASE + QEI_O_CTL) = ((HWREG(QEI0_BASE + QEI_O_CTL)
            & ~(QEI_CTL_INVI)) | QEI_CTL_INVI);
    QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_16, 500000);
    QEIVelocityEnable(QEI0_BASE);
    //QEIIntRegister(QEI0_BASE, FlowMeterCal);
    //QEIIntEnable(QEI0_BASE, QEI_INTTIMER);
    // QEIEnable(QEI0_BASE);
    QEIDisable(QEI0_BASE);

    SetVolume = 200;
    InitPumpingEvent();

    duty = 1000;
    while (1)
    {
        Ptr_Task();
    }
}
// Task 5ms
void A_Base(void)
{
    if (TimerIntStatus(TIMER0_BASE, false) == TIMER_TIMA_TIMEOUT)
    {
        TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
        A_Group_Task();
    }
    Ptr_Task = &B_Base;
}
// Task 5ms
void B_Base(void)
{
    if (TimerIntStatus(TIMER1_BASE, false) == TIMER_TIMA_TIMEOUT)
    {
        TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
        B_Group_Task();
    }

    Ptr_Task = &C_Base;
}
void C_Base(void)
{
    if (TimerIntStatus(TIMER2_BASE, false) == TIMER_TIMA_TIMEOUT)
    {
        TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
        C_Group_Task();
    }
    Ptr_Task = &A_Base;
}
// Task 2ms
void A1(void)
{
    // Communicate and display LCD
    SerialHostComms();

}
// Task 5ms
void Default_B(void)
{
    // Scan to output
    if (TCA9539_IC1.updateOutputFlag)
        TCA9539WriteOutput(&TCA9539_IC1);
    if (TCA9539_IC2.updateOutputFlag)
        TCA9539WriteOutput(&TCA9539_IC2);
    if (TCA9539_IC3.updateOutputFlag)
        TCA9539WriteOutput(&TCA9539_IC3);
    if (InProcess)
        B_Group_Task = &MakeCoffeProcess;
}
// Task 50ms
void Default_C(void)
{
    if (InProcess == 0)
    {
        if (TCA9539_IC1.TCA9539_Input.all & Decatt1_Bt)
        {
            ModeSelected = &Decatt_1;
            MakeCoffee();
        }
        else if (TCA9539_IC1.TCA9539_Input.all & Decatt2_Bt)
        {
            ModeSelected = &Decatt_2;
            MakeCoffee();
        }
        else if (TCA9539_IC1.TCA9539_Input.all & Expresso1_Bt)
        {
            ModeSelected = &Espresso_1;
            MakeCoffee();
        }
        else if (TCA9539_IC1.TCA9539_Input.all & Expresso1_Bt)
        {
            ModeSelected = &Espresso_2;
            MakeCoffee();
        }
    }
}

void Spi0_LCD_Interface_Cnf()
{
    // Configurate pin mux for SSI(SPI) peripheral function
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    SSIClockSourceSet(SSI0_BASE, SSI_CLOCK_SYSTEM);
    // COnfige Mode 0 SSI, Freq = 100Khz, 8Bit
    SSIConfigSetExpClk(SSI0_BASE, 80000000, SSI_FRF_MOTO_MODE_0,
    SSI_MODE_MASTER,
                       100000, 8);

    SSILoopbackEnable(SSI0_BASE);

}
void LCD_Interface_Cnf()
{
    Spi0_LCD_Interface_Cnf();
    // Configurate Pin for proper ssi
    GPIOPinTypeSSI(GPIO_PORTA_BASE,
    GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
    SSIEnable(SSI0_BASE);

    // Configurate Pin for other
    /*    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_STRENGTH_12MA,
     GPIO_PIN_TYPE_STD_WPU);*/
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_STRENGTH_12MA, //PA6 - LCD_RST
                     GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_STRENGTH_12MA, // PA7 - LCD_RS
                     GPIO_PIN_TYPE_STD_WPU);

    //  GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_DIR_MODE_OUT);     // LCD_LED
    GPIODirModeSet(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_DIR_MODE_OUT);
    GPIODirModeSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_DIR_MODE_OUT);

    //   GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);

}
void LCD_Write_Cmd(uint8_t cmd)
{
    CLR_RS;

    SSIDataPut(SSI0_BASE, (uint32_t) cmd);
    while (SSIBusy(SSI0_BASE))
    {
    }
    // SSIDataGet(SSI0_BASE, (uint32_t*) &rdata);     // read dummy data

}
void LCD_Write_Dat(uint8_t cmd)
{
    SET_RS;
    SSIDataPut(SSI0_BASE, (uint32_t) cmd);
    while (SSIBusy(SSI0_BASE))
    {
    }
    //  SSIDataGet(SSI0_BASE, (uint32_t*) &rdata);     // read dummy data

}
void LCD_Address_Set(uint8_t page, uint8_t column)
{
    column = (column - 1) & 0x00FF;
    page = (page - 1) & 0x00FF;
    LCD_Write_Cmd(0xb0 + page);
    LCD_Write_Cmd(((column >> 4) & 0x0f) + 0x10);
    LCD_Write_Cmd(column & 0x0f);
}
void Disp_Str_5x8(volatile uint8_t page, volatile uint8_t column, uint8_t *text)
{

    uint8_t i = 0, j, k;
    while (text[i] > 0x00)
    {
        if ((text[i] >= 0x20) && (text[i] <= 0x7e))
        {
            j = text[i] - 0x20;
            LCD_Address_Set(page, column);
            for (k = 0; k < 5; k++)
            {
                LCD_Write_Dat(ascii_table_5x8[j][k]);
            }
            i++;
            column += 5;

        }
        else
            i++;
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

    //CLR_CS;

    LCD_Write_Cmd(0xE2);
    SysCtlDelay(800000);

    LCD_Write_Cmd(0x2C);
    SysCtlDelay(133333);
    LCD_Write_Cmd(0x2E);
    SysCtlDelay(133333);
    LCD_Write_Cmd(0x2F);
    SysCtlDelay(133333);
    LCD_Write_Cmd(0xA6);

    LCD_Write_Cmd(0x23);
    LCD_Write_Cmd(0x82);

    LCD_Write_Cmd(0xA3);
    LCD_Write_Cmd(0xA4);

    LCD_Write_Cmd(0xAD);
    LCD_Write_Cmd(0x03);

    LCD_Write_Cmd(0x40);

    LCD_Write_Cmd(0xC8);
    LCD_Write_Cmd(0xA0);

    LCD_Write_Cmd(0xAF);

    SysCtlDelay(2666666);
    // SET_CS;
    // LIGHT_ON;
}
void LCD_Disp_Clr(uint8_t dat)
{
    uint8_t i, j; /* so we use this, set the address at each row */
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
void ADS1118_Coms(uint16_t config, int mode, int32_t *_result)
{

    if (mode == 1)
        config = config | 0x8000;
    SSIDataPut(SSI1_BASE, (uint32_t) config);
    while (SSIBusy(SSI1_BASE))
        ;
    SSIDataGet(SSI1_BASE, (uint32_t*) _result);
}
void ADS1118_Cal(ADS1118_t *ADS)
{

    int16_t temp;
    if (GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_1) != 0)
    {
        if (ADS->swBit)
        {
            ADS_Read(1, &ADS->cold_data, ADS->Code);
            ADS->swBit = false;
        }
        else
        {
            ADS_Read(0, &ADS->hot_data, ADS->Code);
            temp = ADS->hot_data + local_compensation(ADS->cold_data);
            ADS->Actual_temperature  = ADC_code2temp(temp);

            ADS->swBit = true;
        }
    }

}
void Spi1_ADS1118_Interface_Cnf()
{
    GPIOPinConfigure(GPIO_PD0_SSI1CLK);
    GPIOPinConfigure(GPIO_PD1_SSI1FSS);
    GPIOPinConfigure(GPIO_PD2_SSI1RX);
    GPIOPinConfigure(GPIO_PD3_SSI1TX);
    SSIClockSourceSet(SSI1_BASE, SSI_CLOCK_SYSTEM);
    // COnfige Mode 0 SSI, Freq = 100Khz, 8Bit
    SSIConfigSetExpClk(SSI1_BASE, 80000000, SSI_FRF_MOTO_MODE_1,
    SSI_MODE_MASTER,
                       100000, 16);
    GPIOPinTypeSSI(GPIO_PORTD_BASE,
    GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    SSIEnable(SSI1_BASE);
}
void FlowMeterCal()
{
    QEIIntClear(QEI0_BASE, QEI_INTTIMER);
    uint32_t temp;
    temp = QEIPositionGet(QEI0_BASE);
    MilliLitresBuffer = (float) temp * Calibration;
    if (MilliLitresBuffer >= SetVolume)
    {
        totalMilliLitres = MilliLitresBuffer;
        QEIIntRegister(QEI0_BASE, &defaultISR);
        QEIDisable(QEI0_BASE);
        QEIIntDisable(QEI0_BASE, QEI_INTTIMER);
        FinishPumpEvent = true;

    }

}
void InitPumpingEvent()

{
    MilliLitresBuffer = 0;
    QEIPositionSet(QEI0_BASE, 0x0000);
    QEIIntRegister(QEI0_BASE, &FlowMeterCal);
    QEIIntEnable(QEI0_BASE, QEI_INTTIMER);
    QEIEnable(QEI0_BASE);
}

void defaultFunction()
{

}
void defaultISR(void)
{

}

