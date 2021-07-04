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

#include "Coffee_Machine.h"
#include "ADS1118.h"
#include "TCA9539.h"
#include "PID.h"
#include "TCA9539_hw_memmap.h"

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//////////////////////////////////PROTOTYPES///////////////////////////////////////////////
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// SYSTEM FUNCTION
//---------------------------------------------------------------
#define Null 0
#define Int_SSI0
//#define Polling_SSI0
extern void InitSysClt(void);      // Initialize system & peripheral clock
void defaultISR(void);      // Default interrupt handler
void GpioConfigure(void);   //
void TimerSysClt(void);
void (*Ptr_Task)(void);     // Pointer task
extern void Cmd_ReadMsg(void);
extern void Cmd_WriteMsg(void (*pFun)(void*), void *pArg);
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
uint16_t Pgain_Steam, Igain_Steam, Dgain_Steam;
uint32_t Dmax_Steam;
// PID Hot water
uint16_t Pgain_HotWater, Igain_HotWater, Dgain_HotWater;
uint32_t Dmax_HotWater;
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

extern void Init_SW_DMA(void);
extern void Init_SPI_DMA(void);
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
extern void WriteImageToDriverLCD(void *bufferPtr);
uint8_t *LCD_IMAGE_Send, *LCD_IMAGE_Write;
extern void ReadTxFiFO(void);
extern void WriteTxFiFO(uint8_t c);
// ---------------------------- ADS1118 Temperature Sensor ------------------------------------
ADS1118_t Steam, Hot_Water;
void Spi1_ADS1118_Interface_Cnf(void); // Configurate Spi for communicate ADS1118
extern void SSI1_IntHandler(void);
void ADS1118_Cal(ADS1118_t *ADS); // Calculate temperature
int32_t dummyTemp;
volatile uint8_t datacount = 0;
uint32_t datas, fb_config;
#define MEM_BUFFER_SIZE         2
extern uint16_t ui16TxBuf[];
extern uint16_t ui16RxBuf[];
uint16_t volatile Settingconfig;
// ---------------------------- TCA9539 IO Expander Module -------------------------------------
TCA9539Regs TCA9539_IC1, TCA9539_IC2, TCA9539_IC3;
TCA9539Regs *TCA9539_IC[3] = { &TCA9539_IC1, &TCA9539_IC2, &TCA9539_IC3 };
extern void I2C0_TCA9539_Configuration(void);
extern void I2C0_TCA9539_IterruptTrigger_Cnf();
extern volatile uint8_t Tx_slavecount, Tx_usedBrust;
extern volatile uint8_t Rx_slavecount, Rx_usedBrust;
// ------------------------------------- Flow Meter --------------------------------------------
float Calibration; // Coeficient for calculate vollume pump
uint32_t totalMilliLitres, MilliLitresBuffer;
uint32_t SetVolume;
volatile bool FinishPumpEvent = false;
void FlowMeterCal(void);
void InitPumpingEvent(void);

extern void SteamLevelControl_Run(void *PrPtr);
extern void SteamLevelControl_Stop(void *PrPtr);
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
void A2(void);
void C1(void);
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
    // + Disable Wdog timer, Disable interrupt
    // + System clk, Peripheral clock
    // + GPIO init
    IntMasterDisable();
    InitSysClt();
    TimerSysClt();
    Ptr_Task = &A_Base;
    A_Group_Task = &A1;
    B_Group_Task = &Default_B;
    C_Group_Task = &Default_C;
    // C_Group_Task = &C1;
    clockrate = SysCtlClockGet();

// ---------------------------------- USER -----------------------------------------
//=================================================================================
//  Temperature Control terminal assign
    CNTL_2P2Z_DBUFF_t Default = { 0, 0, 0, 0, 0 };
    Steam.Code = ADSCON_CH0;
    Steam.Actual_temperature = 0;
    Steam_CNTL.Ref = &Steam_Temperature_Ref;
    Steam_CNTL.Fdbk = &Steam.Actual_temperature;
    Steam_CNTL.Out = &Steam_Vout;
    Steam_CNTL.DBUFF = Default;
    Dmax_Steam = Dmax_HotWater = 80000;
    CNTL_Pole_Zero_Cal(&Steam_CNTL, Pgain_Steam, Igain_Steam, Dgain_Steam,
                       Dmax_Steam, 0, -100);

    Hot_Water.Code = ADSCON_CH1;
    Hot_Water.Actual_temperature = 0;
    HotWater_CNTL.Ref = &HotWater_Temperature_Ref;
    HotWater_CNTL.Fdbk = &Hot_Water.Actual_temperature;
    HotWater_CNTL.Out = &HotWater_Vout;
    HotWater_CNTL.DBUFF = Default;
    CNTL_Pole_Zero_Cal(&HotWater_CNTL, Pgain_HotWater, Igain_HotWater,
                       Dgain_HotWater, Dmax_HotWater, 0, -0.9);

//=================================================================================

    PWMDRV_Coffee_machine_cnf();

//=================================================================================
//  ADS1118 - Termperature Sensor Configuration - 2 channel
//=================================================================================
    Spi1_ADS1118_Interface_Cnf();   // Configurate spi1
    ADS_Config(0);
//=================================================================================
//  TCA9539 - I/O Expander Configuration - 2 channel
//=================================================================================
    TCA9539_IC1._Id = 0x74;
    TCA9539_IC1.TCA9539_Onput.all = 0xABCD;
    TCA9539_IC1.updateOutputFlag = 1;
    TCA9539_IC2._Id = 0x74;
    TCA9539_IC2.TCA9539_Onput.all = 0x01EF;
    TCA9539_IC2.updateOutputFlag = 1;   //0x75
    TCA9539_IC3._Id = 0x74;
    TCA9539_IC3.TCA9539_Onput.all = 0x5678;
    TCA9539_IC3.updateOutputFlag = 1;   //0x77
    I2C0_TCA9539_Configuration();
    I2C0_TCA9539_IterruptTrigger_Cnf(); // Configure GPIO interrupt to respone intertupt signal of TCA9539

// ----------------------------------  Configure QEI -----------------------------------------

    //QEIIntRegister(QEI0_BASE, FlowMeterCal);
    //QEIIntEnable(QEI0_BASE, QEI_INTTIMER);
    // QEIEnable(QEI0_BASE);
    QEIDisable(QEI0_BASE);
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

    //  SetVolume = 200;
    //  InitPumpingEvent();
//=============================Enable And start System ========================================
    TimerEnable(TIMER0_BASE, TIMER_A);
    TimerEnable(TIMER1_BASE, TIMER_A);
    TimerEnable(TIMER2_BASE, TIMER_A);
    TimerEnable(TIMER3_BASE, TIMER_A);

//Clear interrupt Flag & enable interrupt (CPU level)
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT); // Interrupt timer
    IntMasterEnable();

    while (1)
    {
        Ptr_Task();
        Cmd_ReadMsg();

    }
}
// Task 2ms
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
    A_Group_Task = &A2;

}
void A2(void)
{
    if (TCA9539_IC1.ReadCmdFlag && !I2CMasterBusBusy(I2C0_BASE))
        Cmd_WriteMsg((void (*)(void*)) TCA9539ReadInputReg,
                     (void*) &TCA9539_IC1);
    if (TCA9539_IC2.ReadCmdFlag && !I2CMasterBusBusy(I2C0_BASE))
        Cmd_WriteMsg((void (*)(void*)) TCA9539ReadInputReg,
                     (void*) &TCA9539_IC2);
    if (TCA9539_IC3.ReadCmdFlag && !I2CMasterBusBusy(I2C0_BASE))
        Cmd_WriteMsg((void (*)(void*)) TCA9539ReadInputReg,
                     (void*) &TCA9539_IC3);
    A_Group_Task = &A1;
}
// Task 5ms

void Default_B(void)
{
    // Scan to output

    if (TCA9539_IC1.updateOutputFlag && !I2CMasterBusBusy(I2C0_BASE))
        Cmd_WriteMsg((void (*)(void*)) TCA9539WriteOutput,
                     (void*) &TCA9539_IC1);
    if (TCA9539_IC2.updateOutputFlag && !I2CMasterBusBusy(I2C0_BASE))
        Cmd_WriteMsg((void (*)(void*)) TCA9539WriteOutput,
                     (void*) &TCA9539_IC2);
    if (TCA9539_IC3.updateOutputFlag && !I2CMasterBusBusy(I2C0_BASE))
        Cmd_WriteMsg((void (*)(void*)) TCA9539WriteOutput,
                     (void*) &TCA9539_IC3);

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
    C_Group_Task = &C1;
}
void C1(void)
{
    if ((TCA9539_IC3.TCA9539_Input.all & LevelSensor1) == 0)
        Cmd_WriteMsg(&SteamLevelControl_Run, Null);
    else
        Cmd_WriteMsg(&SteamLevelControl_Stop, Null);
    C_Group_Task = &Default_C;
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
                       20000000, 8);
#ifdef Int_SSI0
    SSIIntRegister(SSI0_BASE, &ReadTxFiFO);
    SSIIntEnable(SSI0_BASE, SSI_TXFF);
#endif
    //SSILoopbackEnable(SSI0_BASE);

}
void LCD_Interface_Cnf()
{
    Spi0_LCD_Interface_Cnf();
    // SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    IntEnable(INT_SSI0);
    //uDMAEnable();
    // Init_SW_DMA();
    //Init_SPI_DMA();
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
#ifdef Polling_SSI0
    SSIDataPut(SSI0_BASE, (uint32_t) cmd);
    while (SSIBusy(SSI0_BASE))  // Polling method
    {
    }
#endif
#ifdef Int_SSI0
    SSIIntEnable(SSI0_BASE, SSI_TXFF);
    WriteTxFiFO(cmd);
#endif

}
void LCD_Write_Dat(uint8_t cmd)
{

    SET_RS;
#ifdef Polling_SSI0
    SSIDataPut(SSI0_BASE, (uint32_t) cmd);
    while (SSIBusy(SSI0_BASE))  // polling method
    {
    }
    //  SSIDataGet(SSI0_BASE, (uint32_t*) &rdata);     // read dummy data
#endif
#ifdef uDma_SSI0

    WriteImageToDriverLCD(LCD_IMAGE_Send);
#endif
#ifdef Int_SSI0
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
void Disp_Str_5x8(volatile uint8_t page, volatile uint8_t column, uint8_t *text)
{

    uint8_t i = 0, j, k;

    //index = 600;
    while (text[i] > 0x00)
    {

        //
        if ((text[i] >= 0x20) && (text[i] <= 0x7e))
        {
            j = text[i] - 0x20;
#if (defined  Polling_SSI0) || (defined  Int_SSI0)
            LCD_Address_Set(page, column);
            for (k = 0; k < 5; k++)
            {
                LCD_Write_Dat(ascii_table_5x8[j][k]);
            }
            i++;
            column += 5;
#endif
#ifdef uDma_SSI0
            static uint16_t index = 0;

                for (k = 0; k < 5; k++)
                {
                    LCD_IMAGE_Write[index] = (uint8_t) ascii_table_5x8[15][k];
                    index++;
                    if(index >= 1024)   index = 0;
                }
                i++;
                column += 5;

#endif
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

    LCD_Write_Cmd(0x7f);
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
void ADS1118_Coms(uint16_t config, int mode)
{

    /*    if (mode == 1)
     config = config | 0x8000;   // Signgle shot conversion

     SSIDataPut(SSI1_BASE, (uint32_t) config);
     SSIDataPut(SSI1_BASE, (uint32_t) config);
     while (SSIBusy(SSI1_BASE))
     ;
     SSIDataGet(SSI1_BASE, (uint32_t*) &datas);
     SSIDataGet(SSI1_BASE, (uint32_t*) &fb_config);
     config = ((config & 0x7FFF) | 0x01);
     if (config == fb_config)            // CH0 0x8B8A   // Steam
     {
     if (fb_config == (0xB9B))       //8b9a & 7FFF = B9A
     Steam.hot_data = datas;     //Steam.Code = 0x8B8A
     else if (fb_config == 0xB8B)    //8b8a
     Steam.cold_data = datas;
     else if (fb_config == 0x3B9B)   // CH1 BB8A         // Hot_Water
     Hot_Water.hot_data = datas; //bb9a & 7FFF = 3b9a
     else if (fb_config == 0x3B8B)   //bb8a & 7fff = 3b8a
     Hot_Water.cold_data = datas;
     }*/
    config = config | 0x8000;
    ui16TxBuf[0] = config;
    ui16TxBuf[1] = config;
 //   Settingconfig = ((ui16TxBuf[0] & 0x7FFF) | 0x01);
    uDMAChannelEnable(UDMA_CHANNEL_SSI1TX);
    uDMAChannelEnable(UDMA_CHANNEL_SSI1RX);

}
void ADS1118_Cal(ADS1118_t *ADS)
{

    int16_t temp;

    /*
     ADS_Read(1, ADS->Code);     // Hot data request, read  cold data
     SysCtlDelay(1333333);
     ADS_Read(0, ADS->Code);     // cold data request, read hot data
     */
    if (ADS->swBit)
    {
        ADS_Read(0, ADS->Code);
        ADS->swBit = 0;
    }
    else
    {
        ADS->swBit = 1;
        ADS_Read(1, ADS->Code);
    }

    temp = ADS->hot_data + local_compensation(ADS->cold_data);
    ADS->Actual_temperature = ADC_code2temp(temp);

}
void Spi1_ADS1118_Interface_Cnf()
{
    GPIOUnlockPin(GPIO_PORTF_BASE, GPIO_PIN_0); // unlock pin PF0
    GPIOPinConfigure(GPIO_PF2_SSI1CLK);
    GPIOPinConfigure(GPIO_PF3_SSI1FSS);
    GPIOPinConfigure(GPIO_PF0_SSI1RX);
    GPIOPinConfigure(GPIO_PF1_SSI1TX);
    SSIClockSourceSet(SSI1_BASE, SSI_CLOCK_SYSTEM);
// COnfige Mode 0 SSI, Freq = 100Khz, 8Bit
    SSIConfigSetExpClk(SSI1_BASE, 80000000, SSI_FRF_MOTO_MODE_1,
    SSI_MODE_MASTER,
                       2500000, 16);
    GPIOPinTypeSSI(GPIO_PORTF_BASE,
    GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    SSIIntRegister(SSI1_BASE, &SSI1_IntHandler);
    SSIIntEnable(SSI1_BASE, SSI_DMATX);
    SSIIntEnable(SSI1_BASE, SSI_DMARX);
    IntEnable(INT_SSI1);
    uDMAEnable();
    Init_SPI_DMA();
    SSIEnable(SSI1_BASE);
}

void defaultFunction()
{

}
void defaultISR(void)
{

}

