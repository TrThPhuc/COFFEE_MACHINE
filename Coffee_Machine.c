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
//#define uDma_SSI0
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
// Address store in  EEPROM @ offset from 0x400AF000
#define AddDataEeprom   0x00
// Constant string
const char *Producer = "Davi Engineering";
const char *SerialProduct = "xx";
uint32_t clockrate; // System clock
uint32_t EEPROMInitStastus;
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
uint8_t coefSteam_change, coefHotWater_change;    ////////////////
float Steam_Temperature_Ref, Steam_Vout;
float HotWater_Temperature_Ref, HotWater_Vout;
float Extrude_Vout;
float Gui_TempSteam, Gui_HotWaterSteam, Gui_CoffeExtractionTime, CountSteam;
CNTL_2P2Z_Terminal_t Steam_CNTL, HotWater_CNTL;
//uint32_t PitchOfpress;

uint16_t Process_status;    ////////////
uint16_t SumOfCupInUsed = 0;
uint16_t SumOfCupInUsed_day = 0;
uint16_t Blade, Ron;    //Used times of Blade and Ron
// Mode parameter
Mode_Parameter_t Mode_Espresso_1, Mode_Espresso_2, Mode_Special_1,
        Mode_Special_2;
Mode_Parameter_t *ModeSelected;
float PulWeightRatio;  // Weight calibration

bool cancel_cmd, test_step = 0;    // cancel command
// Temperature monitor/GUI-
        /*float Steam_Temp_Gui;
         float HotWater_Temp_Gui;
         float PressHeating_Temp_Gui;*/
// ---------------------------------- LCD Interface -----------------------------------------
// Reset LCD - PA6
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

// Initialize and configure LCD through SPI interface
void Spi0_LCD_Interface_Cnf(void);  // Configure SSI interface for LCD
void LCD_Interface_Cnf(void);   //  Configure GPIO and interface for LCD

void LCD_ST7567_Init(void);     // Initial Config  LCD

extern void Init_SW_DMA(void);
extern void Init_SPI_DMA(void);
extern void Init_LCDSPI_DMA(void);

//Kernel for Communicate Host to LCD
void SerialCommsInit(void); // Initialize task
void SerialHostComms(void); // Task proceessed in period

// Low Level driver
void LCD_Write_Cmd(uint8_t cmd);    // Write command to LCD
void LCD_Write_Dat(uint8_t dat);    // Write data to LCD
void LCD_Address_Set(uint8_t page, uint8_t column); // Set cursor
// Display ASCII string
void Disp_Str_5x8(volatile uint8_t page, volatile uint8_t column, uint8_t *text);
void Disp_Str_8x16(uint8_t page, uint8_t column, uint8_t *text);
void LCD_Disp_Clr(uint8_t dat);     // Clear LCD
//Variable
extern uint8_t LCD_IMAGE[1024];
uint16_t *dataSentList[16]; // Terminal connect to monitor variable
uint32_t *Pr_Packet[32];    // Terminal connect to Parameter
uint8_t coeff_change;   // Flag for change parameter in mode /////////
int16_t VrTimer1[4];    // Virtual timer
extern const unsigned char ascii_table_5x8[95][5]; // Bit Map for ASCII table (ASCII_Font.c)
extern const unsigned char ascii_table_8x16[95][16];
////////////Used for DMA mode///////////////////////
extern void WriteImageToDriverLCD(void *bufferPtr);
uint8_t *LCD_IMAGE_Send, *LCD_IMAGE_Write;
volatile bool Write_ready = 1;
extern _Bool HomePage;
extern uint8_t id_Page0, idModeRunning, idPage0Display[8];
//////////////////////////////////////////////////
extern void ReadTxFiFO(void);
extern void WriteTxFiFO(uint8_t c);
extern volatile uint8_t pagelcd;
uint16_t counttest, countcup = 1102; // just used for test
void InsertToIdMsg(uint8_t);
void DeleteToIdMsg(uint8_t);
// ---------------------------- ADS1118 Temperature Sensor ------------------------------------
ADS1118_t Steam, Hot_Water;
void Spi1_ADS1118_Interface_Cnf(void); // Configurate Spi for communicate ADS1118
extern void SSI1_IntHandler(void);
void ADS1118_Cal(ADS1118_t *ADS); // Calculate temperature
extern float temp;
volatile uint8_t datacount = 0;
uint32_t datas, fb_config;
#define MEM_BUFFER_SIZE         2
extern uint16_t ui16TxBuf[];
extern uint16_t ui16RxBuf[];
uint16_t volatile Settingconfig;
// ---------------------------- TCA9539 IO Expander Module --------------------------------------
TCA9539Regs TCA9539_IC1, TCA9539_IC2, TCA9539_IC3;
TCA9539Regs *TCA9539_IC[3] = { &TCA9539_IC1, &TCA9539_IC2, &TCA9539_IC3 };
extern void I2C0_TCA9539_Configuration(void);
extern void I2C0_TCA9539_IterruptTrigger_Cnf(); // Configure external interrupt (trigger isr TCA) - PB1
extern volatile uint8_t Tx_slavecount, Tx_usedBrust;
extern volatile uint8_t Rx_slavecount, Rx_usedBrust;
// ------------------------------------- Flow Meter ---------------------------------------------
float Calibration; // Coeficient for calculate vollume pump
volatile float totalMilliLitres, MilliLitresBuffer;
float SetVolume;
volatile bool FinishPumpEvent = false;
void QEP_CoffeeMachine_cnf(void);
void FlowMeterCal(void);
void InitPumpingEvent(void);

extern void SteamLevelControl_Run(void *PrPtr);
extern void SteamLevelControl_Stop(void *PrPtr);

void QEP_VelGrind_Cf(void);
// -----------------------------------Peripheral Clock define -----------------------------------
#define I2C0
#define SPI0 #define SPI1
#define QEI0
#define All_Of_GPIO
// -------------------------------------Driver BLDC Motor ---------------------------------------
// Configure 3 pwm chanel for 3 BLDC motor
void PWMDRV_Coffee_machine_cnf(void);
// -------------------------------- ADC DMA function define ---------------------------------------
void ADC_Cfg(void);
void ADC_READ(void);
extern float fADC_LevelSensor;
extern volatile bool LevelControlTriger, InLevelPumping, Hyteresis;
extern bool SteamReady;
extern void SteamLevelControl_Run(void *PrPtr);
extern void SteamLevelControl_Stop(void *PrPtr);
extern void WarmingPressMachine(void *P);
extern _Bool PWMSSR1Enable, PWMSSR2Enable, PWMSSR3Enable;
// -------------------------------------- Group Task --------------------------------------------
void (*A_Group_Task)(void); // 2ms Task
void (*B_Group_Task)(void); // 5ms Task
void (*C_Group_Task)(void); // 100ms Task
void (*D_Group_Task)(void); // 500ns Task

uint32_t Vr_C2Task;
void A_Base(void);
void B_Base(void);  // Monitor machine
void C_Base(void);
void D_Base(void);

void A1(void);
void A2(void);

void B1(void);
void B2(void);

void C1(void);
void C2(void);

void D1(void);

uint16_t parameter[10] = { };
extern void MakeCoffee(void);

extern void StartUpMachine(void);
extern void MakeCoffeProcess(void);
extern void StartUpProcess(void);
extern void WarmingPressProcess(void);
extern void CleanningMachine(void);
extern void CleanProcess(void);
extern bool clModeRinse;
extern bool InProcess, InCleanning, InStartUp, InWarming, calibVolumeFlag;
bool idleMachine, fullOfGroundsDrawer, Suf_HotWater, FinishStartUp,
        HeatingPress, Error, Id_Msg_flag;
uint8_t countGrounds;
bool InprocessStorage;
extern volatile uint8_t step;
uint32_t duty = 100;

void HomeReturn_Process_Run(void *PrPtr);
uint16_t speedtest = 2000;
uint8_t flag_test;
float m = 0.9;
extern void InitFeedbackVel(void);
volatile _Bool En, EnMakeCoffee;
volatile bool firstCup = true;
uint32_t sp = 2000, dd, ss = 0;
void main(int argc, char **argv)
{
    // Initialize Device/board include:
    // + Disable Wdog timer, Disable interrupt
    // + System clk, Enable peripheral clock
    // + Initialize timers
    // + GPIO init
    IntMasterDisable();
    InitSysClt();
    TimerSysClt();

#ifndef SysCltEeprom
    SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_EEPROM0))
        ;

#endif
    EEPROMInitStastus = EEPROMInit();
    // Recovery Eeprom
#ifdef DEBUG
    if(EEPROMInitStastus != EEPROM_INIT_OK){
        printf("Error initalize Eeprom");
        while(1);
    }

#endif
    Ptr_Task = &A_Base; // Initalize master task poinnter
    A_Group_Task = &A1;
    B_Group_Task = &B1;
    C_Group_Task = &C1;
    D_Group_Task = &D1;
    clockrate = SysCtlClockGet();

// ---------------------------------- USER ----------------------------------------
//=================================================================================

    Steam.Code = ADSCON_CH0; // Assign code channel 0 ADS1118 temperature sensor
    Steam.Actual_temperature = 0;
    //Temperature Control terminal assign - Steam
    CNTL_2P2Z_DBUFF_t Default = { 0, 0, 0, 0, 0 };
    Steam_CNTL.DBUFF = Default;     // Internal buffer
    Steam_CNTL.Ref = &Steam_Temperature_Ref; //Desired temperature(reference signal)
    Steam_CNTL.Fdbk = &Steam.Actual_temperature; // Acutal temperature(feedback signal)
    Steam_CNTL.Out = &Steam_Vout;   // Control output
    Dmax_Steam = Dmax_HotWater = 100;
    Pgain_Steam = 500;
    Igain_Steam = 100;
    CNTL_Pole_Zero_Cal(&Steam_CNTL, Pgain_Steam, Igain_Steam, Dgain_Steam,
                       Dmax_Steam, -10, 0);

    Hot_Water.Code = ADSCON_CH1; // Assign code channel 0 ADS1118 temperature sensor
    Hot_Water.Actual_temperature = 0;
    //Temperature Control terminal assign - Hot water tank
    HotWater_CNTL.DBUFF = Default; // Internal buffer
    HotWater_CNTL.Ref = &HotWater_Temperature_Ref; //Desired temperature(reference signal)
    HotWater_CNTL.Fdbk = &Hot_Water.Actual_temperature; // Acutal temperature(feedback signal)
    HotWater_CNTL.Out = &HotWater_Vout; // Contol output

    Pgain_HotWater = 800;
    Igain_HotWater = 950;
    Dgain_HotWater = 200;
    CNTL_Pole_Zero_Cal(&HotWater_CNTL, Pgain_HotWater, Igain_HotWater,
                       Dgain_HotWater, Dmax_HotWater, -10, 0);

//=================================================================================
//  Configure PWMs for coffe machine
    PWMDRV_Coffee_machine_cnf();
//=================================================================================
//  ADS1118 - Termperature Sensor Configuration - 2 channel
//=================================================================================
    Spi1_ADS1118_Interface_Cnf();   // Configurate spi1
    ADS_Config(0);  // Dummy config (Notused)
//=================================================================================
//  TCA9539 - I/O Expander Configuration - 2 channel
//=================================================================================
    TCA9539_IC1._Id = Address_IC1; // Address Slave IC1
    TCA9539_IC1.TCA9539_Onput.all = InitalOutput_IC1;
    TCA9539_IC1.TCA9539_Config.all = InitalConfig_IC1;

    TCA9539_IC1.updateOutputFlag = 1;
    TCA9539_IC1.ReadCmdFlag = 1;
// ---------------------------------------------------------------------------------
    TCA9539_IC2._Id = Address_IC2; // Address Slave IC2
    TCA9539_IC2.TCA9539_Onput.all = InitalOutput_IC2;
    TCA9539_IC2.TCA9539_Config.all = InitalConfig_IC2;

    TCA9539_IC2.updateOutputFlag = 1;
    TCA9539_IC2.ReadCmdFlag = 1;
// ---------------------------------------------------------------------------------
    TCA9539_IC3._Id = Address_IC3; // Address Slave IC3
    TCA9539_IC3.TCA9539_Onput.all = InitalOutput_IC3;
    TCA9539_IC3.TCA9539_Config.all = InitalConfig_IC3;

    TCA9539_IC3.updateOutputFlag = 1;
    TCA9539_IC3.ReadCmdFlag = 1;

// ---------------------------------------------------------------------------------
#if(BOARD == 1)
    // Configure I2C interface for Communicate with TCA9539
    I2C0_TCA9539_Configuration();
    // Configure GPIO interrupt to respone intertupt signal of TCA9539
    I2C0_TCA9539_IterruptTrigger_Cnf(); // GPIO interrupt PB1
#endif

// ------------------------------  Configure QEI ----------------------------------

    QEP_CoffeeMachine_cnf();
    QEP_VelGrind_Cf();
    // InitPumpingEvent();
    Calibration = (float) 2 / 15.0;
//=================================================================================
//  INITIALISATION - LCD-Display connections
//=================================================================================
    LCD_Interface_Cnf(); // SPI & I/O configruation
    LCD_ST7567_Init();   // LCD initialize

    IntMasterEnable();
    LCD_Disp_Clr(0x00);
    // IntMasterDisable();
    // Initialize GUI interface
    SerialCommsInit();

    VrTimer1[3] = 15;
    // Assign data stream to Gui variable display LCD - Display on Page 0 LCD
    dataSentList[cupsEspresso_1] = &Mode_Espresso_1.Cups;   //&SumOfCupInUsed;
    dataSentList[cupsEspresso_2] = &Mode_Espresso_2.Cups;

    dataSentList[cupsSpecial_1] = &Mode_Special_1.Cups;

    dataSentList[cupsSpecial_2] = &Mode_Special_2.Cups;
    dataSentList[BladeNofTimesUsed] = &Blade;
    dataSentList[RonNofTimesUsed] = &Ron;

    dataSentList[HotWaterTemp] = (uint16_t*) &Gui_HotWaterSteam;
    dataSentList[ExtractionTime] = (uint16_t*) &Gui_CoffeExtractionTime;

    dataSentList[tempExtrude] = (uint16_t*) &MilliLitresBuffer;

    //"Set" variables
    //---------------------------------------
    // Assign GUI parameter  to desired  parameter setting addresses(GUI Parameter)
    Pr_Packet[0] = &Mode_Espresso_1.PreInfusion; //     Preinfution duration
    Pr_Packet[1] = &Mode_Espresso_1.AmountOfWaterPumping.stage_1; //volume
    Pr_Packet[2] = &Mode_Espresso_1.WeigtOfPowder;  // Not used
    Pr_Packet[3] = (uint32_t*) &Mode_Espresso_1.GrindingDuration; // //time for grind
    Pr_Packet[4] = (uint32_t*) &HotWater_Temperature_Ref;   // Gobal variable
    Pr_Packet[5] = &Mode_Espresso_1.Pitch;

    // Pr_Packet[3] = &Mode_Espresso_1.AmountOfWaterPumping.stage_2;

    Pr_Packet[8] = &Mode_Espresso_2.PreInfusion;
    Pr_Packet[9] = &Mode_Espresso_2.AmountOfWaterPumping.stage_1;
    Pr_Packet[10] = &Mode_Espresso_2.WeigtOfPowder; // Not used
    Pr_Packet[11] = (uint32_t*) &Mode_Espresso_2.GrindingDuration;

    //   Pr_Packet[12] = &Mode_Espresso_2.AmountOfWaterPumping.stage_1;

    Pr_Packet[12] = (uint32_t*) &HotWater_Temperature_Ref;

    Pr_Packet[13] = &Mode_Espresso_2.Pitch;

    Pr_Packet[16] = &Mode_Special_1.PreInfusion;
    Pr_Packet[17] = &Mode_Special_1.AmountOfWaterPumping.stage_1;
    Pr_Packet[18] = &Mode_Special_1.WeigtOfPowder;
    Pr_Packet[19] = (uint32_t*) &Mode_Special_1.GrindingDuration;

    //  Pr_Packet[20] = &Mode_Special_1.AmountOfWaterPumping.stage_1;

    Pr_Packet[20] = (uint32_t*) &HotWater_Temperature_Ref;
    Pr_Packet[21] = &Mode_Special_1.Pitch;

    Pr_Packet[24] = &Mode_Special_2.PreInfusion;
    Pr_Packet[25] = &Mode_Special_2.AmountOfWaterPumping.stage_1;
    Pr_Packet[26] = &Mode_Special_2.WeigtOfPowder;
    Pr_Packet[27] = (uint32_t*) &Mode_Special_2.GrindingDuration;

    // Pr_Packet[28] = &Mode_Special_2.AmountOfWaterPumping.stage_1;

    Pr_Packet[28] = (uint32_t*) &HotWater_Temperature_Ref;
    Pr_Packet[29] = &Mode_Special_2.Pitch;
    Pr_Packet[31] = (uint32_t*) &PulWeightRatio;
    // Assign direction motor of grind module

    Mode_Espresso_1.DirGrinding = Mode_Espresso_2.DirGrinding = false;
    Mode_Special_1.DirGrinding = Mode_Special_2.DirGrinding = false; // true

//============================= Configure machine Parameter ========================================
    ParameterDefaultSetting();

    // Read EEPROM memory
    uint32_t tempt32DataRead[32];

#ifdef SaveEeprom
    uint8_t i = 0;
    EEPROMRead(tempt32DataRead, AddDataEeprom, 32 * 4);
    // Attract 32 bit packet to 16 bit packet and initalize to system parameter;
    uint32_t *ui32Ptr = (uint32_t*) tempt32DataRead;
    for (i = 0; i < 32; i++)
    {
        *Pr_Packet[i] = ui32Ptr[i];
    }

#endif
    for (i = 0; i < 8; i++)
        idPage0Display[i] = 0xFF;
//===================================ADC DMA Config================================================
    ADC_Cfg();

//===================================Enable And start System ======================================
    TimerEnable(TIMER0_BASE, TIMER_A);
    TimerEnable(TIMER1_BASE, TIMER_A);
    TimerEnable(TIMER2_BASE, TIMER_A);
    TimerEnable(TIMER3_BASE, TIMER_A);
    TimerEnable(TIMER4_BASE, TIMER_A);
    TimerEnable(TIMER5_BASE, TIMER_A);
    TimerEnable(WTIMER0_BASE, TIMER_BOTH);

//Clear interrupt Flag & enable interrupt (CPU level)
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT); // Temperature control
    TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT); // Temperature control
    TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT); // Timming Process
    IntMasterEnable();
#if(BOARD == 1)
    TCA9539Init(&TCA9539_IC3);
    TCA9539Init(&TCA9539_IC2);
    TCA9539Init(&TCA9539_IC1);
#endif
    // Enable Control temerature of steam and Hotwater tank
    PWMSSR1Enable = 1;
    PWMSSR2Enable = 1;
    PWMSSR3Enable = 1;

    InitFeedbackVel();
    StartUpMachine();
    //  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 100000);
//    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 5000);
    QEP_VelGrind_Cf();
    Error = 0;
    FinishStartUp = 1;
    while (1)
    {

        Ptr_Task(); // Swept periodic tasks
        Cmd_ReadMsg(); //Execute making coffee Machine

    }
}
//===================================================================================================
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
// Task 50ms
void C_Base(void)
{

    if (TimerIntStatus(TIMER2_BASE, false) == TIMER_TIMA_TIMEOUT)
    {

        TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
        C_Group_Task();
    }
    Ptr_Task = &D_Base;
}

void D_Base(void)
{

    if (TimerIntStatus(WTIMER0_BASE, false) == TIMER_TIMA_TIMEOUT)
    {

        TimerIntClear(WTIMER0_BASE, TIMER_TIMA_TIMEOUT);
        D_Group_Task();
    }
    Ptr_Task = &A_Base;
}
//====================================== Periodic Task ===============================================
// Task 2ms -  display LCD & receive cmd from button LCD
void A1(void)
{
// Communicate and display LCD
    SerialHostComms();
    A_Group_Task = &A2;

}
// Task 2ms - Read input from extend GPIO ic TC9539
void A2(void)
{
#if(BOARD == 1)
// When signal change TCA595 wil generate interrupt and  ISR will set ReadCmdFlag
    if (TCA9539_IC1.ReadCmdFlag && !I2CMasterBusBusy(I2C0_BASE))
        Cmd_WriteMsg((void (*)(void*)) TCA9539ReadInputReg,
                     (void*) &TCA9539_IC1);

    if (TCA9539_IC2.ReadCmdFlag && !I2CMasterBusBusy(I2C0_BASE))
        Cmd_WriteMsg((void (*)(void*)) TCA9539ReadInputReg,
                     (void*) &TCA9539_IC2);

    if (TCA9539_IC3.ReadCmdFlag && !I2CMasterBusBusy(I2C0_BASE))
        Cmd_WriteMsg((void (*)(void*)) TCA9539ReadInputReg,
                     (void*) &TCA9539_IC3);

    if (TCA9539_IC1.updateOutputFlag && !I2CMasterBusBusy(I2C0_BASE))
        Cmd_WriteMsg((void (*)(void*)) TCA9539WriteOutput,
                     (void*) &TCA9539_IC1);

    if (TCA9539_IC2.updateOutputFlag && !I2CMasterBusBusy(I2C0_BASE))
        Cmd_WriteMsg((void (*)(void*)) TCA9539WriteOutput,
                     (void*) &TCA9539_IC2);

    if (TCA9539_IC3.updateOutputFlag && !I2CMasterBusBusy(I2C0_BASE))
        Cmd_WriteMsg((void (*)(void*)) TCA9539WriteOutput,
                     (void*) &TCA9539_IC3);
#endif
    A_Group_Task = &A1;
}
// Task 5ms - Update output - Make coffee
void B1(void)
{

// Hardware limit switch - FORCE output low
    uint16_t LSW = TCA9539_IC3.TCA9539_Input.all & LinitSwitch;
    uint16_t RSW = TCA9539_IC3.TCA9539_Input.all & LevelSensor1;
    uint16_t Dir = TCA9539_IC2.TCA9539_Onput.all & Direction_BLDC2;
    uint16_t Duty = PWMPulseWidthGet(PWM0_BASE, PWM_OUT_1);
    if (Duty > 0)
    {
        if ((LSW == 0) && (Dir == 0))
        {

            TCA9539_IC2.TCA9539_Onput.all = ((TCA9539_IC2.TCA9539_Onput.all
                    & ~(Direction_BLDC2)) | Direction_BLDC2);
            PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, false);

        }
        if ((RSW == 0) && (Dir != 0))
        {
            TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                    & ~(Direction_BLDC2));
            PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, false);

        }
    }

// Scan to output TCA
    if (InProcess)
        B_Group_Task = &MakeCoffeProcess;
    else if (InCleanning)
        B_Group_Task = &CleanProcess;
    else if (InStartUp)
        B_Group_Task = &StartUpProcess;
    else if (InWarming)
        B_Group_Task = &WarmingPressProcess;
    else
        B_Group_Task = &B2;
}
void B2(void)
{
#if(Grindmotor == scooter)

    if(InProcess)
        InprocessStorage = 1;
    if(InProcess == 0 && InprocessStorage == 1)
    {
        InprocessStorage = 0;
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 100000);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 5000);   //8500
    }
#else
    if ((TCA9539_IC2.TCA9539_Input.all & (Enable_BLDC2 | Enable_BLDC1)) == 0)
        PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    else
        PWMGenDisable(PWM0_BASE, PWM_GEN_0);
#endif

    B_Group_Task = &B1;
}
// Task 50ms - Button GUI
void C1(void)
{

    static uint8_t release_mode = 1, release = 0, release_CleanB = 1;
    static uint32_t Vrtimer;
// ---------------------------------------- Button cofffee maker & GUI ------------------------------//
// Select mode and make coffe
    if ((release_mode == 1) && (EnMakeCoffee == 1))
    {
        if ((TCA9539_IC1.TCA9539_Input.all & Special1_Bt) == 0)
        {
            ModeSelected = &Mode_Special_1;
            // TCA9539_IC1.TCA9539_Onput.all &= LED_BT5;
            TCA9539_IC1.TCA9539_Onput.all |= LED_BT5;
            idModeRunning = 7;
            test_step = 1;
            step = 2;
            MakeCoffee();
            release_mode = 0;

        }
        else if ((TCA9539_IC1.TCA9539_Input.all & Special2_Bt) == 0)
        {
            ModeSelected = &Mode_Special_2;
            //   TCA9539_IC1.TCA9539_Onput.all &= LED_BT7;
            TCA9539_IC1.TCA9539_Onput.all |= LED_BT7;
            idModeRunning = 8;
            test_step = 1;
            step = 2;
            MakeCoffee();
            release_mode = 0;
        }
        else if ((TCA9539_IC1.TCA9539_Input.all & Expresso1_Bt) == 0)
        {
            ModeSelected = &Mode_Espresso_1;
            //     TCA9539_IC1.TCA9539_Onput.all &= LED_BT4;
            TCA9539_IC1.TCA9539_Onput.all |= LED_BT4;
            idModeRunning = 5;
            MakeCoffee();
            release_mode = 0;
        }
        else if ((TCA9539_IC1.TCA9539_Input.all & Expresso2_Bt) == 0)
        {
            ModeSelected = &Mode_Espresso_2;
            //       TCA9539_IC1.TCA9539_Onput.all &= LED_BT6;
            TCA9539_IC1.TCA9539_Onput.all |= LED_BT6;
            idModeRunning = 6;
            MakeCoffee();
            release_mode = 0;
        }
    }
// ---------------------------------------- Button Cleanning & GUI ------------------------------//
    if ((release_CleanB == 1) && (EnMakeCoffee == 1))
    {
        // Cleanning complete machine
        if ((TCA9539_IC1.TCA9539_Input.all & BT9) == 0)
        {
            TCA9539_IC1.TCA9539_Onput.all |= LED_BT9;
            idModeRunning = 2;
            CleanningMachine();
            clModeRinse = 0;
            release_CleanB = 0;

        }
        // Rinse
        if ((TCA9539_IC1.TCA9539_Input.all & BT11) == 0)
        {
            idModeRunning = 1;
            CleanningMachine();
            clModeRinse = 1;
            release_CleanB = 0;
        }

    }

//-------------------------------------------- Button cancel & GUI-------------------------------
    if ((idleMachine == 0) && (release == 1) && (cancel_cmd == 0))
    {
        if ((TCA9539_IC1.TCA9539_Input.all & Cancel_Task) == 0)
        {

            cancel_cmd = 1;
            release = 0;
            TCA9539_IC1.TCA9539_Onput.all |= LED_BT8;   // BT8
        }
    }

//--------------------------------Release Button left Pannel-------------------------------------
    if ((TCA9539_IC1.TCA9539_Input.all & 0x78) == 0x78)
    {   // all button make coffe release
        release_mode = 1;
        if (InProcess == 0)
        {

            TCA9539_IC1.TCA9539_Onput.all &= ~(LED_BT4 | LED_BT5 | LED_BT6
                    | LED_BT7);
        }
        Vrtimer = 0;
    }
    else
        Vrtimer++;
    if ((Vrtimer > 25) && (calibVolumeFlag == 0) && (InProcess == 1))
    {
        calibVolumeFlag = 1;
        Vrtimer = 0;
    }

//--------------------------------Release Button Right Pannel-------------------------------------
    // Button cancel release
    if (((TCA9539_IC1.TCA9539_Input.all & Cancel_Task) != 0)
            && (idleMachine == 0))
    {
        release = 1;
        TCA9539_IC1.TCA9539_Onput.all &= ~(LED_BT8 | LED_BT9);
    }
    // Button clean release
    if (((TCA9539_IC1.TCA9539_Input.all & (BT9 | BT11)) != 0x84)
            && (InCleanning == 0)) // Button cancel release
    {
        release_CleanB = 1;
        TCA9539_IC1.TCA9539_Onput.all &= ~(LED_BT8 | LED_BT9);
    }

    C_Group_Task = &C2;
}
// Task 50ms - Control level
void C2(void)
{

    ADC_READ();
    if (En)
        (Vr_C2Task >= 300) ?
                (Cmd_WriteMsg(WarmingPressMachine, NULL), Vr_C2Task = 0) :
                (Vr_C2Task++); //Cmd_WriteMsg(WarmingPressMachine, NULL)

// Start pumping water to steam tank
    if (fADC_LevelSensor > (float) 2.2 && (InLevelPumping == 0))
    {
        LevelControlTriger = InLevelPumping = 1;
        Cmd_WriteMsg(SteamLevelControl_Run, NULL);
        Hyteresis = 1;
    }
    // Stop pumping water to steam tank
    else if ((fADC_LevelSensor < (float) 1.1) && (InLevelPumping == 1)
            && Hyteresis)
    {
        Hyteresis = 0;
        Cmd_WriteMsg(SteamLevelControl_Stop, NULL);
    }
    if ((fADC_LevelSensor >= (float) 0.85) && (fADC_LevelSensor <= (float) 2.2))
    {
        if (Gui_TempSteam >= 110)
        {
            (CountSteam >= 1200) ? (SteamReady = 1) : CountSteam++;

        }
        else
        {
            CountSteam = 0;
            SteamReady = 0;
        }

    }
    else
        SteamReady = 0;

    if (coefSteam_change)
    {
        CNTL_Pole_Zero_Cal(&Steam_CNTL, Pgain_Steam, Igain_Steam, Dgain_Steam,
                           7999, -10, 0);
        coefSteam_change = 0;
    }
    if (coefHotWater_change)
    {
        CNTL_Pole_Zero_Cal(&HotWater_CNTL, Pgain_HotWater, Igain_HotWater,
                           Dgain_HotWater, Dmax_HotWater, -10, 0);
        coefHotWater_change = 0;
    }
    if (Hot_Water.Actual_temperature >= (HotWater_Temperature_Ref - 4))
    {
        HotWater_CNTL.Coef.a1 = m;
    }

    else
    {
        HotWater_CNTL.Coef.a1 = 1;
    }

    C_Group_Task = &C1;
}
// Monitor machine
void D1(void)
{

    if (countGrounds >= 50)
        fullOfGroundsDrawer = 1;
    if (Gui_HotWaterSteam >= 20)
        Suf_HotWater = 1;
    idleMachine = !(InProcess || InStartUp || InCleanning);
    FinishStartUp = Suf_HotWater && HeatingPress;
    En = ((idleMachine) && FinishStartUp && (!fullOfGroundsDrawer) && (!Error));
    EnMakeCoffee = En && HomePage;
//================================ Message id to lcd =============================================
    Id_Msg_flag = fullOfGroundsDrawer || !idleMachine || Error || En;


    (!idleMachine && FinishStartUp) ? (idPage0Display[0] = idModeRunning) : (idPage0Display[0] = 0xFF);
//---------------------------------------------------------------------------------
    (fullOfGroundsDrawer && idleMachine) ?
            (idPage0Display[1] = 10) : (idPage0Display[1] = 0xFF);
//---------------------------------------------------------------------------------
    (Error) ? (idPage0Display[2] = 12) : (idPage0Display[2] = 0xFF);
//---------------------------------------------------------------------------------
    if (En)
    {
        (firstCup) ? (idPage0Display[3] = 0) : (idPage0Display[3] = 0xFE);

    }
    else if(!FinishStartUp)
        idPage0Display[3] = 9;
    else
        idPage0Display[3] = 0xFF;

    D_Group_Task = &D1;
}

//======================================================================================================

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
    // SSIIntEnable(SSI0_BASE, SSI_TXFF);
    SSIIntEnable(SSI0_BASE, SSI_DMATX);
    IntEnable(INT_UDMA);    // sw
    IntEnable(INT_SSI0);
    uDMAEnable();
    Init_LCDSPI_DMA();
    Init_SW_DMA();

#endif

//SSILoopbackEnable(SSI0_BASE);

}
void LCD_Interface_Cnf()
{
    Spi0_LCD_Interface_Cnf();   // Used interrupt method for transmit
// IntEnable(INT_SSI0);
// Configurate Pin for proper ssi
    GPIOPinTypeSSI(GPIO_PORTA_BASE,
    GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
    SSIEnable(SSI0_BASE);

// Configurate Pin for other
    /* GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_STRENGTH_12MA,
     GPIO_PIN_TYPE_STD_WPU);*/
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
void Disp_Str_5x8(volatile uint8_t page, volatile uint8_t column, uint8_t *text)
{

    uint8_t i = 0, j, k;
// static uint16_t index = 0;
// index = (page - 1) * 128 + (column - 1);

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
            (j == 0) ? (column += 5) : (column += 6);
#endif
#ifdef uDma_SSI0

            for (k = 0; k < 5; k++)
            {
                LCD_IMAGE_Write[index] = (uint8_t) ascii_table_5x8[j][k];
                index++;
                if (index >= 1024)
                    index = 0;
            }
            i++;
            column += 5;

#endif
        }
        else
            i++;

    }

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

            //  LCD_Address_Set(page, column);
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
void Disp_Str_8x16(uint8_t page, uint8_t column, uint8_t *text)
{
    uint8_t i = 0, j, k, n;

    while (text[i] > 0x00)
    {
        if ((text[i] >= 0x20) && (text[i] <= 0x7E))  //ASii��Χ
        {
            j = text[i] - 0x20;
            for (n = 0; n < 2; n++)
            {
                LCD_Address_Set(page + n, column);
                for (k = 0; k < 8; k++)
                {
                    LCD_Write_Dat(ascii_table_8x16[j][k + 8 * n]);
                }
            }
            i++;
            column += 8;
        }
        else
        {
            i++;
        }
    }
}
void Disp_Str_8x16_Image(uint8_t page, uint8_t column, uint8_t *text)
{
    uint8_t i = 0, j, k, n;
    page = page - 1;
    column = column - 1;
    while (text[i] > 0x00)
    {
        if ((text[i] >= 0x20) && (text[i] <= 0x7E))  //ASii��Χ
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
    ;
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
    /*       if (mode == 1)
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
// Confige Mode 0 SSI, Freq = 100Khz, 8Bit
    SSIConfigSetExpClk(SSI1_BASE, 80000000, SSI_FRF_MOTO_MODE_1,
    SSI_MODE_MASTER,
                       250000, 16);
    GPIOPinTypeSSI(GPIO_PORTF_BASE,
    GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPadConfigSet(GPIO_PORTF_BASE,
    GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
                     GPIO_STRENGTH_8MA,
                     GPIO_PIN_TYPE_STD);

    SSIIntRegister(SSI1_BASE, &SSI1_IntHandler);
    SSIIntEnable(SSI1_BASE, SSI_DMATX);
    SSIIntEnable(SSI1_BASE, SSI_DMARX);
    IntEnable(INT_SSI1); //
    uDMAEnable();
    Init_SPI_DMA();
    SSIEnable(SSI1_BASE);

}

void defaultISR(void)
{

}

