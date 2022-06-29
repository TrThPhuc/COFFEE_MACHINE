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
#include "DCL_PID.h"
#include "TCA9539_hw_memmap.h"

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//////////////////////////////////PROTOTYPES///////////////////////////////////////////////
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// SYSTEM FUNCTION
//---------------------------------------------------------------
#define Null 0
extern void InitSysClt(void);      // Initialize system & peripheral clock
void defaultISR(void);      // Default interrupt handler
void GpioConfigure(void);   //
void TimerSysClt(void);
void (*Ptr_Task)(void);     // Pointer task
extern void Cmd_ReadMsg(void);
extern void Cmd_WriteMsg(void (*pFun)(void*), void *pArg);
void WatchdogInit(void);
void WatchdogIntHandler(void);
void SweptErrorMachine(void);
void SweptWarningMachine(void);
void ClearError(void);
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
#define RINSEMODE   1
#define CLEANMODE   0
#define  ModeInterpreter( _Mode,  _Led, _IdMSg, _maskCompress)                           \
do                                                                                      \
{                                                                                       \
    uint8_t frameOfEachMode = 8, grindObjIndex = 5;                                     \
    ModeSelected = _Mode;                                                               \
ledBlink = _Led;                                                                        \
idModeRunning = _IdMSg;                                                                 \
mode_str = true;                                                                        \
release_mode = 0;                                                                       \
calibWeightObj = (ModeSelected->IndexMode) * frameOfEachMode + grindObjIndex;           \
MaskExtraCompress = _maskCompress;                                                      \
}while(0)

#define CleanInterpreter(_isRinseMode, _Led, _IdMSg)                \
do                                                                  \
{                                                                   \
    clModeRinse = _isRinseMode;                                     \
    ledBlink = _Led;                                                \
    idModeRunning = _IdMSg;                                         \
    release_CleanB = 0;                                             \
}while(0)

// Constant string
char BrandName[16] = "STAR X";
char ModelName[24] = "Special Auto";
char ProductDateStr[16] = "12/03/2022";
uint8_t SeriNum = 1;
uint32_t clockrate; // System clock
uint32_t EEPROMInitStastus;
volatile _Bool feedWacthdog = true;
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
uint8_t coefSteam_change, coefHotWater_change;
float Steam_Temperature_Ref, Steam_Vout;
float HotWater_Temperature_Ref, HotWater_Vout;
float Extrude_Vout = 40;
float Gui_TempSteam, Gui_HotWaterSteam, CountSteam;
float Gui_CoffeeExtractionTime, Gui_CoffeePreinfusionTime, Gui_CoffeeGrindTime;
extern _Bool ErHotWater;
CNTL_2P2Z_Terminal_t Steam_CNTL, HotWater_CNTL;
CNTL_PID_DCL_Terminal_t HotWater_PID_DCL;

uint16_t Process_status;
uint16_t SumOfCupInUsed = 0;
uint16_t SumOfCupInUsed_day = 0;
uint16_t BladeA, BladeB, Ron;
uint32_t ExtractA, ExtractB;    //Used times of Blade and Ron
uint32_t wBladeATimes, wBladeBTimes, wExtractionATimes, wExtractionBTimes;
uint32_t wExtract_MaxTime, wExtract_MinTime;
uint16_t *CountDataStorage[8];
// Mode parameter
Mode_Parameter_t Mode_Espresso_1, Mode_Espresso_2, Mode_Special_1,
        Mode_Special_2;
Mode_Parameter_t *ModeSelected;

bool cancel_cmd, test_step = 0;    // cancel command
extern bool MaskExtraCompress;
// ---------------------------------- LCD Interface -----------------------------------------
// Reset LCD - PA6
// Initialize and configure LCD through SPI interface
extern void LCD_Interface_Cnf(void);   //  Configure GPIO and interface for LCD
extern void LCD_ST7567_Init(void);     // Initial Config  LCD
extern void Init_SW_DMA(void);
extern void Init_SPI_DMA(void);
extern void Init_LCDSPI_DMA(void);
//Kernel for Communicate Host to LCD
extern void SerialCommsInit(void); // Initialize task
extern void SerialHostComms(void); // Task proceessed in period
//Variable
uint16_t *dataSentList[24]; // Terminal connect to monitor variable
uint32_t *Pr_Packet[NumberOfParameter];    // Terminal connect to Parameter
uint8_t coeff_change;   // Flag for change parameter in mode /////////
int16_t VrTimer1[8];    // Virtual timer
////////////Used for DMA mode///////////////////////
uint8_t *LCD_IMAGE_Send, *LCD_IMAGE_Write;
extern _Bool HomePage;
extern uint8_t idModeRunning, idPage0Display[8];
extern volatile uint8_t pagelcd;

//////////////////////////////////////////////////
uint16_t counttest, countcup = 1102; // just used for test
uint8_t VrHoldMsg;
_Bool HoldMsgFlag, HoldCmd, PreReadLcdBt = 0;
// ---------------------------- ADS1118 Temperature Sensor & Heating ------------------------------------
ADS1118_t Steam, Hot_Water;
void Spi1_ADS1118_Interface_Cnf(void); // Configurate Spi for communicate ADS1118
extern void SSI1_IntHandler(void);
void ADS1118_Cal(ADS1118_t *ADS); // Calculate temperature
extern float Group_Temp;
uint32_t wGroupDuty, wGroupShutdownDuty, wGroupTempSet;
volatile uint8_t datacount = 0;
uint32_t datas, fb_config;
#define MEM_BUFFER_SIZE         2
extern uint16_t ui16TxBuf[];
extern uint16_t ui16RxBuf[];
uint16_t volatile Settingconfig;
extern _Bool HeatingHotwater, HeatingSteam;
extern _Bool ErHotWaterTimeOut, ErSteamTimeOut, ErOutletDetect, ErPumpSteam;

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
#ifdef VelGrindDebug
extern void QEP_VelGrind_Cf(void);
extern void InitFeedbackVel(void);
#endif
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
extern float fADC_LevelSensor, fADC_TempWarming;
extern volatile bool LevelControlTriger, InLevelPumping, Hyteresis;
extern bool SteamReady;
extern void SteamLevelControl_Run(void *PrPtr);
extern void SteamLevelControl_Stop(void *PrPtr);
#if(WarmingMethod == SteamWarming)
extern void WarmingPressMachine(void *P);
extern void WarmingPressProcess(void);
#endif
extern _Bool PWMSSR1Enable, PWMSSR2Enable, PWMSSR3Enable;
// -------------------------------------- Group Task --------------------------------------------
void (*A_Group_Task)(void); // 2ms Task
void (*B_Group_Task)(void); // 5ms Task
void (*C_Group_Task)(void); // 100ms Task
void (*D_Group_Task)(void); // 500ns Task

uint16_t Vr_C2Task, Vr_ErrorClear;
uint16_t ledBlink = 0, VrBlinkLed, BtModeStr;
void A_Base(void);
void B_Base(void);  // Monitor machine
void C_Base(void);
void D_Base(void);

void A1(void);
void A2(void);

void B1(void);
void B2(void);
static inline uint32_t ScanTaskMachine(void);

void C1(void);
void C2(void);

void D1(void);
void BlinkModeLed(void);

uint16_t parameter[10] = { };
extern void MakeCoffee(void);

extern void WarmUpMachine(void);
extern void WarmUpProcess(void);
extern void MakeCoffeProcess(void);

extern void CleanningMachine(void);
extern void CleanProcess(void);

extern void WarmingPressMachine(void);
extern void WarmingPressProcess(void);

extern inline void HoldMotorOff(void);
extern void CheckFinsih_ModuleTest(void);
extern void SaveCountToEeprom(void);
extern ModuleTestMachine(void);
extern void ManualControlSelect(void);
extern void ManualControlTest(void);
_Bool SaveDefautCount = 0;
extern bool clModeRinse;
extern bool InProcess, InCleanning, InStartUp, InWarming, calibWeightFlag,
        calibWeightStr, InManualTest;
extern bool InModuleTest;
extern uint8_t calibWeightObj;
bool idleMachine, p_idleMachine, fullOfGroundsDrawer, Suf_HotWater,
        FinishStartUp, Error, WarningFlag, Id_Msg_flag;
extern uint8_t blinkCurVr, ObjSelectFlag;
extern _Bool blinkCur;

uint8_t countGrounds;
bool InprocessStorage;
//extern volatile uint8_t step;
uint32_t duty = 100;

void HomeReturn_Process_Run(void *PrPtr);
uint16_t speedtest = 2000;
uint8_t flag_test;
float m = 0.9;

volatile _Bool En, EnMakeCoffee, GrindTest = 0, EnCleanProcess;

volatile bool firstCup = true;
uint32_t sp = 2000, dd, ss = 0;

void main(int argc, char **argv)
{
// Initialize Device/board include:
// + Enalbe FPU and lazy stacking
// + Disable Wdog timer, Disable interrupt
// + System clk, Enable peripheral clock
// + Initialize timers
// + GPIO init
    FPUEnable();
    FPULazyStackingEnable();
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
    WatchdogInit();

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
    Pgain_Steam = 200;
    Igain_Steam = 100;
    CNTL_Pole_Zero_Cal(&Steam_CNTL, Pgain_Steam, Igain_Steam, Dgain_Steam,
                       Dmax_Steam, -10, 0);

    Hot_Water.Code = ADSCON_CH1; // Assign code channel 0 ADS1118 temperature sensor
    Hot_Water.Actual_temperature = 0;
//Temperature Control terminal assign - Hot water tank
    HotWater_CNTL.DBUFF = Default; // Internal buffer
    HotWater_CNTL.Ref = &HotWater_Temperature_Ref; //Desired temperature(reference signal)
    HotWater_CNTL.Fdbk = &Hot_Water.Actual_temperature; // Acutal temperature(feedback signal)
//HotWater_CNTL.Out = &HotWater_Vout; // Contol output

    PID_DCL_DBUFF_t DCL_Defualt = { 0, 0, 0, 0 };
    HotWater_PID_DCL.DUFF_Str = DCL_Defualt;
    HotWater_PID_DCL.Ref = &HotWater_Temperature_Ref;
    HotWater_PID_DCL.Fdbk = &Hot_Water.Actual_temperature;
    HotWater_PID_DCL.ik = 0;
    HotWater_PID_DCL.Out = &HotWater_Vout;

    HotWater_PID_DCL.Coef.c1 = 0.53;
    HotWater_PID_DCL.Coef.c2 = 0.564;
    PID_DCL_cal(&HotWater_PID_DCL, 20, 0.96, 0.0091, 0.985, Dmax_HotWater, -15);

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
#ifdef VelGrindDebug
    QEP_VelGrind_Cf();
#endif
    Calibration = (float) 2 / 15.0;
//=================================================================================
//  INITIALISATION - LCD-Display connections
//=================================================================================
    LCD_Interface_Cnf(); // SPI & I/O configruation
    LCD_ST7567_Init();   // LCD initialize
// Initialize GUI interface
    SerialCommsInit();
    uint32_t count1sForVrTImer = 15;
    VrTimer1[displayTemperature] = count1sForVrTImer;
// Assign data stream to Gui variable display LCD - Display on Page 0 LCD
    dataSentList[cupsEspresso_1] = &Mode_Espresso_1.Cups;
    dataSentList[cupsEspresso_2] = &Mode_Espresso_2.Cups;

    dataSentList[cupsSpecial_1] = &Mode_Special_1.Cups;

    dataSentList[cupsSpecial_2] = &Mode_Special_2.Cups;
    dataSentList[Blade1NofTimesUsed] = &BladeA;
    dataSentList[Blade2NofTimesUsed] = &BladeB;
    dataSentList[RonNofTimesUsed] = &Ron;

    dataSentList[ExtractAtimes] = (uint16_t*) &ExtractA;
    dataSentList[ExtractBtimes] = (uint16_t*) &ExtractB;

    dataSentList[HotWaterTemp] = (uint16_t*) &Gui_HotWaterSteam;
    dataSentList[ExtractionTime] = (uint16_t*) &Gui_CoffeeExtractionTime;

    dataSentList[PulseCount] = (uint16_t*) &MilliLitresBuffer;

    dataSentList[Brand] = (uint16_t*) &BrandName;
    dataSentList[SeriNumber] = (uint16_t*) &SeriNum;
    dataSentList[Model] = (uint16_t*) &ModelName;
    dataSentList[ProductDate] = (uint16_t*) &ProductDateStr;
    dataSentList[GroupTemp] = (uint16_t*) &Group_Temp;
    dataSentList[BoilerTemp] = (uint16_t*) &Gui_TempSteam;
    dataSentList[PreinfusionTime] = (uint16_t*) &Gui_CoffeePreinfusionTime;
    dataSentList[GrindTime] = (uint16_t*) &Gui_CoffeeGrindTime;

//"Set" variables
//===================================================================================
    ParameterDefaultSetting();

    AssignParameterForMode(&Mode_Espresso_1, Pr_Packet);
    AssignParameterForMode(&Mode_Espresso_2, Pr_Packet);
    AssignParameterForMode(&Mode_Special_1, Pr_Packet);
    AssignParameterForMode(&Mode_Special_2, Pr_Packet);

    AssignGobalParSetting(Pr_Packet);
// Assign direction motor of grind module

    Mode_Espresso_1.DirGrinding = Mode_Espresso_2.DirGrinding = true;
    Mode_Special_1.DirGrinding = Mode_Special_2.DirGrinding = false; // true

//============================= Configure machine Parameter ========================================
    CountDataStorage[CupsE1] = &Mode_Espresso_1.Cups;   //&SumOfCupInUsed;
    CountDataStorage[CupsE2] = &Mode_Espresso_2.Cups;

    CountDataStorage[CupsS1] = &Mode_Special_1.Cups;
    CountDataStorage[CupsS2] = &Mode_Special_2.Cups;

    CountDataStorage[BladeL] = &BladeA;
    CountDataStorage[BladeR] = &BladeB;
    CountDataStorage[RonTimes] = &Ron;
    AssignErrorList();
    AssginWarningList();
// Read EEPROM memory

#ifdef SaveEeprom
    IntMasterDisable();
    uint32_t tempt32DataRead[NumberOfParameter];
    uint32_t *ui32Ptr = (uint32_t*) tempt32DataRead;
    uint8_t i = 0;
//----------------------------------------------------------------------------------------//
    EEPROMRead(tempt32DataRead, AddDataEeprom, NumberOfParameter * 4);
// Attract 32 bit packet to 16 bit packet and initalize to system parameter;
    for (i = 0; i < NumberOfParameter; i++)
    {
        *Pr_Packet[i] = ui32Ptr[i];
    }
//----------------------------------------------------------------------------------------//

    if (SaveDefautCount)
    {
        SaveCountToEeprom();
    }
    else
    {
        EEPROMRead(tempt32DataRead, DataCountSaveAddress, 6 * 4);
        for (i = 0; i < 8; i++)
        {
            *CountDataStorage[i] = ui32Ptr[i];
        }
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
//FPUDisable();
#ifdef VelGrindDebug
    InitFeedbackVel();
#endif

#if(WarmingMethod == SteamWarming)
    if (!GrindTest)
        WarmUpMachine();
#else
    if (!GrindTest)
        WarmUpMachine();    // Warming up
#endif

    while (1)
    {
        feedWacthdog = true;    // Feed watchdog timer
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
// Task 200ms
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
    if (PreReadLcdBt)
        SerialHostComms();  // period 3 task
    A_Group_Task = &A2;
}
// Task 2ms - Read input from extend GPIO ic TC9539
void A2(void)
{
#if(BOARD == 1)
// When signal change TCA595 wil generate interrupt and  ISR will set ReadCmdFlag
    PreReadLcdBt = 1;
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
    }
// Scan to output
    B_Group_Task = (void (*)(void)) ScanTaskMachine();
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
static inline uint32_t ScanTaskMachine(void)
{
    if (InProcess)
        return ((uint32_t) &MakeCoffeProcess);
    if (InCleanning)
        return ((uint32_t) &CleanProcess);
    if (InStartUp)
        return ((uint32_t) &WarmUpProcess);
    if (InModuleTest)
        return ((uint32_t) &CheckFinsih_ModuleTest);
    if (InWarming)
        return ((uint32_t) &WarmingPressProcess);
    return ((uint32_t) &B2);
}
// Task 50ms - Button GUI
void C1(void)
{
// ---------------------------------------- Module test ---------------------------------//
    if (TestModuleTrigger == 0 && TestModuleStart)
    {
        TestModuleTrigger = 1;
        ModuleTestMachine();
    }

// --------------------------------------------------------------------------------------//

    static uint8_t release_mode = 1, release = 0, release_CleanB = 1,
            release_WarmingB = 1;
    static _Bool mode_str;
    static uint32_t Vrtimer_HoldCalibWeigh;

// ---------------------------------------- Button cofffee maker & GUI ------------------------------//
// Select mode and make coffe
    if (release_mode == 1)
    {
        uint8_t id_MsgPage0;
        if ((TCA9539_IC1.TCA9539_Input.all & Special1_Bt) == 0)
        {
            id_MsgPage0 = 7;
            ModeInterpreter(&Mode_Special_1, LED_BT5, id_MsgPage0, false);

        }
        else if ((TCA9539_IC1.TCA9539_Input.all & Special2_Bt) == 0)
        {
            id_MsgPage0 = 8;
            ModeInterpreter(&Mode_Special_2, LED_BT7, id_MsgPage0, true);
        }
        else if ((TCA9539_IC1.TCA9539_Input.all & Expresso1_Bt) == 0)
        {
            id_MsgPage0 = 5;
            ModeInterpreter(&Mode_Espresso_1, LED_BT4, id_MsgPage0, false);
        }
        else if ((TCA9539_IC1.TCA9539_Input.all & Expresso2_Bt) == 0)
        {
            id_MsgPage0 = 6;
            ModeInterpreter(&Mode_Espresso_2, LED_BT6, id_MsgPage0, true);
        }

    }
    // If push and release Mode_button will make coffee
    if (mode_str && (release_mode == 1))
    {

        if (calibWeightFlag && HomePage)
            MakeCoffee();
        else if (EnMakeCoffee)
        {
            MakeCoffee();
            VrHoldMsg = 0;
            HoldCmd = true;
            HoldMsgFlag = true;
        }
        mode_str = false;

    }

// ---------------------------------------- Button Cleanning & GUI ------------------------------//
    if ((release_CleanB == 1) && (EnCleanProcess == 1))
    {
// Cleanning complete machine
        if ((TCA9539_IC1.TCA9539_Input.all & Clean_Bt) == 0)
        {
            //TCA9539_IC1.TCA9539_Onput.all |= LED_BT9;
            /*            ledBlink = LED_CLEAN;
             idModeRunning = 2;
             CleanningMachine();
             clModeRinse = 0;
             release_CleanB = 0;*/
            CleanInterpreter(CLEANMODE, LED_CLEAN, 2);
            CleanningMachine();

        }
// Rinse
        if ((TCA9539_IC1.TCA9539_Input.all & Rinse_Bt) == 0)
        {
            //TCA9539_IC1.TCA9539_Onput.all |= LED_BT8;   // BT8
            /*            ledBlink = LED_RINSE;
             idModeRunning = 1;
             CleanningMachine();
             clModeRinse = 1;
             release_CleanB = 0;
             */
            CleanInterpreter(RINSEMODE, LED_RINSE, 1);
            CleanningMachine();

        }

    }
//----------------------------------------------------------- Button cancel & GUI-------------------------------
    if ((idleMachine == 0) && (release == 1) && (cancel_cmd == 0))
    {
        if ((TCA9539_IC1.TCA9539_Input.all & Cancel_Task) == 0)
        {

            cancel_cmd = 1;
            release = 0;

        }
    }
    if ((TCA9539_IC1.TCA9539_Input.all & Cancel_Task) == 0)
        (Vr_ErrorClear > 40) ? ClearError() : Vr_ErrorClear++;
//-------------------------------------- Warming ---------------------------------------------
    if (release_WarmingB == 1 && (EnMakeCoffee == 1))
    {
        if (TCA9539Regs_Read16Pin(&TCA9539_IC2, Warming_Bt) == 0)
        {
            release_WarmingB = 0;
            idModeRunning = 13;
            WarmingPressMachine();

        }
    }

//----------------------------------------------------Release Button left Pannel-------------------------------------
    bool ModeSelecBttRelease =
            ((TCA9539_IC1.TCA9539_Input.all & Release_Mode_Bt)
                    == Release_Mode_Bt);
    if (ModeSelecBttRelease)
    {   // all button make coffe release
        release_mode = 1;
        if (InProcess == 0)
            TCA9539Regs_Write16Pin(&TCA9539_IC1,
                                   (LED_BT4 | LED_BT5 | LED_BT6 | LED_BT7),
                                   true);
        Vrtimer_HoldCalibWeigh = 0;
    }
    else
        Vrtimer_HoldCalibWeigh++;
    if ((Vrtimer_HoldCalibWeigh > 25) && (calibWeightFlag == 0) && HomePage)
    {
        calibWeightFlag = 1;
        Vrtimer_HoldCalibWeigh = 0;
    }

//--------------------------------Release Button Right Pannel-------------------------------------
// Button cancel release
    if ((TCA9539_IC1.TCA9539_Input.all & Cancel_Task) != 0)
    {
        release = 1;
        Vr_ErrorClear = 0;
    }
    // Button clean release
    bool CleanAndRinseBtRelease = ((TCA9539_IC1.TCA9539_Input.all
            & (Clean_Bt | Rinse_Bt)) == Release_Clean_Bt);
    if (CleanAndRinseBtRelease && (InCleanning == 0))   // Button cancel release
    {
        release_CleanB = 1;
        TCA9539_IC1.TCA9539_Onput.all |= (LED_CLEAN | LED_RINSE);
    }
    if (TCA9539Regs_Read16Pin(&TCA9539_IC2, Warming_Bt) != 0)
    {
        release_WarmingB = 1;
    }
    C_Group_Task = &C2;
}
// Task 50ms - Control level
void C2(void)
{

    ADC_READ();
    static uint8_t VrRelease = 0;
    static _Bool PumpStr = false;

#if(WarmingMethod == SteamWarming)
if (En)
(Vr_C2Task >= 300) ?
(Cmd_WriteMsg(WarmingPressMachine, NULL), Vr_C2Task = 0) :
(Vr_C2Task++); //Cmd_WriteMsg(WarmingPressMachine, NULL)
#endif
// Start pumping water to steam tank
    if (fADC_LevelSensor > (float) HighLvSensor && !ErPumpSteam)
    {
        if (InLevelPumping == 0 && p_idleMachine)
        {
            LevelControlTriger = InLevelPumping = 1;
            Cmd_WriteMsg(SteamLevelControl_Run, NULL);
        }
        Hyteresis = 1;
    }
    // Stop pumping water to steam tank
    else if (fADC_LevelSensor < (float) LowLvSensor || ErPumpSteam)
    {
        if (InLevelPumping == 1)
        {
            Cmd_WriteMsg(SteamLevelControl_Stop, NULL);
            TCA9539Regs_Write16Pin(&TCA9539_IC2, Valve_5, true);
            PumpStr = true;
            VrRelease = 0;
        }
        Hyteresis = 0;
    }
    if (PumpStr)
    {
// Release presure after pump to boiler
        (VrRelease >= 12) ? (PumpStr = 0, TCA9539Regs_Write16Pin(&TCA9539_IC2,
        Valve_5,
                                                                 false)) :
                            (VrRelease++);
    }
#if(WarmingMethod == SteamWarming)
if ((fADC_LevelSensor >= (float) 0.75) && (fADC_LevelSensor <= (float) 2.5))
{
if (Gui_TempSteam >= 110)
{
    (CountSteam >= 900) ? (SteamReady = 1) : CountSteam++;

}
else
{
    CountSteam = 0;
    SteamReady = 0;
}

}
else
SteamReady = 0;
#endif
    BlinkModeLed();
    C_Group_Task = &C1;
}
// Monitor machine
void D1(void)
{
//================================== Manual Control & Test ===============================
    ManualControlSelect();
    ManualControlTest();
//================================== Hold motor off when not used ================================
    HoldMotorOff();
//================================== Blink Error ==================================================
    ((blinkCurVr >= 2) || (ObjSelectFlag == 0)) ? (blinkCurVr = 0, blinkCur =
    true) :
                                                  (blinkCur =
                                                  false, blinkCurVr++);

// Detect Coffee out error
#if(Outlet == SensorOutlet)
if (p_idleMachine)
{
if ((TCA9539_IC3.TCA9539_Input.all & OutletSensor) == 0)
{
    (eVrTimer[eVrCoffeOutlet] > 40) ? // If never detect in 5s full extraction stage will error
    (ErOutletDetect = true, eVrTimer[eVrCoffeOutlet] = 0) : (eVrTimer[eVrCoffeOutlet]++);
}
else
eVrTimer[eVrCoffeOutlet] = 0;

}
#endif
//================================== Detect error heating  hotwater ==========================

    static float f1, f2;
    if (HeatingHotwater && (!InCleanning) && (HotWater_Vout >= 50)
            && (HotWater_Temperature_Ref - 5))
    {
        if (eVrTimer[eVrHotWaterHeatingTimeOut] >= 250)
        {

            f1 = Hot_Water.Actual_temperature;
            // if 20s temperauture not increase 5 degree, heating resistor might be error
            ErHotWaterTimeOut = (f1 <= (f2 + 3.0f)) ? true : false;
            f2 = f1;
            eVrTimer[eVrHotWaterHeatingTimeOut] = 0;

        }
        else
            eVrTimer[eVrHotWaterHeatingTimeOut]++;
    }
    else
    {
        f2 = 0;
        eVrTimer[eVrHotWaterHeatingTimeOut] = 0;
        ErHotWaterTimeOut = false;
    }
//================================= Detect error heating boiler steam  ==========================
    static float f3, f4;
    if (HeatingSteam && (Steam_Vout >= 50) && (Gui_TempSteam < 110))
    {
        if (eVrTimer[eVrSteamHeatingTimeOut] >= 350)
        {

            f3 = Hot_Water.Actual_temperature;
            // if 20s temperauture not increase 5 degree, heating resistor might be error
            ErSteamTimeOut = (f3 <= (f4 + 3.0f)) ? true : false;
            f4 = f3;
            eVrTimer[eVrSteamHeatingTimeOut] = 0;

        }
        else
            eVrTimer[eVrSteamHeatingTimeOut]++;
    }
    else
    {
        f4 = 0;
        eVrTimer[eVrSteamHeatingTimeOut] = 0;
        ErSteamTimeOut = false;
    }
    if (InLevelPumping)
    {
        if (eVrTimer[eVrSteamHeatingTimeOut] >= 310)
        {
            ErPumpSteam = true;

        }
        else
            eVrTimer[eVrSteamHeatingTimeOut]++;
    }

//=================================== Machine status =============================================
#ifndef debug
    if (countGrounds >= 50)
        fullOfGroundsDrawer = 1;
    else
        fullOfGroundsDrawer = 0;
    if (fullOfGroundsDrawer
            && (TCA9539_IC1.TCA9539_Input.all & Cancel_Task) == 0)
    {
        countGrounds = 0;
        cancel_cmd = 0;
    }

    if (Gui_HotWaterSteam >= 94 && !ErHotWater) // Temperature of hotwater steam to extraction
        Suf_HotWater = 1;
    else
        Suf_HotWater = 0;
    idleMachine = !(InProcess || InStartUp || InCleanning || InLevelPumping
            || InWarming || InManualTest);
    p_idleMachine = !(InProcess || InCleanning);
#if(WarmingMethod == HeatingResWarming)
#endif

    if (!GrindTest)
    {
        FinishStartUp = Suf_HotWater && !InStartUp;
        En = ((idleMachine) && FinishStartUp && (!fullOfGroundsDrawer)
                && (!Error));
        SweptErrorMachine();
        SweptWarningMachine();

    }
    else
    {
        FinishStartUp = 1;
        En = 1;
        InStartUp = 0;
        Error = 0;
    }
    EnMakeCoffee = En && HomePage;
    EnCleanProcess = ((idleMachine) && (!Error));
#endif
//================================ Message id to lcd =============================================
    Id_Msg_flag = fullOfGroundsDrawer || !idleMachine || Error || En;
// Delay display when finish extract
    if ((!InProcess) && HoldCmd)
        (VrHoldMsg > 15) ?
                (HoldMsgFlag = false, HoldCmd = false) : (HoldMsgFlag =
                true, VrHoldMsg++);
    ((!idleMachine && (FinishStartUp || calibWeightFlag)) || HoldMsgFlag) ?
            (idPage0Display[0] = idModeRunning) : (idPage0Display[0] = 0xFF);
//---------------------------------------------------------------------------------

    (fullOfGroundsDrawer && idleMachine) ?
            (idPage0Display[1] = 10) : (idPage0Display[1] = 0xFF);
//---------------------------------------------------------------------------------
    (Error) ? (idPage0Display[2] = 12) : (idPage0Display[2] = 0xFF);
    (WarningFlag) ? (idPage0Display[4] = 14) : (idPage0Display[4] = 0xFF);
//---------------------------------------------------------------------------------
    if (En && !calibWeightFlag)
    {
        (firstCup) ? (idPage0Display[3] = 0) : (idPage0Display[3] = 0xFE);

    }
    else if (!FinishStartUp && !HoldMsgFlag)
        idPage0Display[3] = 9;  // Running start up
    else
        idPage0Display[3] = 0xFF;

    D_Group_Task = &D1;
}

//======================================= Peripheral function ===================================================
void ADS1118_Coms(uint16_t config, int mode)
{

    config = config | 0x8000;
    ui16TxBuf[0] = config;
    ui16TxBuf[1] = config;
    uDMAChannelEnable(UDMA_CHANNEL_SSI1TX);
    uDMAChannelEnable(UDMA_CHANNEL_SSI1RX);

}
void ADS1118_Cal(ADS1118_t *ADS)
{

    int16_t temp;
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
//======================================================================================================
void SweptErrorMachine(void)
{
    uint8_t i;
    Error = 0;
    for (i = 0; i < 16; i++)
    {
        if (ErrorMachine[i].ErrorFlag != NULL)

        {
            _Bool e = *ErrorMachine[i].ErrorFlag;
            Error = Error || e;
        }
    }

}
void SweptWarningMachine(void)
{
    uint8_t i;
    WarningFlag = 0;
    for (i = 0; i < 5; i++)
    {
        if (WarningMachine[i].ErrorFlag != NULL)

        {
            _Bool w = *WarningMachine[i].ErrorFlag;
            WarningFlag = WarningFlag || w;
        }
    }

}
void ClearError(void)
{
    uint8_t i;
    for (i = 0; i < 16; i++)
        *ErrorMachine[i].ErrorFlag = false;

}
void BlinkModeLed(void)
{
    if (!(InProcess || InCleanning))
        return;
    if (VrBlinkLed >= 10)
    {
        TCA9539_IC1.TCA9539_Onput.all ^= ledBlink;
        VrBlinkLed = 0;
    }
    else
        VrBlinkLed++;

}
void WatchdogIntHandler(void)
{
// If not feed watchdog timer , not clear interrupt then reset will occur when isr service again
    if (!feedWacthdog)
        return;
    feedWacthdog = false;
    MAP_WatchdogIntClear(WATCHDOG0_BASE);

}
void WatchdogInit(void)
{
    if (MAP_WatchdogLockState(WATCHDOG0_BASE) == true)
    {
        MAP_WatchdogUnlock(WATCHDOG0_BASE);
    }
    MAP_WatchdogReloadSet(WATCHDOG0_BASE, MAP_SysCtlClockGet()); // 1s
    WatchdogIntRegister(WATCHDOG0_BASE, &WatchdogIntHandler);
#ifdef STALLWD
    MAP_WatchdogStallEnable(WATCHDOG0_BASE); // Stall watchdog timer when cpu stall use for debug
#endif
    MAP_WatchdogResetEnable(WATCHDOG0_BASE); // Enable Reset
    MAP_WatchdogEnable(WATCHDOG0_BASE);

}
void defaultISR(void)
{

}

