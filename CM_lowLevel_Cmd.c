//----------------------------------------------------------------------------------
//  FILE:           CM_lowLevel_Cmd.C
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
#include "inc/hw_types.h"
#include "inc/hw_pwm.h"

#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/qei.h"
#include "driverlib/gpio.h"
#include "driverlib/rom_map.h"
#include "driverlib/rom.h"

#include "inc/hw_qei.h"

#include "Coffee_Machine.h"
#include "TCA9539_hw_memmap.h"
#include "TCA9539.h"

#include "CM_lowLevel_Cmd.h"

#define Null 0
#define MSG_QUEUE_SIZE 16
#define MSG_QUEUE_ARG_SIZE 16
// -------------------------Define kernel function ---------------------------
void (*msg_queue[MSG_QUEUE_SIZE])(void*);
void *msg_queue_arg[MSG_QUEUE_SIZE];
volatile uint8_t msg_head = 0, msg_tail = 0;    // Ring buffer

// ------------------------Machine process ------------------------------
// Gobal flag process
volatile bool InProcess, InCleanning, InStartUp, InWarming, WarmingEnalbe = 1;
volatile bool InManualTest;
uint8_t ManualObjSelected;
uint8_t stepWarming;
#if(PosPress == 0)
#define NumberOfStep  8
#else
#define NumberOfStep  9
#endif
// Number of M_Step
//volatile uint8_t M_Step;      // Step count
enum status step_status[NumberOfStep];
/*
 #define St_Finish      1
 #define St_Running     2
 #define St_Pending     3
 */

extern bool cancel_cmd, test_step; // Cancel button
static bool cancel_storage;

#define NumOfStepClean 9
#define NumOfStepWarm 6
enum status step_statusClean[NumOfStepClean];
enum status step_statusWarm[NumOfStepWarm];

// Virtual timer for running process
uint32_t VrTimer_Grinding, VrTimer_Compress, VrTimer_Delay, VrTimer_Pumping,
        VrTimer_Relay;
_Bool holdShaftMotor, StuckGrindPosError;
float VrTimer_Compensate;
_Bool update;
uint16_t VrShaftRelease;
_Bool TriggerShaftRelease = false;
void MakeCoffee();
void CheckingFinish_StepInRuning();
void MakeCoffeProcess(void);
void WarmUpMachine(void);
void WarmUpProcess(void);
inline void SweptErrorInRunning(void);
inline void eVrTimerReset(void);
void SaveCountToEeprom(void);
inline void ReleaseErrorMotor(void);
extern uint16_t BladeA, BladeB, Ron;
int16_t VrTimer1[8];
volatile bool InPending;
extern float Group_Temp;
//------------------------------------- Process - Grinding coffee into powder-----------------------------------

volatile bool GrindingTriger, InGrinding;
uint32_t FreQ = 0, ds = 0;

float vel;
//------------------------------------- Process - Cluster compress powder coffee----------------------------------

volatile bool CompressTriger, InCompress, InCompressBack, CompressBackTrigger;
uint32_t Pos_Compress, PosStep_Compress;
uint32_t PitchOfpress;
volatile uint32_t Pulse_StepMotor, Cnt_StepMotor;
void LookUpCompressTime(void);
void PulseStepCount();
void PulseStepInitCount();
inline void RampSpeed();
inline void ExtraCompress();
bool clModeRinse;
extern float Extrude_Vout;
_Bool postimesFlag;
uint8_t extractPostimes;
//--------------------------------------------- Process - Pumping Hot water -----------------------------------------
volatile bool PumpingTriger, InPumping;
extern volatile bool FinishPumpEvent;
float SpeedDuty_Pump;
bool calibWeightFlag, calibWeightStr;
uint8_t calibWeightObj;
uint16_t fillterSignal = 0;
extern void InitPumpingEvent();
extern float SetVolume;
// Millitre pumping water
extern volatile float totalMilliLitres, MilliLitresBuffer;
// Relay Timming for turn on/off Valve in Pumping process
volatile bool InRelay_Timming;
volatile uint8_t Step_Relay, Step_Pump, Step_Pump_Cl, Step_Pump_r, count_clr;
//--------------------------------------- Process- Return home-------------------------------------------------------
volatile bool HomeReturnTriger, InHomeReturn;
uint8_t timesH = 0;
//---------------------------------------------Process - Insert POD--------------------------------------------------
volatile bool InsertPodTrigger, InInsertPod;
bool PodInsertedFlag;
bool MaskExtraCompress;
bool WarmingFlag;
//--------------------------------------- Control Level water in steam tank -----------------------------------------
volatile bool LevelControlTriger, InLevelPumping, Hyteresis;
bool SteamReady;
//------------------------------------------------------------------------------------------------------------------
// Delay between process
volatile bool delay_flag = 0;
extern uint32_t Vr_C2Task;
//---------------------------------------- Error variable - eVrtimer ------------------------------------------------
_Bool ErOutletDetect, ErHomeReturn, ErHomeStr, ErHotWaterTimeOut,
        ErSteamTimeOut, ErPumpSteam;
extern _Bool ErNoPumpPulse;
uint16_t eVrTimer[8];

// ---------------------------------------------Share variable and function ------------------------------------------
extern Mode_Parameter_t *ModeSelected;
extern TCA9539Regs TCA9539_IC1, TCA9539_IC2, TCA9539_IC3;
#define HomePeding  (TCA9539_IC3.TCA9539_Input.all & LinitSwitch)   // Read LimitSwitch
extern volatile bool firstCup;
extern void (*B_Group_Task)(void);   // B task 5ms
extern void B2(void);
volatile bool flagdelay = 0, flagdelay_Cl = 0;
uint16_t VrTimer_FlagRelay = 100;
uint16_t VrTimer_FlagRelay_Cl;
uint16_t Detect = 0;
uint32_t tp;
void Cmd_WriteMsg(void (*pFun)(void*), void *pArg)
{
    msg_queue[msg_tail] = pFun;
    msg_queue_arg[msg_tail] = pArg;
    if (msg_tail == (MSG_QUEUE_SIZE - 1))
    {
        msg_tail = 0;
    }
    else
    {
        ++msg_tail;
    }
}
void Cmd_ReadMsg(void)
{
    if (msg_head == msg_tail)
        goto skip;
    (*msg_queue[msg_head])(msg_queue_arg[msg_head]);
    if (msg_head == (MSG_QUEUE_SIZE - 1))
    {
        msg_head = 0;
    }
    else
    {
        ++msg_head;
    }
    skip: asm("nop:");
}
float offsetTime;
void Grinding_Process_Run(void *PrPtr)
{
    if (InGrinding && GrindingTriger)

    {

        Mode_Parameter_t *Mode = (Mode_Parameter_t*) PrPtr;
        GrindingTriger = 0;
        // Set direction motor
        if (Mode->DirGrinding)      // Set Direction_BLDC1
        {
            TCA9539Regs_Write16Pin(&TCA9539_IC3, Direction_BLDC1 | Enable_BLDC1,
            true);
            BladeB++;

        }
        else
        {
            // Left blade grinding
            // Clear Direction_BLDC1
            BladeA++;
            TCA9539Regs_Write16Pin(&TCA9539_IC3, Direction_BLDC1 | Enable_BLDC1,
            false);
        }
        TCA9539_IC3.updateOutputFlag = 1;
        // Enable PWM
#if(Grind == pwm)
#if(Grindmotor == bldc)
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 80000);   //1Khz
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, speedgrind);

#elif(Grindmotor == scooter)
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 100000);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 10000);   //8500
#endif
        PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
#else
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, 0);   // Run motor Grind
#endif

        // Calculate grinding time
        offsetTime = (Mode->GrindingDuration) / UnitTimer;
        VrTimer_Grinding = (uint32_t) (offsetTime);

        ppo = 0;

    }

}
void Grinding_Process_Stop(void *PrPtr)
{
    if (InGrinding)
    {
        // Disable BLDC1
        TCA9539_IC3.TCA9539_Onput.all = (TCA9539_IC3.TCA9539_Onput.all
                & ~(Enable_BLDC1));
#if(WarmingMethod == SteamWarming)
        TCA9539_IC3.TCA9539_Onput.all = (TCA9539_IC3.TCA9539_Onput.all
                & ~(Valve_4));
#endif
        TCA9539_IC3.updateOutputFlag = 1;
#if(Grind == pwm)
#if(Grindmotor == bldc)
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 0);
        PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, false);

#elif (Grindmotor == scooter)
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 5000);
#endif
#else
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_PIN_6);
#endif

    }

}
void Compress_Process_Run(void *PrPtr)
{
    if (InCompress && CompressTriger)
    {
        uint8_t Stage_Compress;
        CompressTriger = 0;
        holdShaftMotor = true;
        // Set direction
        TCA9539Regs_Write16Pin(&TCA9539_IC2, Direction_BLDC2, dirCompress_M2);
        TCA9539Regs_Write16Pin(&TCA9539_IC2, Enable_BLDC2, false); // low acitve (Enable)

        Stage_Compress = (InProcess) ? M_Step : Cl_Step; // Both of makecoffe and clean have same compress position
        defaulSpeed = 12000;
        switch (Stage_Compress)
        {
        case 1: // Compress to Pos1 (grind or insert pod position)
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_PIN_6); // Turn off motor grind
            break;
        case 3: // Compress to Pos2 (extraction position)
            TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_2, true); // Open valve 2
            break;
        case 5:
            defaulSpeed = 20000;
            break;
        }
        PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true); // Enable output pwm
#if (Grindmotor == bldc || Grind == Npwm)
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, defaulSpeed); // Freq = 8Khz // speedstep
#elif(Grindmotor == scooter)
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, speedstep / 16);
#endif
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
                         PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) / 2);
        VrTimer_Compress = *(uint32_t*) PrPtr;
        TCA9539_IC2.updateOutputFlag = 1;
        TCA9539_IC3.updateOutputFlag = 1;
        Pulse_StepMotor = (uint32_t) (PosStep_Compress * 2.0f); // scale for 400 pulse/rev and n = 1:4
        PulseStepInitCount();

    }
}
void Compress_Process_Stop(void *PrPtr)
{
// Have compress cmd when pumping
    if (InCompress || InPumping)
    {

        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);
        PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, false);

// Reset motor compress if fault condition occur in Poscompress (after extraction)
        uint16_t tempread = TCA9539Regs_Read16Pin(&TCA9539_IC3,
        WarningPressMotor);
    }

}
void CompressBack_Process_Run(void *PrPtr)
{
    if (InCompressBack && CompressBackTrigger)
    {
        CompressBackTrigger = 0;
        TCA9539Regs_Write16Pin(&TCA9539_IC2, Direction_BLDC2,
        dirCompressBack_M2);
        TCA9539Regs_Write16Pin(&TCA9539_IC2, Enable_BLDC2, false); // low acitve (Enable)
        PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
        uint32_t speed = 20000;
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, speed);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
                         PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) / 2);
        VrTimer_Compress = *(uint32_t*) PrPtr;
        TCA9539_IC2.updateOutputFlag = 1;
        Pulse_StepMotor = (uint32_t) (PosStep_Compress * 2.0f); // scale for 400 pulse/rev and n = 1:4
        PulseStepInitCount();
    }
}
void CompressBack_Process_Stop(void *Ptr)
{
    if (!InCompressBack)
        return;
    TCA9539Regs_Write16Pin(&TCA9539_IC2, Enable_BLDC2, true); // low acitve (Enable)
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, false);

}
void Pumping_Process_Run(void *PrPtr)
{

    if (InPumping && PumpingTriger)
    {

        PumpingTriger = Step_Pump = Step_Pump_Cl = Step_Pump_r = 0;
        MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
        // Configure  turn on valve 1// brewValve

        if (!InCleanning)
        {
            Mode_Parameter_t *Mode = (Mode_Parameter_t*) PrPtr;
            float volume = Mode->AmountOfWaterPumping.stage_1;
            TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_1, true); // Open valve 1
            // Speed motor pumping in preinfusion stage
            SpeedDuty_Pump = (ModeSelected->SpeedMotorInfusion);
            SetVolume = volume;
            //VrTimer_FlagRelay = (Mode->PreInfusion - infu) / UnitTimer; // Time for preinfusion
            VrTimer_FlagRelay = infu / UnitTimer;
            TCA9539Regs_Write16Pin(&TCA9539_IC2, Valve_5, false);
            TCA9539_IC2.updateOutputFlag = true;
            flagdelay = 1;  // Used for stage pumping fuction
        }
        else
        {
            TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_1, true); // Open valve 1
            SetVolume = CleanVolume;
            SpeedDuty_Pump = CleanPressure_Pump;
            if (Cl_Step == Cl_Pumping1)
            {
                flagdelay_Cl = 1;
                VrTimer_FlagRelay_Cl = 3 / UnitTimer;   // hold 1s
            }
        }
        TCA9539_IC3.updateOutputFlag = 1;
        // Initialize a pump envent to calculate volume
        FinishPumpEvent = false;

        if ((ModeSelected->MotorPreInfusion && !InCleanning) || InCleanning)
        {
            PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 4000);
            uint32_t dutyCount = SpeedDuty_Pump
                    * PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1);
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, dutyCount);
            PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
            MilliLitresBuffer = 0;
        }
        Detect = 0;
    }
}
void Pumping_Process_Stop(void *PrPtr)
{
    if (InPumping)
    {
        // Turn off valve 1
        //TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_2 | Valve_1, false);
        TCA9539_IC3.updateOutputFlag = true;
        totalMilliLitres = MilliLitresBuffer;
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
        PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, false);
        uint16_t tempread = TCA9539Regs_Read16Pin(&TCA9539_IC3,
        WarningPressMotor);
        /*        if (tempread == 0) // PosCompress(after extraction) vs extra compress
         {
         TCA9539Regs_Write16Pin(&TCA9539_IC2, Enable_BLDC2, true); // Disable/ connect to gnd - Times 2 for reset warning if fault
         TCA9539_IC2.updateOutputFlag = 1;
         }*/
        FinishPumpEvent = true;

    }

}
void SteamLevelControl_Run(void *PrPtr)
{

    if (InLevelPumping && LevelControlTriger)
    {
        LevelControlTriger = 0;
        TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_4, true);
        uint32_t countDuty = (uint32_t) (0.5
                * PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1));
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, countDuty);
        PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
        TCA9539_IC2.updateOutputFlag = 1;

    }
}
void SteamLevelControl_Stop(void *PrPtr)
{
    if (InLevelPumping)
    {
        InLevelPumping = 0;
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
        PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, false);
        TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_4, false);
        TCA9539_IC2.updateOutputFlag = 1;

    }

}
void HomeReturn_Process_Run(void *PrPtr)
{
    if (InHomeReturn && HomeReturnTriger)
    {

        HomeReturnTriger = 0;
        TCA9539Regs_Write16Pin(&TCA9539_IC2, Direction_BLDC2,
        dirReturnHome_M2); // Set LOW
        TCA9539Regs_Write16Pin(&TCA9539_IC2, Enable_BLDC2, false); // Enable - Not connect to gnd
        TCA9539_IC2.updateOutputFlag = 1;
        PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
        defaulSpeed = 12000;
#if (Grindmotor == bldc || Grind == Npwm)
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, defaulSpeed); // Freq = 8Khz// speedstep

#elif(Grindmotor == scooter)
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, speedstep / 16);   // Freq = 8Khz
#endif
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
                         PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) / 2);
    }
}
void HomeReturn_Process_Stop(void *PrPtr)
{
    if (InHomeReturn)
    {
        //Disable compress driver motor -  Not used
        TCA9539_IC2.updateOutputFlag = 1;
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);
        PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, false);

    }
}

void Delay_20ms(uint16_t time_20ms)
{
    VrTimer_Delay = time_20ms;
    delay_flag = 1;

}
void Relay_Timming(uint32_t Vrtimer)
{
    VrTimer_Relay = Vrtimer;
    Step_Relay = 0;
    InRelay_Timming = 1;

}
// Interrupt external gpio fuction (reading input - tca9539)
void Read_INT_Handler(void)
{
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_1);
    TCA9539_IC1.ReadCmdFlag = 1;
    TCA9539_IC2.ReadCmdFlag = 1;
    TCA9539_IC3.ReadCmdFlag = 1;
}
// Interrupt timming 20ms fucion for coffee maker
void TimmingPorcess()
{
    uint32_t countDuty;
    TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
//----------------------------------------- Timing grind coffee fuction-------------------------------------------------
    if (InGrinding)
    {

        if (VrTimer_Grinding != 0)
        {

            VrTimer_Grinding--;
            ppo++;
            Gui_CoffeeGrindTime = (float) ppo * UnitTimer;
        }
        if (Gui_CoffeeGrindTime > 20.0f)          // Limit time for grind coffee
            VrTimer_Grinding = 0;
        return;

    }
//-------------------------------------Hold compress -------------------------------------------
    bool HoldShaftMotor;
    if (InCleanning || (InProcess && (ModeSelected->smallSize)))
        //   HoldShaftMotor = 0;
        //(InPumping) && HoldShaftMotor)
        if (((InPumping && Step_Pump == 1) && HoldShaftMotor) || InHomeReturn)
            ReleaseErrorMotor();

//---------------------------------------------------- Timming delay fuction---------------------------------------------------
    if (delay_flag)
    {
        (VrTimer_Delay != 0) ? (VrTimer_Delay--) : (delay_flag = 0);

    }
//----------------------------------Timming release pressure & close brew valve 2 fuction --------------------------------------
    if ((step_status[M_ExtractCoffee] == St_Finish
            || step_statusClean[Cl_Pumping1] == St_Finish) && InRelay_Timming)
    {
        if (VrTimer_Relay != 0)
            VrTimer_Relay--;
        else
        {
#if(ReleasePressure == 1)
            switch (Step_Relay)
            {
            case 0:

                VrTimer_Relay = 75;
                // Release pressure
                TCA9539Regs_Write16Pin(&TCA9539_IC2, Valve_5, true);
                TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_1, false);
                Step_Relay++;
                break;
            case 1:
                TCA9539Regs_Write16Pin(&TCA9539_IC2, Valve_5, false);
                InRelay_Timming = 0;
                break;
            }
#else
            TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_1, false);
            TCA9539_IC3.updateOutputFlag = 1;
            InRelay_Timming = 0;
#endif
        }
        return;
    }
//----------------------------------------------- Timing for count extraction time ----------------------------------------------
    if (triggerCount_ExtractionTime)
        CoffeeExtractionTime++;
    if (triggerCount_PreinfusionTime)
        CoffeePreinfusionTime++;
//--------------------------------- PreInfution & full extraction stage in pumping process  -------------------------------------
    uint16_t ShaftHold, ShaftError;
    if (flagdelay && (step_status[M_ExtractCoffee] == St_Running))
    {
        if (VrTimer_FlagRelay != 0)
            VrTimer_FlagRelay--;
        else
        {
            switch (Step_Pump)
            {
            case 0: // Preinfustion extraction stage -
                if (!MaskExtraCompress)
                {
                    VrTimer_FlagRelay = 2 / UnitTimer;
                    /*     VrTimer_FlagRelay = (ModeSelected->PreInfusion - infu)
                     / UnitTimer;*/
                    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
                    ExtraCompress();
                    TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_1, false);
                    TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_2, false);
                }

                Step_Pump++;
                break;
            case 1:

                VrTimer_FlagRelay = (ModeSelected->PreInfusion - infu)
                        / UnitTimer;
                SpeedDuty_Pump = (ModeSelected->SpeedMotorInfusion);
                uint32_t dutyCount = SpeedDuty_Pump
                        * PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1);
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, dutyCount);
                TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_1, true);
                TCA9539_IC3.updateOutputFlag = true;
                Step_Pump++;
                break;
            case 2: // Full extraction stage
                /*                if (ModeSelected->smallSize)
                 {
                 ShaftError = TCA9539Regs_Read16Pin(&TCA9539_IC3,
                 WarningPressMotor); // = 0 is error
                 ShaftHold = TCA9539Regs_Read16Pin(&TCA9539_IC2,
                 Enable_BLDC2); // true (!= 0) is disable
                 if ((VrTimer_Compress != 0) || (ShaftHold != 0)
                 || (ShaftError == 0))
                 break; // Not max pressure when keep loose shaft motor
                 }*/
                SpeedDuty_Pump = HighPressure_Pump;
                VrTimer_FlagRelay = (ModeSelected->PosInfusion) / UnitTimer; // 2s
                TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_2, false);

#if (Grindmotor == bldc || Grind == Npwm)
                countDuty = (uint32_t) (SpeedDuty_Pump
                        * PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1));
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, countDuty);
                TCA9539Regs_Write16Pin(
                        &TCA9539_IC2,
                        Enable_BLDC2,
                        true); // low acitve(conect gnd) Enable
#elif(Grindmotor == scooter)
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, SpeedDuty_Pump / 16);
#endif//
                PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
                triggerCount_ExtractionTime = 1;
                triggerCount_PreinfusionTime = 0;
                Step_Pump++;

                break;
            case 3:
                TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_2, true);
                flagdelay = 0;
                // VrTimer_FlagRelay = infu / UnitTimer;
                Step_Pump++;
                break;
            }
            TCA9539_IC3.updateOutputFlag = true;
        }
        return;
    }
    if (flagdelay_Cl && step_statusClean[Cl_Pumping1] == St_Running)
    {
        if (VrTimer_FlagRelay_Cl != 0)
            VrTimer_FlagRelay_Cl--;
        else
        {
            switch (Step_Pump_Cl)
            {
            case 0:
                if (clModeRinse)
                {
                    Step_Pump_Cl = 8;
                    break;
                }
                switch (Step_Pump_r)
                {
                case 1:
                case 5:

                    TCA9539Regs_Write16Pin(&TCA9539_IC3,
                    Valve_2 | Valve_1,
                                           true);
                    TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_3,
                    false);
                    infu_cl = 4; // 2s
                    VrTimer_FlagRelay_Cl = infu_cl / UnitTimer;
                    Step_Pump_r++;
                    break;
                case 0: // generate pressure
                case 2:
                case 4:
                case 6:
                    infu_cl = 20; // 20s
                    VrTimer_FlagRelay_Cl = infu_cl / UnitTimer;
                    TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_1, true);
                    TCA9539Regs_Write16Pin(&TCA9539_IC3,
                    Valve_2 | Valve_3,
                                           false);
                    Step_Pump_r++;

                    break;

                case 3:
                case 7:
                    infu_cl = 5;
                    VrTimer_FlagRelay_Cl = infu_cl / UnitTimer;

                    TCA9539Regs_Write16Pin(&TCA9539_IC3,
                    Valve_2 | Valve_1,
                                           false);
                    // turn off valve 1 vs valve 2
                    TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_3, true);
                    /*                    TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_1 | Valve_3,
                     false);*/
                    Step_Pump_r++;
                    break;
                case 8:
                    TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_3,
                    false);
                    Step_Pump_Cl++;
                    break;
                }
                break;

            case 1:
            case 3:
            case 5:
            case 7:
            case 9:
                SpeedDuty_Pump = HighPressure_Pump;
                VrTimer_FlagRelay_Cl =
                        (clModeRinse) ? (Timming(5.4)) : (Timming(10));
                Step_Pump_Cl++;

                // Enable motor driver

                TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_2 | Valve_1,
                true);
                TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_3, false);
                countDuty = (uint32_t) (SpeedDuty_Pump
                        * PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1));
                MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, countDuty);
                MAP_PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);

                break;
            case 2:
            case 4:
            case 6:
            case 8:
                VrTimer_FlagRelay_Cl =
                        (clModeRinse) ? (Timming(2)) : (Timming(5));
                TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_1 | Valve_2,
                false);
                TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_3, true);
                Step_Pump_Cl++;
                break;
                // close valve 2
            case 10:
                TCA9539Regs_Write16Pin(&TCA9539_IC3,
                Valve_1 | Valve_3,
                                       false);
                TCA9539Regs_Write16Pin(&TCA9539_IC3,
                Valve_2,
                                       true);
                flagdelay_Cl = Step_Pump_Cl = 0;
                Pumping_Process_Stop(Null);
                break;
            }
            TCA9539_IC3.updateOutputFlag = true;
        }

    }
//-------------------------------------------------------------------------------------------------------------------------------
}

void CheckingFinish_StepInRuning()
{

    if (InProcess)
    {
        switch (M_Step)
        {
//========================================================= Home Return ===========================================
        case M_ReturnHome1:
        case M_ReturnHome2:
            RampSpeed();

            if ((TCA9539_IC3.TCA9539_Input.all & LinitSwitch) == 0)
            {
                Cmd_WriteMsg(&HomeReturn_Process_Stop, Null);
                step_status[M_Step] = St_Finish;
                eVrTimer[eVrHomeReturn] = 0;
                ErHomeReturn = false;

            }
            else
            {
                /*                if (InHomeReturn)
                 (eVrTimer[eVrHomeReturn] > 250) ? // If in return home but not detect home limit signal in 4s will error
                 (ErHomeReturn = true) : (eVrTimer[eVrHomeReturn]++);*/
                uint16_t tempread = TCA9539Regs_Read16Pin(&TCA9539_IC3,
                WarningPressMotor);
                if (InHomeReturn && tempread == 0)
                    ErHomeReturn = true;

            }
            break;
//======================================================== Gringding coffee =========================================
        case M_GrindingCoffee:
            if (InGrinding)
            {

                if (VrTimer_Grinding == 0)
                {
                    Cmd_WriteMsg(&Grinding_Process_Stop, Null);
                    step_status[M_Step] = St_Finish;

                }

            }
            if (cancel_cmd)
            {
                VrTimer_Grinding = 0;
                cancel_storage = 1;
            }
            break;
//=================================================== Compress coffee ================================================
        case M_GrindPos:
            if (StuckGrindPosError)
            {
                cancel_cmd = true;
                StuckGrindPosError = false;
            }

        case M_ExtractPos:
        case M_RemoveGround:
        case M_PosCompress:
            RampSpeed();    // Use for ramping speed of motor
            if ((VrTimer_Compress == 0) && InCompress) // Monitor pulse in PulseStepCount func and stop motor when enough pulse
                step_status[M_Step] = St_Finish;
            break;

            /*        case M_PosCompress:
             if ((VrTimer_Compress == 0) && InCompressBack)
             step_status[M_Step] = St_Finish;
             break;*/

//======================================================== Pumping water =============================================
        case M_ExtractCoffee:
            //TCA9539Regs_Write16Pin(&TCA9539_IC2, Enable_BLDC2, false); // low acitve (Enable)
            if (Step_Pump == 4) // Step full extraction - After infusion
            {
#if(Outlet == SensorOutlet)
//---------------------------------------------- Filter sensor outlet signal - 0.3s -------------------------------
                if ((TCA9539_IC3.TCA9539_Input.all & OutletSensor) == 0
                        && (Detect == 0))
                {
                    fillterSignal++;
                    if (fillterSignal >= 15) // Sensor need detect in 0.3s to comfirm have coffe out
                    {
                        InitPumpingEvent();
                        fillterSignal = 0;
                        Detect = 1;
                    }

                }
                else
                    fillterSignal = 0;
//----------------------------------------- Detect error sensor outlet signal - 5s -----------------------------------------
                if ((TCA9539_IC3.TCA9539_Input.all & OutletSensor) != 0
                        && Detect == 0)
                {
                    if (!ErOutletDetect)
                    {
               /*         (eVrTimer[eVrCoffeOutlet] > 700) ? // If never detect in 5s full extraction stage will error
                        (ErOutletDetect = true) : (eVrTimer[eVrCoffeOutlet]++);*/
                    }
                    else
                        eVrTimer[eVrCoffeOutlet] = 0;
                }
                else
                {
                    eVrTimer[eVrCoffeOutlet] = 0;
                    ErOutletDetect = false;
                }
#else
                if (Detect == 0)
                {
                    InitPumpingEvent();
                    Detect = 1;
                }
#endif

            }
            if (FinishPumpEvent)
                step_status[M_Step] = St_Finish;
            break;
        };
        SweptErrorInRunning();
    }
    B_Group_Task = &B2;

}

void MakeCoffee()
{
    uint8_t i;
    for (i = 0; i < NumberOfStep; i++)
        step_status[i] = St_Pending;
    InHomeReturn = InGrinding = InCompress = InPumping = 0;
// Vr_C2Task = 100;
    InProcess = 1;
    firstCup = false;
    if (test_step == 0)
        M_Step = M_ReturnHome1;
    else
        M_Step = M_GrindingCoffee;
    Gui_CoffeeExtractionTime = Gui_CoffeePreinfusionTime = 0;
    triggerCount_ExtractionTime = triggerCount_PreinfusionTime = 0;
    InWarming = delay_flag = Gui_CoffeeGrindTime = postimesFlag = 0;
#if(WarmingMethod == SteamWarming)
    if (calibWeightFlag)
        TCA9539_IC3.TCA9539_Onput.all = (TCA9539_IC3.TCA9539_Onput.all
                & ~(Valve_4));
    else
        TCA9539_IC3.TCA9539_Onput.all = ((TCA9539_IC3.TCA9539_Onput.all
                & ~(Valve_4)) | Valve_4);
#endif
    eVrTimerReset();
    TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT); // Interrupt timer
    TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
}

void MakeCoffeProcess(void)
{

    if (InProcess)
    {
        switch (M_Step)
        {
        case M_ReturnHome1:
        case M_ReturnHome2: // Home return compress block
            if ((!InHomeReturn) && (!delay_flag) && (!InRelay_Timming))
            {
                step_status[M_Step] = St_Running;
                HomeReturnTriger = InHomeReturn = 1;
                if (HomePeding != 0)
                {
                    speedstep = 7500; // 10000
                    Cmd_WriteMsg(&HomeReturn_Process_Run, Null);
                }

            }

            if ((InHomeReturn) && (step_status[M_Step] == St_Finish))
            {
                InHomeReturn = 0;
                M_Step++;
                if (M_Step == M_GrindPos) // Delay to next grind process
                    Delay_20ms(25);
                if (timesH == 2) // stop at home when complete move to remove grounds
                    M_Step = M_finishProcess;
            }

            break;
        case M_GrindingCoffee: // Grinding coffee
            if ((!InGrinding) && (!delay_flag))
            {
                step_status[M_Step] = St_Running;
                GrindingTriger = InGrinding = 1;
                Cmd_WriteMsg(&Grinding_Process_Run, (void*) ModeSelected);

            }
            if ((InGrinding) && (step_status[M_Step] == St_Finish))
            {
                if (!fullOfGroundsDrawer)   // Count coffee grounds
                    countGrounds++;
                InGrinding = 0;
                Delay_20ms(25);
                if (calibWeightFlag)
                {
                    M_Step = M_ReturnHome2;
                    break;
                }
                M_Step++;
            }
            break;
        case M_GrindPos:
            PosStep_Compress = stepPos1;
            speedstep = 7500;
            Pos_Compress = 1;
            goto compresscmd;
        case M_ExtractPos:
            LookUpCompressTime();
            Pos_Compress = 110; // Mask error
            Ron++;
            PosStep_Compress = stepPos2;
            /*            (!postimesFlag) ?
             (PosStep_Compress = stepPos2) : ((PosStep_Compress =
             stepPos5));*/
            speedstep = 9000;
            goto compresscmd;
        case M_PosCompress:
            PosStep_Compress = stepPos4;
            speedstep = defaulSpeed;
            Pos_Compress = true;
            goto compresscmd;
        case M_RemoveGround:
            speedstep = 9000;
            PosStep_Compress = stepPos3; // stepPos3
            Pos_Compress = 1;
            compresscmd: if ((!InCompress) && (!delay_flag))
            {

                step_status[M_Step] = St_Running;
                InCompress = CompressTriger = 1;

                Cmd_WriteMsg(&Compress_Process_Run, (void*) &Pos_Compress);

            }
            if ((InCompress) && (step_status[M_Step] == St_Finish))
            {

                InCompress = 0;
                if (cancel_cmd)
                {
                    M_Step = M_ReturnHome2;
                    cancel_cmd = 0;
                    cancel_storage = 1;
                    break;
                }
                // Delay in Precompress and after grind

                if (M_Step == M_ExtractPos)
                {
                    /*                    if (extractPostimes < 2 && (ModeSelected->smallSize))
                     {
                     postimesFlag = true;
                     extractPostimes++;
                     break;
                     }*/
                    TCA9539Regs_Write16Pin(&TCA9539_IC2, Valve_5, true);
                    Delay_20ms(50);
                }
                if (M_Step == M_GrindPos)
                    Delay_20ms(50);
                M_Step++;
            }
            break;
            /*        case M_PosCompress:
             PosStep_Compress = stepPos4;
             Pos_Compress = true;
             if (!InCompressBack && (!delay_flag))
             {
             step_status[M_Step] = St_Running;
             InCompressBack = CompressBackTrigger = 1;
             Cmd_WriteMsg(&CompressBack_Process_Run, (void*) &Pos_Compress);

             }
             if ((InCompressBack) && (step_status[M_Step] == St_Finish))
             {
             InCompressBack = 0;
             if (cancel_cmd)
             {
             M_Step = M_ReturnHome2;
             cancel_cmd = 0;
             cancel_storage = 1;
             break;
             }
             M_Step++;
             }
             break;*/
        case M_ExtractCoffee:
            if ((!InPumping) && (!delay_flag))
            {
                step_status[M_Step] = St_Running;
                InPumping = PumpingTriger = 1;
                Cmd_WriteMsg(&Pumping_Process_Run, (void*) ModeSelected);
                triggerCount_PreinfusionTime = 1;
                timesH = 0;
                CoffeePreinfusionTime = CoffeeExtractionTime = 0;
            }
            if (InPumping)
            {
                Gui_CoffeeExtractionTime = CoffeeExtractionTime * UnitTimer;
                Gui_CoffeePreinfusionTime = CoffeePreinfusionTime * UnitTimer;
                if (step_status[M_Step] == St_Finish)
                {
                    triggerCount_ExtractionTime = 0;
                    InPumping = 0;
                    M_Step++;
                    if (cancel_cmd)
                    {

                        cancel_cmd = 0;
                        cancel_storage = 1;
                    }
                    Relay_Timming(0);
                    //(M_Step == M_PosCompress) ? Delay_20ms(50) : Delay_20ms(0);
                    Delay_20ms(50);
                }
                if (cancel_cmd)
                {
                    Cmd_WriteMsg(&Pumping_Process_Stop, Null);
                    cancel_storage = 1;

                }

            }
            break;
        case M_finishProcess:
            if (step_status[4] == St_Finish)
            {
                if (InRelay_Timming == 0)
                {
                    step_status[M_Step] = St_Finish;

                    if (timesH < 2)
                    {
                        M_Step = M_ReturnHome2;
                        timesH++;
                    }
                    else
                    {
                        if (!cancel_storage)
                        {
                            ModeSelected->Cups++;
                            ModeSelected->WeigtOfPowder =
                                    Gui_CoffeeExtractionTime;
                        }
                        timesH = 0;
                        InProcess = cancel_storage = 0, cancel_cmd = 0;
                        M_Step = M_ReturnHome1;
                        calibWeightFlag = 0;
                        TimerIntDisable(TIMER4_BASE,
                        TIMER_TIMA_TIMEOUT);
                        QEIIntDisable(QEI0_BASE, QEI_INTTIMER);
                    }

                }
                else
                    step_status[M_Step] = St_Running;
            }
            else
            {
                if (timesH < 2)
                {
                    M_Step = M_ReturnHome2;
                    timesH++;
                    break;
                }
                InProcess = timesH = 0;
                cancel_cmd = 0;
                M_Step = M_ReturnHome1;
                if (calibWeightFlag)
                    calibWeightStr = 1;
                calibWeightFlag = cancel_storage = 0;
                TimerIntDisable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
                QEIIntDisable(QEI0_BASE, QEI_INTTIMER);
            }
            SaveCountToEeprom();
            break;
        default:
            InProcess = 0;
        };
    }
    B_Group_Task = &CheckingFinish_StepInRuning;
}

void WarmUpMachine()
{
    uint8_t i;
    for (i = 0; i < NumberOfStep; i++)
    {
        step_status[i] = St_Pending;
    }
    InHomeReturn = InCompress = 0;
    InStartUp = true;
    Wm_Step = Wm_TurnOnHeating;
    TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT); // Interrupt timer
    TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
}
void WarmUpProcess()
{
    if (InStartUp)
    {

#if(WarmingMethod == SteamWarming)
        if(SteamReady){
        switch (Wm_Step)

        {
        case Wm_TurnOnHeating:
            TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_4, true);
            Delay_20ms(8000);
            Wm_Step++;
            break;
        case Wm_finish:
            if (!delay_flag)
            {
                InStartUp = cancel_cmd = 0;
                Wm_Step = Wm_TurnOnHeating;
                TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_4, false);
                HeatingPress = 1;
                TimerIntDisable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
            }
            if (cancel_cmd)
                VrTimer_Delay = 0;
            break;
        };
        }
#elif(WarmingMethod == HeatingResWarming)
        switch (Wm_Step)

        {
        case Wm_TurnOnHeating:
            //Delay_20ms(30000);
            Wm_Step++;
            break;
        case Wm_finish:
            /*            if (!delay_flag)
             {
             InStartUp = cancel_cmd = 0;
             Extrude_Vout = 50;
             Wm_Step = Wm_TurnOnHeating;
             TimerIntDisable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
             }
             if (cancel_cmd)
             VrTimer_Delay = 0;*/
            if (Group_Temp >= 55 || cancel_cmd)
            {
                InStartUp = cancel_cmd = 0;
                Wm_Step = Wm_TurnOnHeating;
                TimerIntDisable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
            }
            break;
        };
#endif
    }
    B_Group_Task = &B2;

}

void WarmingPressMachine(void *P)
{
    uint8_t i;
    InWarming = true;
    WarmingFlag = false;
    Wa_Step = Wa_ReturnHome1;
    for (i = 0; i < NumOfStepWarm; i++)
        step_statusWarm[i] = St_Pending;
    TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT); // Interrupt timer
    TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
}
void CheckFinish_Warming(void)
{
    if (InWarming)
    {
        switch (Wa_Step)
        {
        case Wa_ReturnHome1:
        case Wa_ReturnHome2:
            RampSpeed();
            if ((TCA9539_IC3.TCA9539_Input.all & LinitSwitch) == 0)
            {
                Cmd_WriteMsg(&HomeReturn_Process_Stop, Null);
                step_statusWarm[Wa_Step] = St_Finish;
            }
            break;
        case Wa_CompressPos1:
        case Wa_CompressPos2:
            RampSpeed();
            if ((VrTimer_Compress == 0) && InCompress)
                step_statusWarm[Wa_Step] = St_Finish;
            break;
        case Wa_WarmingBrew:
            if (cancel_cmd)
            {
                step_statusWarm[Wa_Step] = St_Finish;
                TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_1, false);
                Delay_20ms(50);
            }
            break;
        }
    }
    B_Group_Task = &B2;
}
void WarmingPressProcess(void)
{
    if (InWarming)
    {
        switch (Wa_Step)
        {

        case Wa_ReturnHome1:
        case Wa_ReturnHome2:
            if ((!InHomeReturn) && (!delay_flag))
            {
                HomeReturnTriger = InHomeReturn = 1;
                if (HomePeding != 0)
                {
                    speedstep = 7500; // 10000
                    Cmd_WriteMsg(&HomeReturn_Process_Run, Null);
                }
            }

            if ((InHomeReturn) && (step_statusWarm[Wa_Step] == St_Finish))
            {
                InHomeReturn = 0;
                Wa_Step++;
            }
            break;
        case Wa_CompressPos1:
            PosStep_Compress = stepPos1 + 5000;
            speedstep = 9000;
            Pos_Compress = 1;
            goto skip_wa;
        case Wa_CompressPos2:
            PosStep_Compress = 1000;
            speedstep = 9000;
            Pos_Compress = 1;
            skip_wa: if ((!InCompress) && (!delay_flag))
            {
                step_statusWarm[Wa_Step] = St_Running;
                InCompress = CompressTriger = 1;

                Cmd_WriteMsg(&Compress_Process_Run, (void*) &Pos_Compress);
            }
            if ((InCompress) && (step_statusWarm[Wa_Step] == St_Finish))
            {
                InCompress = 0;
                if (cancel_cmd)
                {
                    Wa_Step = Wa_ReturnHome2;
                    cancel_cmd = 0;
                    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
                    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, false);
                    break;
                }
                if (Wa_Step == Wa_CompressPos1)
                {

                    TCA9539Regs_Write16Pin(&TCA9539_IC3,
                    Valve_1 | Valve_2,
                                           true);
                    uint32_t countDuty = (uint32_t) (0.45
                            * PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1));
                    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, countDuty);
                    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);

                }
                Delay_20ms(100);
                Wa_Step++;
            }
            break;
        case Wa_WarmingBrew:
            if ((!WarmingFlag) && (!delay_flag))
            {
                WarmingFlag = true;
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
                PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, false);
                TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_2, false);

            }
            if (step_statusWarm[Wa_Step] == St_Finish)
            {
                Wa_Step++;
            }
            break;
        case Wa_FinishWarm:
            InWarming = cancel_cmd = 0;
            Wa_Step = Wa_ReturnHome1;
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
            PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, false);
            TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_2, true);
            TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_1, false);
            TimerIntDisable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
            QEIIntDisable(QEI0_BASE, QEI_INTTIMER);
            break;
        }
    }
    B_Group_Task = &CheckFinish_Warming;
}

void LookUpCompressTime(void)
{
    PitchOfpress = ModeSelected->Pitch;
    switch (PitchOfpress)
    {
    case 11:
        stepPos2 = 6000;
        break;
    case 12:
        stepPos2 = 5900;
        break;
    case 13:
        stepPos2 = 5800;
        break;
    case 14:
        stepPos2 = 5700;
        break;
    case 15:
        stepPos2 = 5600;
        break;
    case 16:
        stepPos2 = 5500;
        break;
    case 17:
        stepPos2 = 5400;
        break;
    case 18:
        stepPos2 = 5300;
        break;
    case 19:
        stepPos2 = 5200;
        break;
    case 20:
        stepPos2 = 5100;
        break;
    case 21:
        stepPos2 = 5000;
        break;
    case 22:
        stepPos2 = 4900;
        break;
    case 23:
        stepPos2 = 4800;
        break;
    case 24:
        stepPos2 = 4700;
        break;
    case 25:
        stepPos2 = 4600;
        break;
    case 26:
        stepPos2 = 4500;
        break;
    case 27:
        stepPos2 = 4400;
        break;
    case 28:
        stepPos2 = 4300;
        break;
    case 29:
        stepPos2 = 4200;
        break;
    case 30:
        stepPos2 = 4100;
        break;
    }
}
#ifdef VelGrindDebug
void FeedbackVelGrind(void)
{
    if (QEIIntStatus(QEI1_BASE, true) == QEI_INTTIMER)
    {
        QEIIntClear(QEI1_BASE, 0x0F);
        ds = QEIPositionGet(QEI1_BASE);
        FreQ = QEIVelocityGet(QEI1_BASE);

#if(Grind == Npwm)
        vel = FreQ / 4 * 600;
#endif
        static uint8_t i = 0;
        if (i > 8)
            i = 0;
        buffervel[i] = vel;
        uint8_t j;
        tempv = 0;
        for (j = 0; j < 8; j++)
            tempv += buffervel[j];
        avgvel = tempv / 8;
        i++;
    }
}

void InitFeedbackVel(void)
{
    QEIPositionSet(QEI1_BASE, 0x0000);      // Initalize
    QEIIntClear(QEI1_BASE, 0x0F);   // Clear any interrupt flag
    QEIIntEnable(QEI1_BASE, QEI_INTTIMER);  // Init interrupt timer out
    QEIEnable(QEI1_BASE);
}
void QEP_VelGrind_Cf(void)
{
    QEIDisable(QEI1_BASE);
    QEIIntDisable(QEI1_BASE,
    QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

    GPIOPinConfigure(GPIO_PC5_PHA1);
    GPIOPinConfigure(GPIO_PC6_PHB1);
    GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6);
///////////////////////////////////////////////////////////////////
#if(Grindmotor == scooter || Grind == Npwm)
    QEIConfigure(
            QEI1_BASE,
            QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP
                    | QEI_CONFIG_CAPTURE_A_B,
            0xFFFFFFFF);

#endif
    HWREG(QEI1_BASE + QEI_O_CTL) = ((HWREG(QEI1_BASE + QEI_O_CTL)
            & ~(QEI_CTL_INVI)) | QEI_CTL_INVI);     // Invert Index Pulse

    QEIVelocityConfigure(QEI1_BASE, QEI_VELDIV_1, 8000000); // 100ms
    QEIFilterConfigure(QEI1_BASE, QEI_FILTCNT_16);
    QEIFilterEnable(QEI1_BASE);
    QEIVelocityEnable(QEI1_BASE);
    QEIIntRegister(QEI1_BASE, &FeedbackVelGrind);
    uint32_t status = QEIIntStatus(QEI1_BASE, true);
    QEIIntClear(QEI1_BASE, status);
    QEIEnable(QEI1_BASE);

}
#endif
void PulseStepInitCount()
{
    Cnt_StepMotor = 0;
    // PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true); // Enable output pwm
    MAP_PWMGenIntClear(PWM0_BASE, PWM_GEN_0, PWM_INT_CNT_LOAD);
    MAP_PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_0, PWM_INT_CNT_LOAD);
    MAP_PWMIntEnable(PWM0_BASE, PWM_INT_GEN_0);

}

void PulseStepCount()
{
    PWMGenIntClear(PWM0_BASE, PWM_GEN_0, PWM_INT_CNT_LOAD);
    if (TCA9539Regs_Read16Pin(&TCA9539_IC2, Enable_BLDC2) != 0) // If disable Motor not count (true - disable)
    {
        PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, false);
        return;
    }
    else
        PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
    if (Cnt_StepMotor > Pulse_StepMotor)
    {

        MAP_PWMIntDisable(PWM0_BASE, PWM_INT_GEN_0);
        VrTimer_Compress = 0;
        if (InCompress)
            Cmd_WriteMsg(&Compress_Process_Stop, Null);
        if (InCompressBack)
            Cmd_WriteMsg(&CompressBack_Process_Stop, Null);
    }
    Cnt_StepMotor++;
}
inline void RampSpeed()
{
    if (M_Step == M_ExtractPos)  // Not ramp when compress coffee
        return;
    speedTemp = PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0);
    if (((speedTemp - PWMIncrement) >= speedstep))
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, speedTemp - PWMIncrement);
    else
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, speedstep);
}
void CleanningMachine(void)
{
    uint8_t i = 0;
    for (i = 0; i < NumOfStepClean; i++)
        step_statusClean[i] = St_Pending;
    InHomeReturn = InGrinding = InCompress = InPumping = 0;
    Cl_Step = Cl_ReturnHome1;
    InCleanning = 1;
    TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT); // Interrupt timer
    TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);

}
void CheckingFinish_Cleanning(void)
{

    if (InCleanning)
    {
        switch (Cl_Step)
        {
        case Cl_ReturnHome1:
        case Cl_ReturnHome2:
            RampSpeed();
            if ((TCA9539_IC3.TCA9539_Input.all & LinitSwitch) == 0)
            {
                Cmd_WriteMsg(&HomeReturn_Process_Stop, Null);
                step_statusClean[Cl_Step] = St_Finish;
            }
            break;
        case Cl_InsertPodClean: // Insert POD key clean
            if (InInsertPod && (TCA9539_IC1.TCA9539_Input.all & Clean_Bt) == 0) // Select tha thuoc ve sinh
                step_statusClean[Cl_Step] = St_Finish;

            break;

        case Cl_CompressPos1: // Compress coffee
        case Cl_CompressPos2:
        case Cl_CompressPos3:
            RampSpeed();
            if ((VrTimer_Compress == 0) && InCompress)
                step_statusClean[Cl_Step] = St_Finish;
            break;
        case Cl_Pumping1:
        case Cl_Pumping2:
            if (FinishPumpEvent)
                step_statusClean[Cl_Step] = St_Finish;
            break;

        };
    }
    B_Group_Task = &B2;
}
void CleanProcess()
{
    if (InCleanning)
    {
        switch (Cl_Step)
        {
        case Cl_ReturnHome1:
        case Cl_ReturnHome2:
            if ((!InHomeReturn) && (!delay_flag) && (!InRelay_Timming))
            {
                step_statusClean[Cl_Step] = St_Running;
                HomeReturnTriger = InHomeReturn = 1;
                if (HomePeding != 0)
                {
                    speedstep = 7500; // 10000
                    Cmd_WriteMsg(&HomeReturn_Process_Run, Null);
                }

            }
            if ((InHomeReturn) && (step_statusClean[Cl_Step] == St_Finish))
            {
                InHomeReturn = 0;
                Cl_Step++;
                /*        if (clModeRinse && Cl_Step == Cl_Pumping2)
                 step_statusClean = Cl_FinishCLean;*/
                if (cancel_cmd)
                {
                    Cl_Step = Cl_FinishCLean;
                    cancel_cmd = 0;
                }
            }
            break;
        case Cl_CompressPos1:
            speedstep = 7500;
            PosStep_Compress = stepPos1;
            goto Cl_compresscmd;
        case Cl_CompressPos2:
            speedstep = 9000;
            stepPos2 = 5600;    // 6100
            PosStep_Compress = stepPos2; // stepPos3
            Ron++;
            goto Cl_compresscmd;
        case Cl_CompressPos3:
            speedstep = 8000;
            PosStep_Compress = 500;
            Cl_compresscmd: if (clModeRinse && Cl_Step == Cl_CompressPos1)
            {
                Cl_Step = Cl_CompressPos2;
                step_statusClean[Cl_CompressPos1] = St_Finish;
                step_statusClean[Cl_InsertPodClean] = St_Finish;
                PosStep_Compress = 5600 + stepPos1;

            }
            if ((!InCompress) && (!delay_flag))
            {
                Pos_Compress = 1;
                step_statusClean[Cl_Step] = St_Running;
                InCompress = CompressTriger = 1;
                Cmd_WriteMsg(&Compress_Process_Run, (void*) &Pos_Compress);

            }
            if ((InCompress) && (step_statusClean[Cl_Step] == St_Finish))
            {

                InCompress = 0;
                Delay_20ms(30);
                Cl_Step++;
                if (cancel_cmd)
                    Cl_Step = Cl_ReturnHome2;
            }

            break;

        case Cl_InsertPodClean:
            if (!InInsertPod)
            {
                InInsertPod = InsertPodTrigger = 1;
                step_statusClean[Cl_Step] = St_Running;

            }

            if ((InInsertPod) && (step_statusClean[Cl_Step] == St_Finish))
            {
                Cl_Step++;
                idModeRunning = 3;
            }
            if (cancel_cmd)
            {
                Cl_Step = Cl_ReturnHome2;

            }
            break;

        case Cl_Pumping2:
        case Cl_Pumping1:
            if ((!InPumping) && (!delay_flag))
            {
                step_statusClean[Cl_Step] = St_Running;
                InPumping = PumpingTriger = 1;
                Cmd_WriteMsg(&Pumping_Process_Run, (void*) ModeSelected);
                if (Cl_Step == Cl_Pumping2)
                    InitPumpingEvent();
            }
            if (InPumping)
            {
                if (step_statusClean[Cl_Step] == St_Finish)
                {
                    InPumping = 0;
                    Cl_Step++;
                    Relay_Timming(0);
                    TCA9539Regs_Write16Pin(&TCA9539_IC3,
                    Valve_1 | Valve_3,
                                           false);
                }
            }
            if (cancel_cmd || ((Cl_Step == Cl_Pumping2) && clModeRinse))
            {
                Cmd_WriteMsg(&Pumping_Process_Stop, Null);
            }
            break;
        case Cl_FinishCLean:
            if (InRelay_Timming)
                break;
            InCleanning = cancel_cmd = 0;
            clModeRinse = 0;
            Cl_Step = Cl_ReturnHome1;
            TimerIntDisable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
            QEIIntDisable(QEI0_BASE, QEI_INTTIMER);
            break;
        }
    }
    B_Group_Task = &CheckingFinish_Cleanning;

}

inline void ExtraCompress(void)
{
    if (InPumping)
    {
        TCA9539Regs_Write16Pin(&TCA9539_IC2, Enable_BLDC2, false); // low acitve(conect gnd) Enable
        PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true); // Enable output pwm
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, defaulSpeed); // Freq = 8Khz // speedstep
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
                         PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) / 2);
        VrTimer_Compress = 1;
        TCA9539_IC2.updateOutputFlag = 1;
        Pulse_StepMotor = (uint32_t) (stepPos5 * 2.0f);
        PulseStepInitCount();

    }
}
inline void SweptErrorInRunning(void)
{
    static uint8_t trigger = 0;
    _Bool errorForceCancel = ErNoPumpPulse || ErOutletDetect;
    /*    if (ErHomeReturn && (ErHomeStr == 0))
     {
     TCA9539Regs_Write16Pin(&TCA9539_IC2, Enable_BLDC2, true);
     (vr_ReleaseDelay > 10) ?
     (TCA9539Regs_Write16Pin(&TCA9539_IC2, Enable_BLDC2, false), ErHomeStr =
     1, vr_ReleaseDelay = 0) :
     (vr_ReleaseDelay++);
     }*/
    if (!errorForceCancel)
        trigger = 0;
    if (errorForceCancel & trigger == 0)
    {
        trigger = 1;
        cancel_cmd = 1;
    }

}
inline void eVrTimerReset(void)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
        eVrTimer[i] = 0;
    ErHomeStr = 0;

}
inline void HoldMotorOff(void)
{
    if (!InGrinding)
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_PIN_6); // Turn off grind motor
    /*
     if (!InPumping)
     PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, false);  // Turn off pump motor
     */
}
void SaveCountToEeprom(void)
{
    uint8_t i;
    uint32_t CountData[8];
    //  uint16_t ***PtrToData = &CountDataStorage;
    for (i = 0; i < 8; i++)
    {
        CountData[i] = *(CountDataStorage[i]);
    }
    EEPROMProgram(CountData, DataCountSaveAddress, 8 * 4); // Polling method

}
void ModuleTestMachine(void)
{
    InModuleTest = 1;
    InGrinding = 0;
    ModuleTest();
    TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT); // Interrupt timer
    TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
}
inline void TurnOffTimming()
{
    InModuleTest = TestModuleTrigger = 0;
    TimerIntDisable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
    QEIIntDisable(QEI0_BASE, QEI_INTTIMER);
}
void CheckFinsih_ModuleTest(void)
{
    uint32_t dutyCount;
    if (InModuleTest)
    {
        switch (TargetTest)
        {

        case 0:
            if (VrTimer_Grinding == 0)
            {
                GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_PIN_6);
                TurnOffTimming();
            }
            break;
        case 1:
            dutyCount = speedPumpTest * PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1);
            PWMPulseWidthSet( PWM0_BASE, PWM_OUT_2, dutyCount);
            if (VrTimer_Delay == 0)
            {

                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
                PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, false);
                TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_3, false);
                TurnOffTimming();

            }
            break;
        }
    }
    B_Group_Task = &B2;
}
inline void ReleaseErrorMotor(void)
{
    static uint8_t stage = 0;
    if (!holdShaftMotor)
        return;
    uint16_t errorM = TCA9539Regs_Read16Pin(&TCA9539_IC3,
    WarningPressMotor);
    if ((errorM == 0) && (TriggerShaftRelease == 0))
    {
        TCA9539Regs_Write16Pin(&TCA9539_IC2, Enable_BLDC2, true); //  Disable motor
        TCA9539_IC2.updateOutputFlag = true;
        TriggerShaftRelease = true;
        VrShaftRelease = 0;
        if (M_Step == M_GrindPos || (Cl_Step == Cl_CompressPos1))
            StuckGrindPosError = true;

    }
    if (!TriggerShaftRelease)
        return;
    if (VrShaftRelease >= 5)
    {
        switch (stage)
        {
        case 0:
            TCA9539Regs_Write16Pin(&TCA9539_IC2, Enable_BLDC2, false); // Enalbe motor
            TCA9539_IC2.updateOutputFlag = true;
            VrShaftRelease = 0;
            stage = 1;
            break;
        case 1:
            stage = 0;
            TriggerShaftRelease = false;

            break;
        }
    }
    else
        VrShaftRelease++;

}
void ModuleTest(void)
{
    if (InModuleTest)
        switch (TargetTest)
        {
        case 0:
            if (dirGrindTest)
                TCA9539Regs_Write16Pin(&TCA9539_IC3,
                Direction_BLDC1 | Enable_BLDC1,
                                       true);
            else
                TCA9539Regs_Write16Pin(&TCA9539_IC3,
                Direction_BLDC1 | Enable_BLDC1,
                                       true);
            InGrinding = 1;
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, 0); // Run motor Grind
            VrTimer_Grinding = (uint32_t) (4 / UnitTimer);
            break;
        case 1:

            PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 4000);

            PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);

            //  TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_3, true);
            Delay_20ms(10 / 0.02f);
            break;

        }
}
inline void ManualOpenValve(TCA9539Regs *thisTCA, uint16_t Valve)
{

    bool OpenValve = (TCA9539Regs_Read16Pin(&TCA9539_IC2, Warming_Bt) == 0);
    TCA9539Regs_Write16Pin(thisTCA, Valve, OpenValve);

}
inline void ManualRunPressMotor()
{
    uint16_t DefaultSpeed = 9000;
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, DefaultSpeed);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
                     PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) / 2);
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
}

void ManualControlTest(void)
{

    if (!InManualTest)
        return;
    VrTimer1[holdButtonHome] = 0;
    switch (ManualObjSelected)
    {
#if(ManualMotor)
    case Ma_GrindMotor:
    {

        bool CCW = TCA9539Regs_Read16Pin(&TCA9539_IC3, Warming_Bt);
        bool CW = TCA9539Regs_Read16Pin(&TCA9539_IC3, Rinse_Bt);
        if ((CCW != 0) && (CW != 0))
        {
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, 0);
            break;
        }
        if ((CCW == 0) && (CW != 0))
        {
            TCA9539Regs_Write16Pin(&TCA9539_IC3,
            Direction_BLDC1,
                                   false);
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, true);
        }
        else if ((CW == 0) && (CCW != 0))
        {
            TCA9539Regs_Write16Pin(&TCA9539_IC3,
            Direction_BLDC1,
                                   true);
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, true);
        }

        break;
    }
    case Ma_PumpMotor:
    {
        bool RunningPump =
                (TCA9539Regs_Read16Pin(&TCA9539_IC3, Warming_Bt) == 0);
        if (!RunningPump)
        {
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
            PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, false);

            break;
        }
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 4000);
        uint32_t dutyCount = ManualRun_DutyPump
                * PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, dutyCount);
        PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
        break;
    }
    case Ma_CompressMotor:
    {
        bool CCW = TCA9539Regs_Read16Pin(&TCA9539_IC3, Warming_Bt);
        bool CW = TCA9539Regs_Read16Pin(&TCA9539_IC3, Rinse_Bt);
        if ((CCW != 0) && (CW != 0))
        {
            uint8_t dutyCount = 0;
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, dutyCount);
            PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, false);
            bool DisableMotor = false;
            TCA9539Regs_Write16Pin(&TCA9539_IC2, Enable_BLDC2, DisableMotor); // low acitve (Enable)
            break;
        }
        if ((CCW == 0) && (CW != 0))
        {
            TCA9539Regs_Write16Pin(&TCA9539_IC2, Direction_BLDC2, false);
            ManualRunPressMotor();

        }
        else if ((CW == 0) && (CCW != 0))
        {
            TCA9539Regs_Write16Pin(&TCA9539_IC2, Direction_BLDC2, true);
            ManualRunPressMotor();

        }
        break;
    }
#endif
    case Ma_BrewValve:
        ManualOpenValve(&TCA9539_IC3, Valve_1);
        break;
    case Ma_CoffeeOutletValve:
        ManualOpenValve(&TCA9539_IC3, Valve_2);
        break;
    case Ma_BackRinseValve:
        ManualOpenValve(&TCA9539_IC3, Valve_3);
        break;
    case Ma_SteamInValve:
        ManualOpenValve(&TCA9539_IC3, Valve_4);
        break;
    case Ma_DrainBoilerValve:
        ManualOpenValve(&TCA9539_IC2, Valve_5);
        break;

    };
}

