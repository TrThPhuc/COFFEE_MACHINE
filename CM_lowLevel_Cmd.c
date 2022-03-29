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

uint8_t stepWarming;
#if(PosPress == 0)
#define NumberOfStep  8
#else
#define NumberOfStep  9
#endif
// Number of step
volatile uint8_t step;      // Step count
uint8_t step_status[NumberOfStep];
#define Finish      1
#define Running     2
#define Pending     3

extern bool cancel_cmd, test_step; // Cancel button
static bool cancel_storage;

#define NumOfStepClean 9

enum status step_statusClean[NumOfStepClean];
// Virtual timer for running process
uint32_t VrTimer_Grinding, VrTimer_Compress, VrTimer_Delay, VrTimer_Pumping,
        VrTimer_Relay;
float VrTimer_Compensate, Compensate;
_Bool update;
void MakeCoffee();
void CheckingFinish_StepInRuning();
void MakeCoffeProcess(void);
void WarmUpMachine(void);
void WarmUpProcess(void);
inline void SweptErrorInRunning(void);
inline void eVrTimerReset(void);
//------------------------------------- Process - Grinding coffee into powder-----------------------------------

volatile bool GrindingTriger, InGrinding;
uint32_t FreQ = 0, ds = 0;

float vel;
//------------------------------------- Process - Cluster compress powder coffee----------------------------------

volatile bool CompressTriger, InCompress;
uint32_t Pos_Compress, PosStep_Compress;
uint32_t PitchOfpress;
extern bool HeatingPress;
volatile uint32_t Pulse_StepMotor, Cnt_StepMotor;
void LookUpCompressTime(void);
void PulseStepCount();
void PulseStepInitCount();
inline void RampSpeed();
inline void ExtraCompress();
bool clModeRinse;
//--------------------------------------------- Process - Pumping Hot water -----------------------------------------
volatile bool PumpingTriger, InPumping;
extern volatile bool FinishPumpEvent;
uint16_t SpeedDuty_Pump;
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
//--------------------------------------- Control Level water in steam tank -----------------------------------------
volatile bool LevelControlTriger, InLevelPumping, Hyteresis;
bool SteamReady;
//---------------------------------------
// Delay between process
volatile bool delay_flag = 0;
extern uint32_t Vr_C2Task;
//--------------------------------------- Error variable - eVrtimer -----------------------------------------
_Bool ErOutletDetect, ErHomeReturn, ErHotWaterTimeOut, ErSteamTimeOut;
extern _Bool ErNoPumpPulse;
uint16_t eVrTimer[8];

// ------------------------Share variable and function ------------------------------
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
        // PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 100000);
        //  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 5000);

        Mode_Parameter_t *Mode = (Mode_Parameter_t*) PrPtr;
        GrindingTriger = 0;
        // Set direction motor
        if (Mode->DirGrinding)      // Set Direction_BLDC1
            TCA9539_IC3.TCA9539_Onput.all = ((TCA9539_IC3.TCA9539_Onput.all
                    & ~(Direction_BLDC1)) | Direction_BLDC1);
        else
            // Clear Direction_BLDC1
            TCA9539_IC3.TCA9539_Onput.all = (TCA9539_IC3.TCA9539_Onput.all
                    & ~(Direction_BLDC1));
        // Enable BLDC1 driver
        TCA9539_IC3.TCA9539_Onput.all = ((TCA9539_IC3.TCA9539_Onput.all
                & ~(Enable_BLDC1)) | Enable_BLDC1);

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

        pp1 = pp2 = pp3 = ppo = Compensate = 0;

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
        // Set direction
        TCA9539_IC2.TCA9539_Onput.all = ((TCA9539_IC2.TCA9539_Onput.all
                & ~(Direction_BLDC2)) | Direction_BLDC2);
        // Enable motor driver
        TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                & ~(Enable_BLDC2));     // Enable - not connect to GND
        Stage_Compress = (InProcess) ? step : Cl_Step; // Both of makecoffe and clean have same compress position
        switch (Stage_Compress)
        {
        case 1: // Compress to Pos1 (grind or insert pod position)
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_PIN_6); // Turn off motor grind
            break;
        case 3: // Compress to Pos2 (extraction position)
            /*
             TCA9539_IC3.TCA9539_Onput.all = ((TCA9539_IC3.TCA9539_Onput.all // process compress coffee
             & ~(Valve_2)) | Valve_2);            // Turn on valve 2 (valve brew)
             */
            TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_2, true);
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

// Reset motor compress if fault condition occur in Poscompress after extraction
        if (step == 5 || step == 4) // PosCompress(after extraction)
            TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                    & ~(Enable_BLDC2) | Enable_BLDC2); // Disable connect to gnd - Times 2 for reset warning if fault
        TCA9539_IC2.updateOutputFlag = 1;
    }

}

void Pumping_Process_Run(void *PrPtr)
{

    if (InPumping && PumpingTriger)
    {

        PumpingTriger = Step_Pump = Step_Pump_Cl = Step_Pump_r = 0;
        MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
        // Configure  turn on valve 1// brewValve
        /*        TCA9539_IC3.TCA9539_Onput.all = ((TCA9539_IC3.TCA9539_Onput.all
         & ~(Valve_1) | Valve_1));*/
        TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_1, true);
        TCA9539_IC3.updateOutputFlag = 1;
        if (!InCleanning)
        {
            Mode_Parameter_t *Mode = (Mode_Parameter_t*) PrPtr;
            float volume = Mode->AmountOfWaterPumping.stage_1;

            SpeedDuty_Pump = PreInfusion_pump; // Speed motor pumping in preinfusion stage
            SetVolume = volume;
            VrTimer_FlagRelay = Mode->PreInfusion / UnitTimer; // Time for preinfusion
            flagdelay = 1;  // Used for stage pumping fuction
        }
        else
        {

            SetVolume = CleanVolume;
            SpeedDuty_Pump = HighPressure_Pump;
            if (Cl_Step == Cl_Pumping1)
            {
                flagdelay_Cl = 1;
                VrTimer_FlagRelay_Cl = 1 / UnitTimer;
            }
        }

        // Initialize a pump envent to calculate volume
        FinishPumpEvent = false;

#if (Grindmotor == bldc || Grind == Npwm)
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 8000);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, SpeedDuty_Pump);
#elif(Grindmotor == scooter)
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 8000 / 16);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, SpeedDuty_Pump / 16);
#endif
        PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
        //Configure Direction
        TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                & ~(Direction_BLDC3));
        // Enable motor driver
        TCA9539_IC2.TCA9539_Onput.all = ((TCA9539_IC2.TCA9539_Onput.all
                & ~(Enable_BLDC3)) | Enable_BLDC3);
// Disable motor compress to reset if fault codition in precompress
        TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                & ~(Enable_BLDC2) | Enable_BLDC2); // Disable  driver compress - connect to gnd
        TCA9539_IC2.updateOutputFlag = true;
        Detect = MilliLitresBuffer = 0;

    }
}
void Pumping_Process_Stop(void *PrPtr)
{
    if (InPumping)
    {
        TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                & ~(Enable_BLDC3));
        // Turn off valve 1
        /*        TCA9539_IC3.TCA9539_Onput.all = (TCA9539_IC3.TCA9539_Onput.all
         & ~(Valve_1));*/
        TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_1, false);
        TCA9539_IC3.updateOutputFlag = true;
        totalMilliLitres = MilliLitresBuffer;
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
        PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, false);
        FinishPumpEvent = true;

    }

}
void SteamLevelControl_Run(void *PrPtr)
{

    if (InLevelPumping && LevelControlTriger)
    {
        LevelControlTriger = 0;
        /*        TCA9539_IC2.TCA9539_Onput.all = ((TCA9539_IC2.TCA9539_Onput.all
         & ~(Valve_5)) | Valve_5);*/
        TCA9539Regs_Write16Pin(&TCA9539_IC2, Valve_5, true);
        TCA9539_IC2.updateOutputFlag = 1;

    }
}
void SteamLevelControl_Stop(void *PrPtr)
{
    if (InLevelPumping)
    {
        InLevelPumping = 0;
        /*        TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC3.TCA9539_Onput.all
         & ~(Valve_5));*/
        TCA9539Regs_Write16Pin(&TCA9539_IC2, Valve_5, false);
        TCA9539_IC2.updateOutputFlag = 1;

    }

}
void HomeReturn_Process_Run(void *PrPtr)
{
    if (InHomeReturn && HomeReturnTriger)
    {

        HomeReturnTriger = 0;

        // Set direction
        TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                & ~(Direction_BLDC2));  // Set LOW
        // Enable motor driver
        TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                & ~(Enable_BLDC2));         // Enable - Not connect to gnd
        TCA9539_IC2.updateOutputFlag = 1;
        PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);

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
    TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);

//----------------------------------------- Timing grind coffee fuction-------------------------------------------------
    if (InGrinding)
    {

        if (VrTimer_Grinding != 0)
        {

            VrTimer_Grinding--;
            ppo++;
            ppi = (float) ppo * UnitTimer;
        }
        if (ppi <= 15.0f)       // Limit time for grind coffee
        {
            if ((VrTimer_Grinding == 0) && (VrTimer_Compensate > 0)
                    && (cancel_cmd != 1))
            {

                VrTimer_Grinding += (uint32_t) VrTimer_Compensate;
                Compensate += VrTimer_Compensate;
                VrTimer_Compensate = 0;
            }
        }
        else
            VrTimer_Grinding = 0;

    }
//---------------------------------------------------- Timming delay fuction---------------------------------------------------
    if (delay_flag)
    {
        (VrTimer_Delay != 0) ? (VrTimer_Delay--) : (delay_flag = 0);

    }
//----------------------------------Timming release pressure & close brew valve 2 fuction --------------------------------------
    if ((step_status[4] == Finish || step_statusClean[Cl_Pumping1] == St_Finish)
            && (InRelay_Timming == 1))
    {
        if (VrTimer_Relay != 0)
            VrTimer_Relay--;
        else
        {
            switch (Step_Relay)
            {
            case 0:

                VrTimer_Relay = 50;
                /*
                 TCA9539_IC3.TCA9539_Onput.all = ((TCA9539_IC3.TCA9539_Onput.all
                 & ~(Valve_3)) | Valve_3); //Open Valve 3 to release water pressure in tank(after pumping)
                 */
                TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_3, true);
                Step_Relay++;
                break;
            case 1:
                VrTimer_Relay = 50;
                /*
                 TCA9539_IC3.TCA9539_Onput.all = (TCA9539_IC3.TCA9539_Onput.all
                 & ~(Valve_3)); // Close Bypass valve
                 */
                TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_3, false);
                Step_Relay++;
                break;
            case 2:
                InRelay_Timming = 0;
                break;
            }
            TCA9539_IC3.updateOutputFlag = 1;
        }

    }
//----------------------------------------------- Timing for count extraction time ----------------------------------------------
    if (triggerCount_ExtractionTime)
        CoffeeExtractionTime++;
//--------------------------------- PreInfution & full extraction stage in pumping process  -------------------------------------
    if (flagdelay && (step_status[4] == Running))
    {
        if (VrTimer_FlagRelay != 0)
            VrTimer_FlagRelay--;
        else
        {
            switch (Step_Pump)
            {
            case 0: // Preinfustion extraction stage -

                VrTimer_FlagRelay = infu / UnitTimer;
                // Hold motor compress in full extraction
                ExtraCompress();
                Step_Pump++;
                break;

            case 1: // Full extraction stage
                SpeedDuty_Pump = HighPressure_Pump;
                VrTimer_FlagRelay = 50;
#if (Grindmotor == bldc || Grind == Npwm)
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, SpeedDuty_Pump);

#elif(Grindmotor == scooter)
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, SpeedDuty_Pump / 16);
#endif//

                TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                        & ~(Direction_BLDC3));
                // Enable motor driver
                TCA9539_IC2.TCA9539_Onput.all = ((TCA9539_IC2.TCA9539_Onput.all
                        & ~(Enable_BLDC3)) | Enable_BLDC3);
                TCA9539_IC2.updateOutputFlag = true;
                PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
                Step_Pump++;
                flagdelay = 0;
                break;
            }
            TCA9539_IC3.updateOutputFlag = true;
        }
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
                switch (Step_Pump_r)
                {
                case 1:
                case 5:
                    /*                    TCA9539_IC3.TCA9539_Onput.all =
                     ((TCA9539_IC3.TCA9539_Onput.all & ~(Valve_2)
                     | Valve_2));
                     TCA9539_IC3.TCA9539_Onput.all =
                     ((TCA9539_IC3.TCA9539_Onput.all & ~(Valve_1)
                     | Valve_1));*/
                    TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_2 | Valve_1,
                    true);
                    infu_cl = 2;
                    VrTimer_FlagRelay_Cl = infu_cl / UnitTimer;
                    Step_Pump_r++;
                    break;
                case 0: // generate pressure
                case 2:
                case 4:
                    infu_cl = 20;
                    VrTimer_FlagRelay_Cl = infu_cl / UnitTimer;
                    /*                    TCA9539_IC3.TCA9539_Onput.all =
                     (TCA9539_IC3.TCA9539_Onput.all // Open valve 1
                     & ~(Valve_2));
                     TCA9539_IC3.TCA9539_Onput.all =
                     ((TCA9539_IC3.TCA9539_Onput.all & ~(Valve_1)
                     | Valve_1));*/
                    TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_1, true);
                    TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_2, false);
                    Step_Pump_r++;

                    break;

                case 3:
                case 6:
                    infu_cl = 4;
                    VrTimer_FlagRelay_Cl = infu_cl / UnitTimer;
                    /*                    TCA9539_IC3.TCA9539_Onput.all =
                     (TCA9539_IC3.TCA9539_Onput.all // Open valve 1
                     & ~(Valve_2));
                     TCA9539_IC3.TCA9539_Onput.all =
                     (TCA9539_IC3.TCA9539_Onput.all & ~(Valve_1));*/
                    TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_2 | Valve_1,
                    false);
                    Step_Pump_r++;
                    break;
                }

                if (Step_Pump_r > 6)
                {
                    Step_Pump_r = 0;
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
                VrTimer_FlagRelay_Cl = 5 / UnitTimer;

                TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                        & ~(Direction_BLDC3));
                // Enable motor driver
                TCA9539_IC2.TCA9539_Onput.all = ((TCA9539_IC2.TCA9539_Onput.all
                        & ~(Enable_BLDC3)) | Enable_BLDC3);
                /*                TCA9539_IC3.TCA9539_Onput.all = ((TCA9539_IC3.TCA9539_Onput.all
                 & ~(Valve_1) | Valve_1));             // open valve 1

                 TCA9539_IC3.TCA9539_Onput.all = ((TCA9539_IC3.TCA9539_Onput.all
                 & ~(Valve_2) | Valve_2));  */           // open valve 2
                TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_2 | Valve_1,
                true);
                MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, SpeedDuty_Pump);
                MAP_PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
                Step_Pump_Cl++;
                break;
            case 2:
            case 4:
            case 6:
            case 8:

                VrTimer_FlagRelay_Cl = 2 / UnitTimer;
                /*
                 TCA9539_IC3.TCA9539_Onput.all = (TCA9539_IC3.TCA9539_Onput.all
                 & ~(Valve_1)); // close valve 1
                 */
                TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_1, false);
                Step_Pump_Cl++;
                break;
                // close valve 2
            case 10:
                TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_1 | Valve_2, false);
                flagdelay_Cl = Step_Pump_Cl = 0;
                Pumping_Process_Stop(Null);
                break;
            }
            TCA9539_IC3.updateOutputFlag = true;
        }

    }
//-------------------------------------------------------------------------------------------------------------------------------
}
#if(PosPress == 0)
void CheckingFinish_StepInRuning()
{

    if (InProcess || InStartUp)
    {
        switch (step)
        {
        case 0:
        case 5: // Home return
            /*
             speedTemp = PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0);
             if ((speedTemp - PWMIncrement) >= MaxSpeed)
             {
             PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, speedTemp - PWMIncrement);
             }
             */
            if ((TCA9539_IC3.TCA9539_Input.all & LinitSwitch) == 0)
            {
                Cmd_WriteMsg(&HomeReturn_Process_Stop, Null);
                step_status[step] = Finish;

            }
            break;
        case 2: // Gringding coffee
            if (InGrinding)
            {

                if (VrTimer_Grinding == 0)
                {
                    Cmd_WriteMsg(&Grinding_Process_Stop, Null);
                    step_status[step] = Finish;

                }

            }
            if (cancel_cmd)
            {
                VrTimer_Grinding = 0;
                cancel_storage = 1;
            }

            break;
        case 1: // Compress coffee
        case 3:
        case 6:
            if ((VrTimer_Compress == 0) && InCompress)
            {
                step_status[step] = Finish;

            }

            break;
        case 4:
//        case 5:   // pump hot water
            if ((TCA9539_IC3.TCA9539_Input.all & LevelSensor2) == 0
                    && (Detect == 0) && (Step_Pump == 2))
            {
                fillterSignal++;
                if (fillterSignal >= 15)
                {
                    InitPumpingEvent();
                    fillterSignal = 0;
                    Detect = 1;
                }

            }
            else
                fillterSignal = 0;

            if (FinishPumpEvent)
            {

                step_status[step] = Finish;

            }
            break;

        };

    }
    B_Group_Task = &B2;

}
#else
void CheckingFinish_StepInRuning()
{

    if (InProcess)
    {
        switch (step)
        {
//========================================================= Home Return ===========================================
        case 0:
        case 6:
            RampSpeed();
            if ((TCA9539_IC3.TCA9539_Input.all & LinitSwitch) == 0)
            {
                Cmd_WriteMsg(&HomeReturn_Process_Stop, Null);
                step_status[step] = Finish;
                eVrTimer[eVrHomeReturn] = 0;
                ErHomeReturn = false;

            }
            else if (InHomeReturn)
                (eVrTimer[eVrHomeReturn] > 200) ? // If in return home but not detect home limit signal in 4s will error
                (ErHomeReturn = true) : (eVrTimer[eVrHomeReturn]++);
            break;
//======================================================== Gringding coffee =========================================
        case 2:
            if (InGrinding)
            {

                if (VrTimer_Grinding == 0)
                {
                    Cmd_WriteMsg(&Grinding_Process_Stop, Null);
                    step_status[step] = Finish;

                }

            }
            if (cancel_cmd)
            {
                VrTimer_Grinding = 0;
                cancel_storage = 1;
            }
            break;
//=================================================== Compress coffee ================================================
        case 1:
        case 3:
        case 5:
        case 7:
            RampSpeed();    // Use for ramping speed of motor
            if ((VrTimer_Compress == 0) && InCompress) // Monitor pulse in PulseStepCount func and stop motor when enough pulse
                step_status[step] = Finish;
            break;

//======================================================== Pumping water =============================================
        case 4:
            if (Step_Pump == 2) // Step full extraction - After infusion
            {
//---------------------------------------------- Filter sensor outlet signal - 0.3s -------------------------------
                if ((TCA9539_IC3.TCA9539_Input.all & LevelSensor2) == 0
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
                if ((TCA9539_IC3.TCA9539_Input.all & LevelSensor2) != 0
                        && Detect == 0)
                {
                    if (!ErOutletDetect)
                    {
                        (eVrTimer[eVrCoffeOutlet] > 200) ? // If never detect in 5s full extraction stage will error
                        (ErOutletDetect = true) : (eVrTimer[eVrCoffeOutlet]++);
                    }
                    else
                        eVrTimer[eVrCoffeOutlet] = 0;
                }
                else
                {
                    eVrTimer[eVrCoffeOutlet] = 0;
                    ErOutletDetect = false;
                }

            }
            if (FinishPumpEvent)
                step_status[step] = Finish;
            break;
        };
        SweptErrorInRunning();
    }
    B_Group_Task = &B2;

}
#endif

void MakeCoffee()
{
    uint8_t i;
    for (i = 0; i < NumberOfStep; i++)
        step_status[i] = Pending;
    InHomeReturn = InGrinding = InCompress = InPumping = 0;
// Vr_C2Task = 100;
    InProcess = 1;
    firstCup = false;
    if (test_step == 0)
        step = 0;
    Gui_CoffeExtractionTime = 0;
    InWarming = delay_flag = ppi = 0;
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
#if(PosPress == 0)
void MakeCoffeProcess(void)
{

    if (InProcess)
    {
        switch (step)
        {
        case 0:
        case 5: // Home return compress block
            if ((!InHomeReturn) && (!delay_flag) && (!InRelay_Timming))
            {
                step_status[step] = Running;
                HomeReturnTriger = InHomeReturn = 1;
                if (HomePeding != 0)
                {
                    speedstep = 12000; // 12000
                    Cmd_WriteMsg(&HomeReturn_Process_Run, Null);
                }

            }

            if ((InHomeReturn) && (step_status[step] == Finish))
            {
                InHomeReturn = 0;
                (cancel_cmd) ?
                        (step = 6, cancel_cmd = 0, cancel_storage = 1, ) :
                        (step++);

                Delay_20ms(50);

            }
            break;
        case 2: // Grinding coffee
            if ((!InGrinding) && (!delay_flag))
            {
                step_status[step] = Running;
                GrindingTriger = InGrinding = 1;
                Cmd_WriteMsg(&Grinding_Process_Run, (void*) ModeSelected);

            }
            if ((InGrinding) && (step_status[step] == Finish))
            {

                InGrinding = 0;
                step++;
                Delay_20ms(25);
                if (test_step)
                    InProcess = step = test_step = cancel_cmd = cancel_storage =
                            0;

            }
            break;
        case 1:
        case 3:
        case 6:

            if ((!InCompress) && (!delay_flag))
            {
                step_status[step] = Running;
                InCompress = CompressTriger = 1;
                (step == 1) ?
                        (Pos_Compress = pos1, PosStep_Compress = stepPos1, speedstep =
                                12000) : //speedstep =12000
                        (LookUpCompressTime(), Pos_Compress = pos2, PosStep_Compress =
                                stepPos2, speedstep = 28000); // speedstep = 28000
                if (step == 6)
                {
                    speedstep = 12000;
                    Pos_Compress = 45;
                    PosStep_Compress = stepPos3; // stepPos3
                }

                Cmd_WriteMsg(&Compress_Process_Run, (void*) &Pos_Compress);

            }
            if ((InCompress) && (step_status[step] == Finish))
            {

                InCompress = 0;
                if (cancel_cmd)
                {
                    step = 5;
                    cancel_cmd = 0;
                    cancel_storage = 1;
                    goto skip_compress;

                }
                (step == 3 || step == 1) ? (Delay_20ms(25)) : (Delay_20ms(0));

                step++;
                skip_compress: ;

            }
            break;
        case 4:
            //       case 5:
            if ((!InPumping) && (!delay_flag))
            {
                step_status[step] = Running;
                InPumping = PumpingTriger = 1;
                Cmd_WriteMsg(&Pumping_Process_Run, (void*) ModeSelected);
                triggerCount_ExtractionTime = 1;
                CoffeeExtractionTime = timesH = 0;
            }
            if (InPumping)
            {
                Gui_CoffeExtractionTime = CoffeeExtractionTime * UnitTimer;
                if (step_status[step] == Finish)
                {
                    triggerCount_ExtractionTime = 0;
                    InPumping = 0;
                    step++;
                    if (test_step)
                    {
                        step = 7;
                        test_step = 0;

                    }
                    if (cancel_cmd)
                    {
                        step = 5;
                        cancel_cmd = 0;
                        cancel_storage = 1;
                    }
                    Relay_Timming(25);
                    //  Delay_20ms(100);
                }

                if (cancel_cmd)
                {
                    Cmd_WriteMsg(&Pumping_Process_Stop, Null);
                    cancel_storage = 1;

                }

            }
            break;
        case 7:
            if (step_status[4] == Finish)
            {
                if (InRelay_Timming == 0)
                {
                    step_status[step] = Finish;
                    (!cancel_storage) ?
                            (ModeSelected->Cups++) : (cancel_storage = 0);
                    if (timesH < 1)
                    {
                        step = 5;
                        timesH++;
                    }
                    else
                    {
                        InProcess = step = 0, cancel_cmd = 0;
                        //calibWeightFlag = 0;
                        TimerIntDisable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
                        QEIIntDisable(QEI0_BASE, QEI_INTTIMER);
                    }

                }
                else
                    step_status[step] = Running;
            }
            else
            {
                InProcess = step = 0, cancel_cmd = 0;
                //calibWeightFlag = 0;
                TimerIntDisable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
                QEIIntDisable(QEI0_BASE, QEI_INTTIMER);

            }

            break;

        default:
            InProcess = step = 0;

        };

    }
    B_Group_Task = &CheckingFinish_StepInRuning;
}
#else
void MakeCoffeProcess(void)
{

    if (InProcess)
    {
        switch (step)
        {
        case 0:
        case 6: // Home return compress block
            if ((!InHomeReturn) && (!delay_flag) && (!InRelay_Timming))
            {
                step_status[step] = Running;
                HomeReturnTriger = InHomeReturn = 1;
                if (HomePeding != 0)
                {
                    speedstep = 7500; // 10000
                    Cmd_WriteMsg(&HomeReturn_Process_Run, Null);
                }

            }

            if ((InHomeReturn) && (step_status[step] == Finish))
            {
                InHomeReturn = 0;
                (cancel_cmd) ?
                        (step = 8, cancel_cmd = 0, cancel_storage = 1) :
                        (step++);

                if (step == 1)
                    Delay_20ms(25);
                if (timesH == 2)
                    step = 8;
            }
            break;
        case 2: // Grinding coffee
            if ((!InGrinding) && (!delay_flag))
            {
                step_status[step] = Running;
                GrindingTriger = InGrinding = 1;
                Cmd_WriteMsg(&Grinding_Process_Run, (void*) ModeSelected);

            }
            if ((InGrinding) && (step_status[step] == Finish))
            {
                if (!fullOfGroundsDrawer)
                    countGrounds++;
                InGrinding = 0;
                Delay_20ms(25);
                if (test_step)
                    InProcess = step = test_step = cancel_cmd = cancel_storage =
                            0;
                if (calibWeightFlag)
                {
                    step = 6;
                    break;
                }
                step++;
            }
            break;
        case 1:
            PosStep_Compress = stepPos1;
            speedstep = 7500; // 12000
            goto compresscmd;
        case 3:
            LookUpCompressTime();
            PosStep_Compress = stepPos2;
            speedstep = 9000; //18000
            goto compresscmd;
        case 5:
            PosStep_Compress = stepPos4;
            speedstep = 13500; // 18000
            goto compresscmd;
        case 7:
            speedstep = 9000; //12000
            PosStep_Compress = stepPos3; // stepPos3

            compresscmd: if ((!InCompress) && (!delay_flag))
            {
                Pos_Compress = 1;
                step_status[step] = Running;
                InCompress = CompressTriger = 1;
                Cmd_WriteMsg(&Compress_Process_Run, (void*) &Pos_Compress);

            }
            if ((InCompress) && (step_status[step] == Finish))
            {

                InCompress = 0;
                if (cancel_cmd)
                {
                    step = 6;
                    cancel_cmd = 0;
                    cancel_storage = 1;
                    goto skip_compress;

                }
                (step == 3 || step == 1) ? (Delay_20ms(30)) : (Delay_20ms(0));
                step++;
                skip_compress: ;

            }
            break;
        case 4:
            if ((!InPumping) && (!delay_flag))
            {
                step_status[step] = Running;
                InPumping = PumpingTriger = 1;
                Cmd_WriteMsg(&Pumping_Process_Run, (void*) ModeSelected);
                triggerCount_ExtractionTime = 1;
                CoffeeExtractionTime = timesH = 0;
            }
            if (InPumping)
            {
                Gui_CoffeExtractionTime = CoffeeExtractionTime * UnitTimer;
                if (step_status[step] == Finish)
                {
                    triggerCount_ExtractionTime = 0;
                    InPumping = 0;
                    step++;
                    if (cancel_cmd)
                    {

                        cancel_cmd = 0;
                        cancel_storage = 1;
                    }
                    /*
                     TCA9539_IC3.TCA9539_Onput.all =
                     (TCA9539_IC3.TCA9539_Onput.all & ~(Valve_2)); // Close cofffe outlet
                     */
                    TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_2, false);
                    Relay_Timming(25);
                    (step == 5) ? Delay_20ms(25) : Delay_20ms(0);
                }
                if (cancel_cmd)
                {
                    Cmd_WriteMsg(&Pumping_Process_Stop, Null);
                    cancel_storage = 1;

                }

            }
            break;
        case 8:
            if (step_status[4] == Finish)
            {
                if (InRelay_Timming == 0)
                {
                    step_status[step] = Finish;
                    if (!cancel_storage)
                    {
                        ModeSelected->Cups++;
                        ModeSelected->WeigtOfPowder = Gui_CoffeExtractionTime;
                    }
                    if (timesH < 2)
                    {
                        step = 6;
                        timesH++;
                    }
                    else
                    {
                        timesH = 0;
                        InProcess = cancel_storage = step = 0, cancel_cmd = 0;
                        calibWeightFlag = 0;
                        TimerIntDisable(TIMER4_BASE,
                        TIMER_TIMA_TIMEOUT);
                        QEIIntDisable(QEI0_BASE, QEI_INTTIMER);
                    }

                }
                else
                    step_status[step] = Running;
            }
            else
            {
                if (timesH < 2)
                {
                    step = 6;
                    timesH++;
                    break;
                }
                InProcess = step = 0, cancel_cmd = 0;
                timesH = 0;
                if (calibWeightFlag)
                    calibWeightStr = 1;
                calibWeightFlag = cancel_storage = 0;
                TimerIntDisable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
                QEIIntDisable(QEI0_BASE, QEI_INTTIMER);

            }

            break;

        default:
            InProcess = step = 0;

        };

    }
    B_Group_Task = &CheckingFinish_StepInRuning;
}

#endif
void WarmUpMachine()
{
    uint8_t i;
    for (i = 0; i < NumberOfStep; i++)
    {
        step_status[i] = Pending;
    }
    InHomeReturn = InCompress = 0;
    InStartUp = 1;
    step = 0;
    TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT); // Interrupt timer
    TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
}
void WarmUpProcess()
{
    if (InStartUp && SteamReady)
    {
        switch (step)

        {
        case 0:
            /*            TCA9539_IC3.TCA9539_Onput.all = ((TCA9539_IC3.TCA9539_Onput.all
             & ~(Valve_4)) | Valve_4);*/

            TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_4, true);
            Delay_20ms(8000);
            step++;
            break;
        case 1:
            if (!delay_flag)
            {
                InStartUp = step = cancel_cmd = 0;
                /*                TCA9539_IC3.TCA9539_Onput.all = (TCA9539_IC3.TCA9539_Onput.all
                 & ~(Valve_4));*/
                TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_4, false);
                HeatingPress = 1;
                TimerIntDisable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
            }
            if (cancel_cmd)
                VrTimer_Delay = 0;
            break;
        };

    }
    B_Group_Task = &CheckingFinish_StepInRuning;
}
void WarmingPressMachine(void *P)
{
    if (WarmingEnalbe)
    {
        InWarming = 1;
        stepWarming = 0;
        TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT); // Interrupt timer
        TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
    }
}
void WarmingPressProcess(void)
{
    static uint32_t stepWarming;
    if (InWarming && SteamReady)
    {
        switch (stepWarming)
        {
        case 0:
            /*            TCA9539_IC3.TCA9539_Onput.all = ((TCA9539_IC3.TCA9539_Onput.all
             & ~(Valve_4)) | Valve_4);*/
            TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_4, true);
            Delay_20ms(500);
            stepWarming++;
            break;
        case 1:
            if (!delay_flag)
            {
                InWarming = stepWarming = 0;
                /*                TCA9539_IC3.TCA9539_Onput.all = (TCA9539_IC3.TCA9539_Onput.all
                 & ~(Valve_4));*/
                TCA9539Regs_Write16Pin(&TCA9539_IC3, Valve_4, false);
                TimerIntDisable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
            }

        }
    }
    B_Group_Task = &B2;
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
    MAP_PWMGenIntClear(PWM0_BASE, PWM_GEN_0, PWM_INT_CNT_LOAD);
    MAP_PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_0, PWM_INT_CNT_LOAD);
    MAP_PWMIntEnable(PWM0_BASE, PWM_INT_GEN_0);

}
void PulseStepCount()
{
    PWMGenIntClear(PWM0_BASE, PWM_GEN_0, PWM_INT_CNT_LOAD);
    if (Cnt_StepMotor > Pulse_StepMotor)
    {
        //PWMGenIntTrigDisable(PWM0_BASE, PWM_GEN_0, PWM_INT_CNT_LOAD);
        MAP_PWMIntDisable(PWM0_BASE, PWM_INT_GEN_0);
        VrTimer_Compress = 0;
        Cmd_WriteMsg(&Compress_Process_Stop, Null);
    }

    Cnt_StepMotor++;

}
inline void RampSpeed()
{
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
            if (InInsertPod && (TCA9539_IC1.TCA9539_Input.all & BT9) == 0) // Select tha thuoc ve sinh
                step_statusClean[Cl_Step] = St_Finish;

            break;

        case Cl_CompressPos1: // Compress coffee
        case Cl_CompressPos2:
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
                if (cancel_cmd)
                {
                    Cl_Step = Cl_FinishCLean;
                    cancel_cmd = 0;
                }

            }
            break;
        case Cl_CompressPos1:
            speedstep = 7500; //12000
            PosStep_Compress = stepPos1;
            goto Cl_compresscmd;
        case Cl_CompressPos2:
            speedstep = 9000; //12000
            stepPos2 = 6000;
            PosStep_Compress = stepPos2; // stepPos3

            Cl_compresscmd: if (clModeRinse)
            {
                Cl_Step = Cl_CompressPos2;
                step_statusClean[Cl_CompressPos1] = St_Finish;
                step_statusClean[Cl_InsertPodClean] = St_Finish;
                PosStep_Compress = 5800 + stepPos1;

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
                    if (Cl_Step == Cl_ReturnHome2)
                        Relay_Timming(25);
                }

            }
            if (cancel_cmd)
            {
                Cmd_WriteMsg(&Pumping_Process_Stop, Null);
                cancel_storage = 1;

            }
            break;
        case Cl_FinishCLean:
            InCleanning = cancel_cmd = 0;
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
        TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                & ~(Enable_BLDC2));     // Enable - not connect to GND
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
    _Bool error = ErNoPumpPulse || ErOutletDetect || ErHomeReturn;
    if (!error)
        trigger = 0;
    if (error & trigger == 0)
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

}
inline void HoldMotorOff(void)
{
    if (!InGrinding)
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_PIN_6); // Turn off grind motor
    if (!InPumping)
        PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, false);  // Turn off pump motor
}

