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
volatile bool InProcess, FinishProcess, InStartUp, InWarming;
uint8_t stepWarming;
#define NumberOfStep  8     // Number of step
volatile uint8_t step;      // Step count
uint8_t step_status[NumberOfStep];
#define Finish      1
#define Running     2
#define Pending     3
extern bool cancel_cmd, test_step; // Cancel button
static bool cancel_storage;

// Virtual timer for running process
uint32_t VrTimer_Grinding, VrTimer_Compress, VrTimer_Delay, VrTimer_Pumping,
        VrTimer_Relay;
float VrTimer_Compensate, Compensate;
_Bool update;
void MakeCoffee();
void CheckingFinish_StepInRuning();
void MakeCoffeProcess(void);
void StartUpMachine(void);
void StartUpProcess(void);
//---------------------------------------
// Process - Grinding coffee into powder
volatile bool GrindingTriger, InGrinding;
void InitFeedbackVel();
void FeedbackVelGrind(void);
uint32_t FreQ = 0, ds = 0;
float vel;
//---------------------------------------
// Process - Cluster compress powder coffee
volatile bool CompressTriger, InCompress;
uint32_t Pos_Compress, PosStep_Compress;
uint32_t PitchOfpress;
volatile uint32_t Pulse_StepMotor, Cnt_StepMotor;
void LookUpCompressTime(void);
void PulseStepCount();
void PulseStepInitCount();
//---------------------------------------
// Process - Pumping Hot water
volatile bool PumpingTriger, InPumping;
extern volatile bool FinishPumpEvent;
uint16_t SpeedDuty_Pump;
bool calibVolumeFlag, calibVolumeStr;
uint16_t fillterSignal = 0;
extern void InitPumpingEvent();
extern float SetVolume, PulWeightRatio;
// Millitre pumping water
extern volatile float totalMilliLitres, MilliLitresBuffer;
// Relay Timming for turn on/off Valve in Pumping process
volatile bool InRelay_Timming;
volatile uint8_t Step_Relay, Step_Pump;
//---------------------------------------
// Process- Return home
volatile bool HomeReturnTriger, InHomeReturn;
uint8_t timesH = 0;
//---------------------------------------
// Control Level water in steam tank
volatile bool LevelControlTriger, InLevelPumping, Hyteresis;
bool SteamReady;
//---------------------------------------
// Delay between process
volatile bool delay_flag = 0;
extern uint32_t Vr_C2Task;
// ------------------------Share variable and function ------------------------------
extern Mode_Parameter_t *ModeSelected;
extern TCA9539Regs TCA9539_IC1, TCA9539_IC2, TCA9539_IC3;
#define HomePeding  (TCA9539_IC3.TCA9539_Input.all & LinitSwitch)   // Read LimitSwitch

extern void (*B_Group_Task)(void);   // B task 5ms
extern void B2(void);
volatile bool flagdelay = 0;
uint16_t VrTimer_FlagRelay = 100, ph = 100;
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
#if(Grindmotor == scooter)
        SysCtlPWMClockSet(SYSCTL_PWMDIV_16);
#endif

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
        TCA9539_IC3.TCA9539_Onput.all = (TCA9539_IC3.TCA9539_Onput.all
                & ~(Valve_4));
        TCA9539_IC3.updateOutputFlag = 1;
        InitFeedbackVel();
        // Enable PWM

        // PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 80000);   //1Khz

        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 100000);
        PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
        //PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, speedgrind);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 8500);   //8500

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
        TCA9539_IC3.updateOutputFlag = 1;
#if(Grindmotor == bldc)
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 0);
        PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, false);

#else if (Grindmotor == scooter)
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 5000);
#endif

    }

}
void Compress_Process_Run(void *PrPtr)
{
    if (InCompress && CompressTriger)
    {
#if(Grindmotor == scooter)
        SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
#endif
        CompressTriger = 0;
        // Set direction
        TCA9539_IC2.TCA9539_Onput.all = ((TCA9539_IC2.TCA9539_Onput.all
                & ~(Direction_BLDC2)) | Direction_BLDC2);
        // Enable motor driver
        TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                & ~(Enable_BLDC2));     // Enable - not connect to GND
        switch (step)
        {
        case 1:

            /*            TCA9539_IC3.TCA9539_Onput.all = ((TCA9539_IC3.TCA9539_Onput.all
             & ~(Valve_4)) | Valve_4);*/
            break;
        case 3:
            TCA9539_IC3.TCA9539_Onput.all = ((TCA9539_IC3.TCA9539_Onput.all
                    & ~(Valve_2)) | Valve_2);
            TCA9539_IC3.TCA9539_Onput.all = (TCA9539_IC3.TCA9539_Onput.all
                    & ~(Valve_4));
            break;

        }
        PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true); // Enable output pwm
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, speedstep);   // Freq = 8Khz
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
                         PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) / 3);
        VrTimer_Compress = *(uint32_t*) PrPtr;
        TCA9539_IC2.updateOutputFlag = 1;
        Pulse_StepMotor = PosStep_Compress;
        PulseStepInitCount();

    }
}
void Compress_Process_Stop(void *PrPtr)
{
    if (InCompress)
    {
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);
        PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, false);
        TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                & ~(Enable_BLDC2) | Enable_BLDC2);     // disable connect to gnd
        TCA9539_IC2.updateOutputFlag = 1;

    }
}
void Pumping_Process_Run(void *PrPtr)
{

    if (InPumping && PumpingTriger)
    {
#if(Grindmotor == scooter)
        SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
#endif
        PumpingTriger = Step_Pump = 0;
        // InPumping = 1;
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
        // Configure  turn on valve 1, valve 2
        TCA9539_IC3.TCA9539_Onput.all = ((TCA9539_IC3.TCA9539_Onput.all
                & ~(Valve_1) | Valve_1));
        TCA9539_IC3.TCA9539_Onput.all = (TCA9539_IC3.TCA9539_Onput.all
                & ~(Valve_2));
        TCA9539_IC3.updateOutputFlag = 1;
        Mode_Parameter_t *Mode = (Mode_Parameter_t*) PrPtr;

        /*        float volume =
         (Mode->WeigtOfPowder * Mode->AmountOfWaterPumping.stage_1)
         / (float) PulWeightRatio );*/
        float volume = Mode->AmountOfWaterPumping.stage_1;

        SpeedDuty_Pump = PreInfusion_pump;
        // Select calib mode or general mode
        SetVolume = (!calibVolumeFlag) ? volume : MaxVolume
        // Initialize a pump envent to calculate volume
        FinishPumpEvent = false;
        flagdelay = 1;  // Used for process pumping
        VrTimer_FlagRelay = Mode->PreInfusion / UnitTimer;     // ph = 100
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, SpeedDuty_Pump);
        PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
        //
        TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                & ~(Direction_BLDC3));
        // Enable motor driver
        TCA9539_IC2.TCA9539_Onput.all = ((TCA9539_IC2.TCA9539_Onput.all
                & ~(Enable_BLDC3)) | Enable_BLDC3);
        TCA9539_IC2.updateOutputFlag = true;
        Detect = MilliLitresBuffer = 0;
        B_Group_Task = &CheckingFinish_StepInRuning;
        //
    }
}
void Pumping_Process_Stop(void *PrPtr)
{
    if (InPumping)
    {
        //Mode_Parameter_t *Mode = (Mode_Parameter_t*) PrPtr;
        TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                & ~(Enable_BLDC3));
        // Turn off valve 1
        TCA9539_IC3.TCA9539_Onput.all = (TCA9539_IC3.TCA9539_Onput.all
                & ~(Valve_1));
        TCA9539_IC3.TCA9539_Onput.all = (TCA9539_IC3.TCA9539_Onput.all
                & ~(Valve_2));
        TCA9539_IC3.updateOutputFlag = true;
        totalMilliLitres = MilliLitresBuffer;
        // MilliLitresBuffer = 0;
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
        PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, false);
        if (calibVolumeFlag)
        {
            ModeSelected->AmountOfWaterPumping.stage_1 = totalMilliLitres;
            calibVolumeStr = 1;
        }
        FinishPumpEvent = true;

    }

}
void SteamLevelControl_Run(void *PrPtr)
{

    if (InLevelPumping && LevelControlTriger)
    {
        LevelControlTriger = 0;
        TCA9539_IC2.TCA9539_Onput.all = ((TCA9539_IC2.TCA9539_Onput.all
                & ~(Valve_5)) | Valve_5);
        TCA9539_IC2.updateOutputFlag = 1;

    }
}
void SteamLevelControl_Stop(void *PrPtr)
{
    if (InLevelPumping)
    {
        InLevelPumping = 0;
        TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC3.TCA9539_Onput.all
                & ~(Valve_5));
        TCA9539_IC2.updateOutputFlag = 1;

    }

}
void HomeReturn_Process_Run(void *PrPtr)
{
    if (InHomeReturn && HomeReturnTriger)
    {
#if(Grindmotor == scooter)
        SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
#endif
        HomeReturnTriger = 0;

        // Set direction
        TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                & ~(Direction_BLDC2));  // Set LOW
        // Enable motor driver
        TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                & ~(Enable_BLDC2));         // enable - Not connect to gnd
        TCA9539_IC2.updateOutputFlag = 1;
        PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, speedstep);   // Freq = 8Khz
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
                         PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) / 3);

    }
}
void HomeReturn_Process_Stop(void *PrPtr)
{
    if (InHomeReturn)
    {
        //Disable driver motor
        TCA9539_IC2.TCA9539_Onput.all = ((TCA9539_IC2.TCA9539_Onput.all
                & ~(Enable_BLDC2) | Enable_BLDC2)); // Disable -  connect to gnd
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
void Read_INT_Handler(void)
{
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_1);
    TCA9539_IC1.ReadCmdFlag = 1;
    TCA9539_IC2.ReadCmdFlag = 1;
    TCA9539_IC3.ReadCmdFlag = 1;
}
void TimmingPorcess()
// interrupt timer 25ms
{
    TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
    if (InGrinding)
    {
        if (avgvel >= sp3)
        {
            VrTimer_Compensate += k3;
            pp3++;
            update = 1;
        }
        else if (avgvel >= sp2 && avgvel < sp3)
        {
            VrTimer_Compensate += k2;
            pp2++;
            update = 1;
        }
        else if (avgvel >= sp1 && avgvel < sp2) //avgvel >= sp1
        {
            VrTimer_Compensate += k1;
            update = 1;
            pp1++;
        }
        if (VrTimer_Grinding != 0)
        {

            VrTimer_Grinding--;
            ppo++;
            ppi = (float) ppo * UnitTimer;
        }
        if (ppi <= 15.0f)
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

    /*    if (InCompress)
     {
     if (VrTimer_Compress != 0)
     VrTimer_Compress--;

     }*/
    if (delay_flag)
    {
        if (VrTimer_Delay != 0)
            VrTimer_Delay--;
        else

            delay_flag = 0;

    }
    if (step_status[4] == Finish && (InRelay_Timming == 1))
    {
        if (VrTimer_Relay != 0)
            VrTimer_Relay--;
        else
        {
            switch (Step_Relay)
            {
            case 0:

                VrTimer_Relay = 100;
                TCA9539_IC3.TCA9539_Onput.all = ((TCA9539_IC3.TCA9539_Onput.all
                        & ~(Valve_3)) | Valve_3); //Open Valve 3
                Step_Relay++;
                break;
            case 1:
                VrTimer_Relay = 50;
                TCA9539_IC3.TCA9539_Onput.all = (TCA9539_IC3.TCA9539_Onput.all
                        & ~(Valve_3));
                Step_Relay++;
                break;
            case 2:
                InRelay_Timming = 0;
                break;
            }
            TCA9539_IC3.updateOutputFlag = 1;
        }

    }
    if (triggerCount_ExtractionTime)
        CoffeeExtractionTime++;

    if (flagdelay && (step_status[4] == Running))
    {
        if (VrTimer_FlagRelay != 0)
            VrTimer_FlagRelay--;
        else
        {
            switch (Step_Pump)
            {
            case 0:

                VrTimer_FlagRelay = infu / UnitTimer;
                /*                TCA9539_IC3.TCA9539_Onput.all = (TCA9539_IC3.TCA9539_Onput.all
                 & ~(Valve_1));*/
                Step_Pump++;
                break;

            case 1:
                SpeedDuty_Pump = HighPressure_Pump;
                VrTimer_FlagRelay = 50;
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, SpeedDuty_Pump);
                PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
                //
                TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                        & ~(Direction_BLDC3));
                // Enable motor driver
                TCA9539_IC2.TCA9539_Onput.all = ((TCA9539_IC2.TCA9539_Onput.all
                        & ~(Enable_BLDC3)) | Enable_BLDC3);
                TCA9539_IC3.TCA9539_Onput.all = ((TCA9539_IC3.TCA9539_Onput.all
                        & ~(Valve_1) | Valve_1));
                TCA9539_IC3.TCA9539_Onput.all = ((TCA9539_IC3.TCA9539_Onput.all
                        & ~(Valve_2) | Valve_2));
                TCA9539_IC2.updateOutputFlag = true;
                Step_Pump++;
                flagdelay = 0;
                break;
            }
            TCA9539_IC3.updateOutputFlag = true;
        }
    }
}
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

                speedTemp = PWMPulseWidthGet(PWM0_BASE, PWM_OUT_0);
                if ((speedTemp + GringPWMIncrement) <= MaxSpeedGring)
                {
                    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,
                                     speedTemp + GringPWMIncrement);
                }

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

void MakeCoffee()
{
    uint8_t i;
    for (i = 0; i < NumberOfStep; i++)
    {
        step_status[i] = Pending;
    }
    InHomeReturn = InGrinding = InCompress = InPumping = 0;
    Vr_C2Task = 0;
    InProcess = 1;
    FinishProcess = 0;
    if (test_step == 0)
        step = 0;
    Gui_CoffeExtractionTime = 0;
    InWarming = delay_flag = 0;
    TCA9539_IC3.TCA9539_Onput.all =
            (TCA9539_IC3.TCA9539_Onput.all & ~(Valve_4));
    TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT); // Interrupt timer
    TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
// B_Group_Task = &MakeCoffeProcess;
}

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
                        (step = 6, cancel_cmd = 0, cancel_storage = 1) :
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
                        calibVolumeFlag = 0;
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
                calibVolumeFlag = 0;
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
void StartUpMachine()
{
    uint8_t i;
    for (i = 0; i < NumberOfStep; i++)
    {
        step_status[i] = Pending;
    }
    InHomeReturn = InCompress = 0;
    InStartUp = 1;
    FinishProcess = 0;
    step = 0;
    TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT); // Interrupt timer
    TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
}
void StartUpProcess()
{
    if (InStartUp && SteamReady)
    {
        switch (step)
        {
        case 0:
        case 2:
            if (!InHomeReturn && (!delay_flag))
            {
                step_status[step] = Running;
                HomeReturnTriger = InHomeReturn = 1;
                if (HomePeding != 0)
                {
                    Cmd_WriteMsg(&HomeReturn_Process_Run, Null);

                }
                if (step == 2)
                    TCA9539_IC3.TCA9539_Onput.all =
                            (TCA9539_IC3.TCA9539_Onput.all & ~(Valve_4));
            }

            if ((InHomeReturn) && (HomePeding == 0))   // Read LimitSwitch))
            {
                InHomeReturn = 0;

                if (step == 2)
                {
                    InStartUp = step = 0;
                    TimerIntDisable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
                }
                step++;
                Delay_20ms(25);
            }
            break;
        case 1:
            if ((!InCompress) && (!delay_flag))
            {
                step_status[step] = Running;
                InCompress = CompressTriger = 1;
                Pos_Compress = 20;
                PosStep_Compress = stepPos1 + stepPos2;
                Cmd_WriteMsg(&Compress_Process_Run, (void*) &Pos_Compress);

            }
            if ((InCompress) && (step_status[step] == Finish))
            {
                InCompress = 0;
                step++;
                TCA9539_IC3.TCA9539_Onput.all = ((TCA9539_IC3.TCA9539_Onput.all
                        & ~(Valve_4)) | Valve_4);
                Delay_20ms(6000);

            }
            break;

        };
        B_Group_Task = &CheckingFinish_StepInRuning;
    }
}
void WarmingPressMachine(void *P)
{

    InWarming = 1;
    stepWarming = 0;
    TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT); // Interrupt timer
    TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
}
void WarmingPressProcess(void)
{
    static uint32_t stepWarming;
    if (InWarming && SteamReady)
    {
        switch (stepWarming)
        {
        case 0:
            TCA9539_IC3.TCA9539_Onput.all = ((TCA9539_IC3.TCA9539_Onput.all
                    & ~(Valve_4)) | Valve_4);
            Delay_20ms(750);
            stepWarming++;
            break;
        case 1:
            if (!delay_flag)
            {
                InWarming = stepWarming = 0;
                TCA9539_IC3.TCA9539_Onput.all = (TCA9539_IC3.TCA9539_Onput.all
                        & ~(Valve_4));
                TimerIntDisable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
            }

        }
    }
    B_Group_Task = &CheckingFinish_StepInRuning;
}
void FeedbackVelGrind(void)
{
    if (QEIIntStatus(QEI1_BASE, true) == QEI_INTTIMER)
    {
        QEIIntClear(QEI1_BASE, 0x0F);
        ds = QEIPositionGet(QEI1_BASE);
        FreQ = QEIVelocityGet(QEI1_BASE);
        vel = FreQ / 4.0 * 600;

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
void LookUpCompressTime(void)
{
    switch (PitchOfpress)
    {
    case 15:
        stepPos2 = 5300;
        break;
    case 16:
        stepPos2 = 5200;
        break;
    case 17:
        stepPos2 = 5100;
        break;
    case 18:
        stepPos2 = 5000;
        break;
    case 19:
        stepPos2 = 4900;
        break;
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

    QEIVelocityConfigure(QEI1_BASE, QEI_VELDIV_1, 8000000); // 100ms
    QEIFilterConfigure(QEI1_BASE, QEI_FILTCNT_16);
    QEIFilterEnable(QEI1_BASE);
    QEIVelocityEnable(QEI1_BASE);
    QEIIntRegister(QEI1_BASE, &FeedbackVelGrind);
    uint32_t status = QEIIntStatus(QEI1_BASE, true);
    QEIIntClear(QEI1_BASE, status);
    QEIEnable(QEI1_BASE);
}
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

void CleanProcess()
{

}
