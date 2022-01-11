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
volatile bool InProcess, FinishProcess, InStartUp;
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
void MakeCoffee();
void CheckingFinish_StepInRuning();
void MakeCoffeProcess(void);
void StartUpMachine(void);
void StartUpProcess(void);
//---------------------------------------
// Process - Grinding coffee into powder
volatile bool GrindingTriger, InGrinding;
//---------------------------------------
// Process - Cluster compress powder coffee
volatile bool CompressTriger, InCompress;
uint32_t Pos_Compress;
//---------------------------------------
// Process - Pumping Hot water
volatile bool PumpingTriger, InPumping;
extern volatile bool FinishPumpEvent;
uint16_t SpeedDuty_Pump;
bool calibVolumeFlag, calibVolumeStr;
extern void InitPumpingEvent();
extern float SetVolume;
// Millitre pumping water
extern volatile float totalMilliLitres, MilliLitresBuffer;
// Relay Timming for turn on/off Valve in Pumping process
volatile bool InRelay_Timming;
volatile uint8_t Step_Relay;
//---------------------------------------
// Process- Return home
volatile bool HomeReturnTriger, InHomeReturn;
//---------------------------------------
// Control Level water in steam tank
volatile bool LevelControlTriger, InLevelPumping;
//---------------------------------------
// Delay between process
volatile bool delay_flag = 0;
// ------------------------Share variable and function ------------------------------
extern Mode_Parameter_t *ModeSelected;
extern TCA9539Regs TCA9539_IC1, TCA9539_IC2, TCA9539_IC3;
#define HomePeding  (TCA9539_IC3.TCA9539_Input.all & LinitSwitch)   // Read LimitSwitch

extern void (*B_Group_Task)(void);   // B task 5ms
extern void B2(void);
volatile bool flagdelay = 0;
uint16_t VrTimer_FlagRelay = 160, ph = 100;
uint16_t Detect = 0;
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

void Grinding_Process_Run(void *PrPtr)
{
    if (InGrinding && GrindingTriger)

    {
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
        PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 5000);
        // Calculate grinding time
        float offsetTime = (Mode->GrindingDuration * 0.1) / UnitTimer;
        VrTimer_Grinding = (uint32_t) (offsetTime
                + (Mode->WeigtOfPowder * K_VrTimer_Grinding));

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
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 0);
        PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, false);

    }

}
void Compress_Process_Run(void *PrPtr)
{
    if (InCompress && CompressTriger)
    {
        CompressTriger = 0;
        // Set direction
        TCA9539_IC2.TCA9539_Onput.all = ((TCA9539_IC2.TCA9539_Onput.all
                & ~(Direction_BLDC2)) | Direction_BLDC2);
        // Enable motor driver
        TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                & ~(Enable_BLDC2));     // Enable - not connect to GND
        if (step == 3)
            TCA9539_IC3.TCA9539_Onput.all = ((TCA9539_IC3.TCA9539_Onput.all
                    & ~(Valve_2)) | Valve_2);
        PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true); // Enable output pwm
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 10000);   // Freq = 10Khz
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 2000);
        VrTimer_Compress = *(uint32_t*) PrPtr;
        TCA9539_IC2.updateOutputFlag = 1;

    }
}
void Compress_Process_Stop(void *PrPtr)
{
    if (InCompress)
    {
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);
        TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                & ~(Enable_BLDC2) | Enable_BLDC2);     // disable connect to gnd
        TCA9539_IC2.updateOutputFlag = 1;
        PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, false);
    }
}
void Pumping_Process_Run(void *PrPtr)
{
    if (InPumping && PumpingTriger)
    {
        PumpingTriger = 0;
        // InPumping = 1;
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
        // Configure  turn on valve 1, valve 2
        TCA9539_IC3.TCA9539_Onput.all = ((TCA9539_IC3.TCA9539_Onput.all
                & ~(Valve_1) | Valve_1));

        TCA9539_IC3.updateOutputFlag = 1;

        Mode_Parameter_t *Mode = (Mode_Parameter_t*) PrPtr;

        SpeedDuty_Pump = PreInfusion_pump;

        //SpeedDuty_Pump = HighPressure_Pump;
        SetVolume =
                (!calibVolumeFlag) ?
                        Mode->AmountOfWaterPumping.stage_1 : MaxVolume

        // Initialize a pump envent to calculate volume
        FinishPumpEvent = false;
        flagdelay = 1;
        VrTimer_FlagRelay = ph;

        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, SpeedDuty_Pump);
        PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
        //
        TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                & ~(Direction_BLDC3));
        // Enable motor driver
        TCA9539_IC2.TCA9539_Onput.all = ((TCA9539_IC2.TCA9539_Onput.all
                & ~(Enable_BLDC3)) | Enable_BLDC3);
        TCA9539_IC2.updateOutputFlag = true;
        Detect = 0;
        B_Group_Task = &CheckingFinish_StepInRuning;
        //
    }
}
void Pumping_Process_Stop(void *PrPtr)
{
    if (InPumping)
    {
        Mode_Parameter_t *Mode = (Mode_Parameter_t*) PrPtr;
        TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                & ~(Enable_BLDC3));
        // Turn off valve 1
        TCA9539_IC3.TCA9539_Onput.all = (TCA9539_IC3.TCA9539_Onput.all
                & ~(Valve_1));
        // Turn on valve 3 after press-part return home
        /*        TCA9539_IC3.TCA9539_Onput.all = ((TCA9539_IC3.TCA9539_Onput.all
         & ~(Valve_3)) | Valve_3);*/

        TCA9539_IC3.TCA9539_Onput.all = (TCA9539_IC3.TCA9539_Onput.all
                & ~(Valve_2));
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
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);

        SpeedDuty_Pump = HighPressure_Pump;
        // Set direction
        TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                & ~(Direction_BLDC3));
        // Enable motor driver
        TCA9539_IC2.TCA9539_Onput.all = ((TCA9539_IC2.TCA9539_Onput.all
                & ~(Enable_BLDC3)) | Enable_BLDC3);
        TCA9539_IC2.updateOutputFlag = 1;
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, SpeedDuty_Pump);
        PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
    }
}
void SteamLevelControl_Stop(void *PrPtr)
{
    if (InLevelPumping)
    {
        InLevelPumping = 0;
        // Disable driver motor
        TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                & ~(Enable_BLDC3));
        TCA9539_IC2.updateOutputFlag = 1;
        // Turn off Hot Water Valve

        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
        PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, false);
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
                & ~(Enable_BLDC2));         // enable - Not connect to gnd
        TCA9539_IC2.updateOutputFlag = 1;
        PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);

        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 6000);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
                         PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) / 2);
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
void Delay_20ms(uint16_t time_25ms)
{
    VrTimer_Delay = time_25ms;
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
        /*        if (VrTimer_Grinding != 0)
         VrTimer_Grinding--;*/

    }

    if (InCompress)
    {
        if (VrTimer_Compress != 0)
            VrTimer_Compress--;

    }
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

                VrTimer_Relay = 200;
                TCA9539_IC3.TCA9539_Onput.all = ((TCA9539_IC3.TCA9539_Onput.all
                        & ~(Valve_3)) | Valve_3); //Open Valve 3
                Step_Relay++;
                break;
            case 1:
                //   VrTimer_Relay = 80;
                TCA9539_IC3.TCA9539_Onput.all = (TCA9539_IC3.TCA9539_Onput.all
                        & ~(Valve_3));
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
            flagdelay = 0;
            SpeedDuty_Pump = HighPressure_Pump;
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, SpeedDuty_Pump);
            PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
            //
            TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                    & ~(Direction_BLDC3));
            // Enable motor driver
            TCA9539_IC2.TCA9539_Onput.all = ((TCA9539_IC2.TCA9539_Onput.all
                    & ~(Enable_BLDC3)) | Enable_BLDC3);
            TCA9539_IC2.updateOutputFlag = true;

        }
    }
}
void CheckingFinish_StepInRuning()
{
    uint32_t speedTemp;
    if (InProcess || InStartUp)
    {
        switch (step)
        {
        case 0:
        case 5: // Home return
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
                Cmd_WriteMsg(&Compress_Process_Stop, Null);
                step_status[step] = Finish;

            }

            break;
        case 4:
//        case 5:   // pump hot water
            if ((TCA9539_IC3.TCA9539_Input.all & LevelSensor2) == 0
                    && (Detect == 0))
            {
                InitPumpingEvent();
                Detect = 1;
            }

            if (FinishPumpEvent)
            {
                // QEIIntDisable(QEI0_BASE, QEI_INTTIMER);
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
    InProcess = 1;
    FinishProcess = 0;
    if (test_step == 0)
        step = 0;
    Gui_CoffeExtractionTime = 0;
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
            if ((!InHomeReturn) && (!delay_flag))
            {
                step_status[step] = Running;
                HomeReturnTriger = InHomeReturn = 1;
                if (HomePeding != 0)
                {
                    Cmd_WriteMsg(&HomeReturn_Process_Run, Null);
                    if (step == 0)
                        TCA9539_IC3.TCA9539_Onput.all =
                                ((TCA9539_IC3.TCA9539_Onput.all & ~(Valve_1)
                                        | Valve_1));
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
                Delay_20ms(80);
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
                        (Pos_Compress = pos1, TCA9539_IC3.TCA9539_Onput.all =
                                TCA9539_IC3.TCA9539_Onput.all & ~(Valve_1)) :
                        (Pos_Compress = pos2);
                if (step == 6)
                    (Pos_Compress = 20);
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
                    TCA9539_IC3.TCA9539_Onput.all =
                            (TCA9539_IC3.TCA9539_Onput.all & ~(Valve_2));
                    goto skip_compress;

                }
                //  (step == 6)  (InProcess = step = 0, cancel_cmd = 0) : (step++);
                (step == 3) ? (Delay_20ms(100)) : (Delay_20ms(40));

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
                CoffeeExtractionTime = 0;
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
                        //       Relay_Timming(0);
                    }
                    if (cancel_cmd)
                    {
                        step = 5;
                        cancel_cmd = 0;
                        cancel_storage = 1;
                    }
                    Relay_Timming(280);
                    Delay_20ms(200);
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

                    InProcess = step = 0, cancel_cmd = 0;
                    calibVolumeFlag = 0;
                    TimerIntDisable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
                    QEIIntDisable(QEI0_BASE, QEI_INTTIMER);

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
        B_Group_Task = &CheckingFinish_StepInRuning;
    }
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
    if (InStartUp)
    {
        switch (step)
        {
        case 0:
            if (!InHomeReturn)
            {
                step_status[step] = Running;
                HomeReturnTriger = InHomeReturn = 1;
                if (HomePeding != 0)
                {
                    Cmd_WriteMsg(&HomeReturn_Process_Run, Null);
                    TCA9539_IC3.TCA9539_Onput.all =
                            ((TCA9539_IC3.TCA9539_Onput.all & ~(Valve_1)
                                    | Valve_1));
                }

            }

            if ((InHomeReturn) && (step_status[step] == Finish))
            {
                InHomeReturn = 0;
                step++;
                Delay_20ms(100);
            }
            break;
        case 1:
            if ((!InCompress) && (!delay_flag))
            {
                step_status[step] = Running;
                InCompress = CompressTriger = 1;

                Pos_Compress = 20;
                Cmd_WriteMsg(&Compress_Process_Run, (void*) &Pos_Compress);

            }
            if ((InCompress) && (step_status[step] == Finish))
            {
                InCompress = 0;
                InStartUp = step = 0;
                TCA9539_IC3.TCA9539_Onput.all = TCA9539_IC3.TCA9539_Onput.all
                        & ~(Valve_1);
                TimerIntDisable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);

            }
            break;
        };
        B_Group_Task = &CheckingFinish_StepInRuning;
    }
}
void CleanProcess()
{

}
