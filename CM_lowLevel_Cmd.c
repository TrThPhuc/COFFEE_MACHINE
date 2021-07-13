/*
 * CM_lowLevel_Cmd.c
 *
 *  Created on: Jun 23, 2021
 *      Author: 16126
 */
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
#define Null 0
#define MSG_QUEUE_SIZE 16
#define MSG_QUEUE_ARG_SIZE 16
volatile bool InProcess;
volatile bool GrindingTriger, InGrinding;
volatile bool CompressTriger, InCompress;
volatile bool PumpingTriger, InPumping;
volatile bool HomeReturnTriger, InHomeReturn;
volatile bool LevelControlTriger;
uint32_t VrTimer_Grinding;
uint32_t VrTimer_Compress;
uint16_t SpeedDuty_Pump;
const uint32_t Pos_Compress = 80;

extern volatile uint32_t SetVolume;
extern volatile bool FinishPumpEvent;
extern volatile uint32_t totalMilliLitres, MilliLitresBuffer;
extern void defaultISR(void);

volatile uint8_t stage, HomePeding;
extern Mode_Parameter_t *ModeSelected;

#define Osmosis_pump        4000
#define HighPressure_Pump   7999
#define NumberOfStep  6
void (*msg_queue[MSG_QUEUE_SIZE])(void*);
void *msg_queue_arg[MSG_QUEUE_SIZE];
volatile uint8_t msg_head = 0, msg_tail = 0;
volatile uint8_t step;
bool error_process = false;
uint8_t step_status[NumberOfStep];
#define Finish      1
#define Running     2
#define Pending      3

extern TCA9539Regs TCA9539_IC1, TCA9539_IC2, TCA9539_IC3;
extern void defaultFunction();
extern void (*B_Group_Task)(void);
extern void Default_B(void);
extern uint32_t clockrate;
extern bool readinput;
extern bool cancel_cmd;

extern void InitPumpingEvent();
extern void TCA9539ReadInputReg_BrustSlave(TCA9539Regs **RegsA);
extern TCA9539Regs *TCA9539_IC[3];
void CheckingFinish_StepInRuning();
void MakeCoffeProcess(void);
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
    skip: asm(" nop");
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
        // Enable PWM
        PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 7999); // Max speed
        VrTimer_Grinding = Mode->GrindingDuration;
        TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT); // Interrupt timer
        TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);

    }

}
void Grinding_Process_Stop(void *PrPtr)
{
    if (InGrinding)
    {
        // Disable BLDC1
        TCA9539_IC3.TCA9539_Onput.all = (TCA9539_IC3.TCA9539_Onput.all
                & ~(Enable_BLDC1));
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
        TCA9539_IC2.TCA9539_Onput.all = ((TCA9539_IC2.TCA9539_Onput.all
                & ~(Enable_BLDC2)) | Enable_BLDC2);
        PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true); // Enable output pwm
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,  // 50% Duty
                         PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) / 2);
        VrTimer_Compress = *(uint32_t*) PrPtr;
        TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT); // Interrupt timer
        TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
    }
}
void Compress_Process_Stop(void *PrPtr)
{
    if (InCompress)
    {
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);
        TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                & ~(Enable_BLDC2));
        PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, false);
    }
}
void Pumping_Process_Run(void *PrPtr)
{
    if (InPumping && PumpingTriger)
    {
        PumpingTriger = 0;
        InPumping = 1;
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
        // Configure valve to steam (turn off) or Hotwater(turn on)
        TCA9539_IC2.TCA9539_Onput.all = ((TCA9539_IC2.TCA9539_Onput.all
                & ~(SteamValve)));
        TCA9539_IC2.TCA9539_Onput.all = ((TCA9539_IC2.TCA9539_Onput.all
                & ~(HotWater)) | HotWater);
        SysCtlDelay(8000000);   // Delay 0.1s
        Mode_Parameter_t *Mode = (Mode_Parameter_t*) PrPtr;

        if (stage == 0)
        {
            SpeedDuty_Pump = Osmosis_pump;
            SetVolume = Mode->AmountOfWaterPumping.stage_1;

        }
        else
        {
            SpeedDuty_Pump = HighPressure_Pump;
            SetVolume = Mode->AmountOfWaterPumping.stage_2;
        }
        // Initialize a pump envent to calculate volume
        FinishPumpEvent = false;
        InitPumpingEvent();
        // Set direction
        TCA9539_IC2.TCA9539_Onput.all = ((TCA9539_IC2.TCA9539_Onput.all
                & ~(Direction_BLDC3)) | Direction_BLDC2);
        // Enable motor driver
        TCA9539_IC2.TCA9539_Onput.all = ((TCA9539_IC2.TCA9539_Onput.all
                & ~(Enable_BLDC3)) | Enable_BLDC3);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, SpeedDuty_Pump);
        PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);

        B_Group_Task = &CheckingFinish_StepInRuning;
    }
}
void Pumping_Process_Stop(void *PrPtr)
{
    if (InPumping)
    {
        TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                & ~(Enable_BLDC3));
        // Turn off Hot Water Valve
        TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                & ~(HotWater));
        totalMilliLitres = MilliLitresBuffer;
        MilliLitresBuffer = 0;
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
        PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, false);
        FinishPumpEvent = true;

    }

}
void SteamLevelControl_Run(void *PrPtr)
{
    if (LevelControlTriger)
    {
        LevelControlTriger = 0;
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
        TCA9539_IC2.TCA9539_Onput.all = ((TCA9539_IC2.TCA9539_Onput.all
                & ~(SteamValve)) | SteamValve);
        SysCtlDelay(8000000);
        SpeedDuty_Pump = HighPressure_Pump;
        // Set direction
        TCA9539_IC2.TCA9539_Onput.all = ((TCA9539_IC2.TCA9539_Onput.all
                & ~(Direction_BLDC3)) | Direction_BLDC2);
        // Enable motor driver
        TCA9539_IC2.TCA9539_Onput.all = ((TCA9539_IC2.TCA9539_Onput.all
                & ~(Enable_BLDC3)) | Enable_BLDC3);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, SpeedDuty_Pump);
        PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
    }
}
void SteamLevelControl_Stop(void *PrPtr)
{
    // Disable driver motor
    TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
            & ~(Enable_BLDC3));
    // Turn off Hot Water Valve
    TCA9539_IC2.TCA9539_Onput.all =
            (TCA9539_IC2.TCA9539_Onput.all & ~(HotWater));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, false);

}
void HomeReturn_Process_Run(void *PrPtr)
{
    if (InHomeReturn && HomeReturnTriger)
    {
        HomeReturnTriger = 0;

        // Set direction
        TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                & ~(Direction_BLDC2));
        // Enable motor driver
        TCA9539_IC2.TCA9539_Onput.all = ((TCA9539_IC2.TCA9539_Onput.all
                & ~(Enable_BLDC2)) | Enable_BLDC2);
        PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
                         PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) - 1);
    }
}
void HomeReturn_Process_Stop(void *PrPtr)
{
    if (InHomeReturn)
    {
        //Disable driver motor
        TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                & ~(Enable_BLDC2));
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);
        PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, false);

    }
}
void Delay_ms(uint16_t time_ms)
{

}
void Read_INT_Handler(void)
{
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_1);
    TCA9539_IC1.ReadCmdFlag = 1;
    TCA9539_IC2.ReadCmdFlag = 1;
    TCA9539_IC3.ReadCmdFlag = 1;

    /*   Cmd_WriteMsg((void(*)(void*))TCA9539ReadInputReg, (void*)&TCA9539_IC1);
     Cmd_WriteMsg((void(*)(void*))TCA9539ReadInputReg, (void*)&TCA9539_IC2);
     Cmd_WriteMsg((void(*)(void*))TCA9539ReadInputReg, (void*)&TCA9539_IC3);*/

}
void TimmingPorcess()

{
    TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
    if (InGrinding)
    {
        if (VrTimer_Grinding != 0)
            VrTimer_Grinding--;
        else
            TimerIntDisable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);

    }

    if (InCompress)
    {
        if (VrTimer_Compress != 0)
            VrTimer_Compress--;
        else
            TimerIntDisable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
    }

}
void CheckingFinish_StepInRuning()
{
    if (InProcess)
    {
        switch (step)
        {
        case 0:
        case 5: // Home return
            if (TCA9539_IC1.TCA9539_Input.all & LinitSwitch)
            {
                Cmd_WriteMsg(&HomeReturn_Process_Stop, Null);
                step_status[step] = Finish;

            }
            break;
        case 1: // Gringding coffee
            if ((VrTimer_Grinding == 0) && InGrinding)
            {
                Cmd_WriteMsg(&Grinding_Process_Stop, Null);
                step_status[step] = Finish;

            }
            if (cancel_cmd)
                VrTimer_Grinding = 0;
            break;
        case 2: // Compress coffee
            if ((VrTimer_Compress == 0) && InCompress)
            {
                Cmd_WriteMsg(&Compress_Process_Stop, Null);
                step_status[step] = Finish;

            }

            break;
        case 3:
        case 4:   // pump hot water
            if (FinishPumpEvent)
            {

                QEIIntDisable(QEI0_BASE, QEI_INTTIMER);
                step_status[step] = Finish;
                //   Cmd_WriteMsg(&Pumping_Process_Run, &Pumping_Process_Stop);

            }
            break;

        };
    }
    B_Group_Task = &Default_B;

}
void CheckProperlyStep()
{
    uint8_t i;
    if (InProcess)
    {
        for (i = 0; i < NumberOfStep; i++)
        {
            if (i < step)
            {
                if (step_status[i] != Finish)
                    error_process = true;
            }
            else
            {
                if ((step_status[i] != Running) && (step_status[i] != Pending))
                    error_process = true;
            }
        }
        if (error_process)
        {
            error_process = 0;
            InProcess = 0;
            step = 0;
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 0);
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
            PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, false);
            PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, false);
            PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, false);

        }
    }
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

// B_Group_Task = &MakeCoffeProcess;
}

void MakeCoffeProcess(void)
{

    if (!error_process && InProcess)
    {
        switch (step)
        {
        case 0:
        case 5: // Home return compress block
            if (!HomePeding && !InHomeReturn)
            {
                step_status[step] = Running;
                HomeReturnTriger = InHomeReturn = 1;
                Cmd_WriteMsg(&HomeReturn_Process_Run, Null);
            }
            if ((InHomeReturn) && (step_status[step] == Finish))
            {
                InHomeReturn = 0;

                ((step == 5) || (cancel_cmd)) ?
                        (InProcess = step = 0, cancel_cmd = 0) : (step++);
            }
            break;
        case 1: // Grinding coffee
            if (!InGrinding)
            {
                step_status[step] = Running;
                GrindingTriger = InGrinding = 1;
                Cmd_WriteMsg(&Grinding_Process_Run, (void*) ModeSelected);

            }
            if ((InGrinding) && (step_status[step] == Finish))
            {

                InGrinding = 0;
                step++;
            }
            break;
        case 2:
            if (!InCompress)
            {
                step_status[step] = Running;
                InCompress = CompressTriger = 1;
                Cmd_WriteMsg(&Compress_Process_Run, (void*) &Pos_Compress);

            }
            if ((InCompress) && (step_status[step] == Finish))
            {

                InCompress = 0;
                if (cancel_cmd)
                {
                    step = 0;
                    goto skip_compress;
                }
                step++;
                skip_compress: ;
            }
            break;
        case 3:
        case 4:
            if (!InPumping)
            {
                step_status[step] = Running;
                InPumping = PumpingTriger = 1;
                (step == 3) ? (stage = 0) : (stage = 1);
                Cmd_WriteMsg(&Pumping_Process_Run, (void*) ModeSelected);
            }
            if (InPumping)
            {
                if (step_status[step] == Finish)
                {
                    InPumping = 0;
                    step++;

                }
                if (cancel_cmd)
                {
                    step = 0;
                    Cmd_WriteMsg(&Pumping_Process_Stop, Null);
                }
            }
            break;
        default:
            InProcess = step = 0;

        };
        //CheckProperlyStep();
        B_Group_Task = &CheckingFinish_StepInRuning;
    }
}
