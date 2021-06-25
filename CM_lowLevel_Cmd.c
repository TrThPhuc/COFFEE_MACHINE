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

#include "Coffee_Machine.h"
#include "TCA9539_hw_memmap.h"
#include "TCA9539.h"
#define Null 0
#define MSG_QUEUE_SIZE 5
#define MSG_QUEUE_ARG_SIZE 5
bool InProcess;
bool GrindingTriger, InGrinding;
bool CompressTriger, InCompress;
bool PumpingTriger, InPumping;
bool HomeReturnTriger, InHomeReturn;
uint32_t VrTimer_Grinding;
uint32_t VrTimer_Compressding;
uint16_t SpeedDuty_Pump, Volume;
extern volatile bool FinishPumpEvent;
uint8_t stage, HomePeding;
extern Mode_Parameter_t ModeSelected;

#define Osmosis_pump        2000
#define HighPressure_Pump   7999
#define NumberOfStep  5
void (*msg_queue[MSG_QUEUE_SIZE])(void*);
void *msg_queue_arg[MSG_QUEUE_ARG_SIZE];
volatile uint8_t msg_head = 0, msg_tail = 0;
static uint8_t step;
bool error_process = false;
uint8_t step_status[NumberOfStep];
#define Finish      1
#define Running     2
#define Peding      3

extern TCA9539Regs TCA9539_IC1, TCA9539_IC2, TCA9539_IC3;
extern void defaultFunction();
extern void (*B_Group_Task)(void);
extern void Default_B(void);
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
        InGrinding = 1;
        GrindingTriger = 0;
        // Set direction motor
        if (Mode->DirGrinding)
            TCA9539_IC3.TCA9539_Onput.all = ((TCA9539_IC3.TCA9539_Onput.all
                    & ~(Direction_BLDC1)) | Direction_BLDC1);
        else
            TCA9539_IC3.TCA9539_Onput.all = (TCA9539_IC3.TCA9539_Onput.all
                    & ~(Direction_BLDC1));
        // Enable BLDC1
        TCA9539_IC3.TCA9539_Onput.all = ((TCA9539_IC3.TCA9539_Onput.all
                & ~(Enable_BLDC1)) | Enable_BLDC1);
        PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,
                         PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) - 1);
        VrTimer_Grinding = Mode->GrindingDuration;
        B_Group_Task = &CheckingFinish_StepInRuning;

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
        InGrinding = 1;
        GrindingTriger = 0;
        TCA9539_IC2.TCA9539_Onput.all = ((TCA9539_IC2.TCA9539_Onput.all
                & ~(Direction_BLDC2)) | Direction_BLDC2);
        TCA9539_IC2.TCA9539_Onput.all = ((TCA9539_IC2.TCA9539_Onput.all
                & ~(Enable_BLDC2)) | Enable_BLDC2);
        PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
                         PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) - 1);
        VrTimer_Grinding = *(uint32_t*) PrPtr;
        B_Group_Task = &CheckingFinish_StepInRuning;
    }
}
void Compress_Process_Stop(void *PrPtr)
{
    if (InCompress)
    {
        TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                & ~(Enable_BLDC2));
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);
        PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, false);

    }
}
void Pumping_Process_Run(void *PrPtr)
{
    if (InPumping && PumpingTriger)
    {

        Mode_Parameter_t *Mode = (Mode_Parameter_t*) PrPtr;
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
        if (stage == 0)
        {
            SpeedDuty_Pump = Osmosis_pump
            ;
            Volume = Mode->AmountOfWaterPumping.stage_1;

        }
        else
        {
            SpeedDuty_Pump = HighPressure_Pump
            ;
            Volume = Mode->AmountOfWaterPumping.stage_2;
        }
        TCA9539_IC2.TCA9539_Onput.all = ((TCA9539_IC2.TCA9539_Onput.all
                & ~(Direction_BLDC3)) | Direction_BLDC2);
        TCA9539_IC2.TCA9539_Onput.all = ((TCA9539_IC2.TCA9539_Onput.all
                & ~(Enable_BLDC3)) | Enable_BLDC3);
        TCA9539_IC2.TCA9539_Onput.all = ((TCA9539_IC2.TCA9539_Onput.all
                & ~(SteamValve)) | SteamValve);
        TCA9539_IC2.TCA9539_Onput.all = ((TCA9539_IC2.TCA9539_Onput.all
                & ~(HotWater)) | HotWater);
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
        TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                & ~(HotWater));
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
        PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, false);
    }

}
void HomeReturn_Process_Run(void *PrPtr)
{
    if (InHomeReturn && HomeReturnTriger)
    {
        InHomeReturn = 1;
        PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
                         PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) - 1);
        B_Group_Task = &CheckingFinish_StepInRuning;

    }
}
void HomeReturn_Process_Stop(void *PrPtr)
{
    if (InHomeReturn)
    {
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);
        PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, false);

    }
}
void Delay_ms(uint16_t time_ms)
{

}
void Read_INT_Handler(void)
{
    TCA9539ReadInputReg(&TCA9539_IC1);
    TCA9539ReadInputReg(&TCA9539_IC2);
    TCA9539ReadInputReg(&TCA9539_IC3);
}
void CheckingFinish_StepInRuning()
{
    switch (step)
    {
    case 0: // Home return
        if (TCA9539_IC1.TCA9539_Input.all & LinitSwitch)
        {
            Cmd_WriteMsg(&HomeReturn_Process_Stop, Null);
            step_status[step] = Finish;

        }
        break;
    case 1: // gringding coffee
        if (VrTimer_Grinding == 0)
        {
            Cmd_WriteMsg(&Grinding_Process_Stop, Null);
            step_status[step] = Finish;

        }
        else
            VrTimer_Grinding--;
        break;
    case 2: // Compress coffee
        if (VrTimer_Compressding == 0)
        {
            Cmd_WriteMsg(&Compress_Process_Stop, Null);
            step_status[step] = Finish;

        }
        break;
    case 3: // pump hot water
        if (FinishPumpEvent)
        {
            Cmd_WriteMsg(&Pumping_Process_Stop, Null);
            step_status[step] = Finish;

        }
        break;

    };
    B_Group_Task = &Default_B;

}
void CheckProperlyStep(uint8_t step)
{
    uint8_t i;
    for (i = 0; i < NumberOfStep; i++)
    {
        if (i < step)
        {
            if (step_status[i] != Finish)
                error_process = true;
        }
        else
        {
            if ((step_status[i] != Running) || (step_status[i] != Running))
                error_process = true;
        }
    }
}
void MakeCoffee()
{
    InProcess = 1;
   // B_Group_Task = &MakeCoffeProcess;
}

void MakeCoffeProcess(void)
{
//  InProcess = true;
    CheckProperlyStep(step);
    if (!error_process && InProcess)
    {
        switch (step)
        {
        case 0:
            if (!HomePeding && InHomeReturn)
            {
                HomeReturnTriger = 1;
                Cmd_WriteMsg(&HomeReturn_Process_Run, Null);
            }
            if (step_status[step] == Finish)
            {
                InHomeReturn = 0;
                step++;
            }
            break;
        case 1:
            if (InGrinding)
            {
                GrindingTriger = 1;
                Cmd_WriteMsg(&Grinding_Process_Run, &ModeSelected);
            }
            if (step_status[step] == Finish)
            {
                InGrinding = 0;
                step++;
            }
            break;
        case 2:
            if (InCompress)
            {
                CompressTriger = 1;
                Cmd_WriteMsg(&Compress_Process_Run, &ModeSelected);
            }
            if (step_status[step] == Finish)
            {
                InCompress = 0;
                step++;
            }
            break;
        case 3:
            break;
        };
        B_Group_Task = &CheckingFinish_StepInRuning;
    }
}
