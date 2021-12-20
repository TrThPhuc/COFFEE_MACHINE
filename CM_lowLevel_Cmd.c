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
volatile bool LevelControlTriger, InLevelPumping;
volatile bool InRelay_Timming;
uint32_t VrTimer_Grinding, VrTimer_Compress, VrTimer_Delay, VrTimer_Pumping,
        VrTimer_Relay;
uint16_t SpeedDuty_Pump;
uint32_t Pos_Compress, pos1 = 240, pos2 = 180;  //180,120
uint8_t t1 = 100, t2 = 40;     // for test pre-infusion

uint32_t GrringPWMIncrement = 100;

uint16_t delay;
bool delay_flag = 0;
volatile uint8_t Step_Relay;

extern uint32_t SetVolume;
extern volatile bool FinishPumpEvent;
extern volatile uint32_t totalMilliLitres, MilliLitresBuffer;
extern void defaultISR(void);

volatile uint8_t stage;
extern Mode_Parameter_t *ModeSelected;

#define HomePeding  (TCA9539_IC3.TCA9539_Input.all & LinitSwitch)
#define PreInfusion_pump        3000
#define HighPressure_Pump   7999
#define NumberOfStep  7
#define MaxSpeedGring 9999

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

extern void (*B_Group_Task)(void);
extern void B2(void);
extern uint32_t clockrate;
extern bool readinput;
extern bool cancel_cmd, test_step; // Cancel button

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
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 3000); // Max speed
        VrTimer_Grinding = Mode->GrindingDuration;
        /*        TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT); // Interrupt timer
         TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);*/

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
        PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true); // Enable output pwm
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 10000);    // Freq = 10Khz
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 2000);
        VrTimer_Compress = *(uint32_t*) PrPtr;
        TCA9539_IC2.updateOutputFlag = 1;
        /*        TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT); // Interrupt timer
         TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);*/
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
        TCA9539_IC3.TCA9539_Onput.all = ((TCA9539_IC3.TCA9539_Onput.all
                & ~(Valve_2)) | Valve_2);
        TCA9539_IC3.updateOutputFlag = 1;

        Mode_Parameter_t *Mode = (Mode_Parameter_t*) PrPtr;

        SpeedDuty_Pump = PreInfusion_pump;
        SetVolume = Mode->AmountOfWaterPumping.stage_1;

        // Initialize a pump envent to calculate volume
        FinishPumpEvent = false;
        InitPumpingEvent();
        // Set direction
        TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                & ~(Direction_BLDC3));
        // Enable motor driver
        TCA9539_IC2.TCA9539_Onput.all = ((TCA9539_IC2.TCA9539_Onput.all
                & ~(Enable_BLDC3)) | Enable_BLDC3);

        VrTimer_Pumping = t1;
        /*        TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT); // Interrupt timer
         TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);*/
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
        // Turn off valve 1
        TCA9539_IC3.TCA9539_Onput.all = (TCA9539_IC3.TCA9539_Onput.all
                & ~(Valve_1));
        // Turn on valve 3 after press-part return home
        /*        TCA9539_IC3.TCA9539_Onput.all = ((TCA9539_IC3.TCA9539_Onput.all
         & ~(Valve_3)) | Valve_3);*/

        totalMilliLitres = MilliLitresBuffer;
        MilliLitresBuffer = 0;
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
        PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, false);
        FinishPumpEvent = true;

    }

}
void SteamLevelControl_Run(void *PrPtr)
{
    if (InLevelPumping && LevelControlTriger)
    {
/*        LevelControlTriger = 0;
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
                TCA9539_IC3.TCA9539_Onput.all = ((TCA9539_IC3.TCA9539_Onput.all
         & ~(SteamValve)) | SteamValve);
        SpeedDuty_Pump = HighPressure_Pump;
        // Set direction
        TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                & ~(Direction_BLDC3));
        // Enable motor driver
        TCA9539_IC2.TCA9539_Onput.all = ((TCA9539_IC2.TCA9539_Onput.all
                & ~(Enable_BLDC3)) | Enable_BLDC3);
        TCA9539_IC2.updateOutputFlag = 1;
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, SpeedDuty_Pump);
        PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);*/
    }
}
void SteamLevelControl_Stop(void *PrPtr)
{
    if (InLevelPumping)
    {
/*        InLevelPumping = 0;
        // Disable driver motor
        TCA9539_IC2.TCA9539_Onput.all = (TCA9539_IC2.TCA9539_Onput.all
                & ~(Enable_BLDC3));
        TCA9539_IC2.updateOutputFlag = 1;
        // Turn off Hot Water Valve
            TCA9539_IC2.TCA9539_Onput.all =
         (TCA9539_IC2.TCA9539_Onput.all & ~(HotWater));
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
        PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, false);*/
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
    /*    TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT); // Interrupt timer
     TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);*/

}
void Relay_Timming(void)
{
    VrTimer_Relay = 80;
    Step_Relay = 0;
    InRelay_Timming = 1;
    /*    TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT); // Interrupt timer
     TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);*/
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
        if (VrTimer_Grinding != 0)
            VrTimer_Grinding--;

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

                VrTimer_Relay = 80;
                TCA9539_IC3.TCA9539_Onput.all = ((TCA9539_IC3.TCA9539_Onput.all
                        & ~(Valve_3)) | Valve_3);

                Step_Relay++;
                break;
            case 1:
                VrTimer_Relay = 160;

                TCA9539_IC3.TCA9539_Onput.all = (TCA9539_IC3.TCA9539_Onput.all
                        & ~(Valve_2));
                Step_Relay++;
                break;

            case 2:
                TCA9539_IC3.TCA9539_Onput.all = (TCA9539_IC3.TCA9539_Onput.all
                        & ~(Valve_3));
                InRelay_Timming = 0;

                break;
            }
            TCA9539_IC3.updateOutputFlag = 1;
        }

    }
    if (InPumping)
    {
        if (VrTimer_Pumping != 0)
            VrTimer_Pumping--;
        else
        {
            if (stage == 0)  // Pre_infusion
            {
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
                stage = 1;
                VrTimer_Pumping = t2;   // Duration : 1S
            }
            else
            {
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, HighPressure_Pump);

            }

        }
    }

}
void CheckingFinish_StepInRuning()
{
    uint32_t speedTemp;
    if (InProcess)
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
                if ((speedTemp + GrringPWMIncrement) <= MaxSpeedGring)
                {
                    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,
                                     speedTemp + GrringPWMIncrement);
                }

                if (VrTimer_Grinding == 0)
                {
                    Cmd_WriteMsg(&Grinding_Process_Stop, Null);
                    step_status[step] = Finish;

                }

            }
            if (cancel_cmd)
                VrTimer_Grinding = 0;
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
            if (FinishPumpEvent)
            {
                QEIIntDisable(QEI0_BASE, QEI_INTTIMER);
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

    TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT); // Interrupt timer
    TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
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
            if ((!InHomeReturn) && (!delay_flag))
            {
                step_status[step] = Running;
                HomeReturnTriger = InHomeReturn = 1;
                if (HomePeding != 0)
                {
                    Cmd_WriteMsg(&HomeReturn_Process_Run, Null);
                    if (step == 5)
                        Relay_Timming();
                }

            }

            if ((InHomeReturn) && (step_status[step] == Finish))
            {
                InHomeReturn = 0;
                (cancel_cmd) ?
                        (InProcess = step = 0, cancel_cmd = 0) : (step++);

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

            }
            break;
        case 1:
        case 3:
        case 6:
            if ((!InCompress) && (!delay_flag))
            {
                step_status[step] = Running;
                InCompress = CompressTriger = 1;
                (step == 1) ? (Pos_Compress = pos1) : (Pos_Compress = pos2);
                if (step == 6)
                    (Pos_Compress = pos1 / 4);
                Cmd_WriteMsg(&Compress_Process_Run, (void*) &Pos_Compress);

            }
            if ((InCompress) && (step_status[step] == Finish))
            {

                InCompress = 0;
                if (cancel_cmd)
                {
                    step = 5;
                    cancel_cmd = 0;

                    goto skip_compress;

                }
                (step == 6) ? (InProcess = step = 0, cancel_cmd = 0) : (step++);

                skip_compress: ;
            }
            break;
        case 4:
            //       case 5:
            if ((!InPumping) && (!delay_flag))
            {
                step_status[step] = Running;
                InPumping = PumpingTriger = 1;
                stage = 0;
                // (step == 4) ? (stage = 0) : (stage = 1);
                Cmd_WriteMsg(&Pumping_Process_Run, (void*) ModeSelected);
            }
            if (InPumping)
            {
                if (step_status[step] == Finish)
                {
                    InPumping = 0;
                    step++;
                    /*                    if (test_step)
                     InProcess = step = test_step = 0;*/
                    if (cancel_cmd)
                    {
                        step = 5;
                        cancel_cmd = 0;
                    }

                    Delay_20ms(80);
                    // VrTimer_Valve3 = 240;
                }
                if (cancel_cmd)
                {

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
