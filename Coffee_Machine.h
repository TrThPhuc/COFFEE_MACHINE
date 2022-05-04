/*
 * Coffee_Machine.h
 *
 *  Created on: Jun 23, 2021
 *      Author: 16126
 */

#ifndef COFFEE_MACHINE_H_
#define COFFEE_MACHINE_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "inc/hw_memmap.h"
#include "inc/hw_qei.h"
#include "inc/hw_types.h"
#include "inc/hw_pwm.h"
#include "inc/hw_ints.h"

#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/ssi.h"
#include "driverlib/pin_map.h"
#include "driverlib/qei.h"
#include "driverlib/i2c.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/udma.h"
#include "driverlib/eeprom.h"
#include "driverlib/watchdog.h"
#include "driverlib/rom_map.h"
#include "driverlib/fpu.h"

#define BOARD 1
#define pwm 1
#define Npwm 0
#define Grind Npwm

#define scooter 1
#define bldc 0
#define Grindmotor bldc

#define SteamWarming 1
#define HeatingResWarming 0

#define WarmingMethod HeatingResWarming

#define PosPress 1
#define Int_SSI0

#define HighLvSensor 2.2
#define LowLvSensor 1.1

#define SensorOutlet        1
#define NonSensorOutlet     0
#define Outlet  NonSensorOutlet
struct AmountofWater
{
    uint32_t stage_1;   //  Low flow
    uint32_t stage_2;   //  High flow
};
typedef struct Mode_Parameter
{
    uint32_t PreInfusion;           // Pre-Infusion Time. Uint - second
    uint32_t Time;                  // Time for extraction  Uinit - second

    struct AmountofWater AmountOfWaterPumping;
    float GrindingDuration;         // Grinding interval. Uint - 0.1s
    uint32_t WeigtOfPowder;         //  Weight of coffe powder - g
    bool DirGrinding;               // CW or CCW Grinding
    uint32_t Pitch;
    uint16_t Cups;                  // Number of Cups
    bool smallSize;

} Mode_Parameter_t;
void ParameterDefaultSetting();
void AssignErrorList(void);
enum GuiParamter
{
    cupsEspresso_1,
    cupsEspresso_2,
    cupsSpecial_1,
    cupsSpecial_2,
    HotWaterTemp,
    ExtractionTime,
    Blade1NofTimesUsed,
    Blade2NofTimesUsed,
    RonNofTimesUsed,
    Brand,
    SeriNumber,
    Model,
    ProductDate,
    tempExtrude,
    PulseCount,
    ExtractAtimes,
    ExtractBtimes

};

enum errorlist
{
    eTCA_Ic1,
    eTCA_Ic2,
    eTCA_Ic3,
    eAds11118,
    eFaultMotor,
    eLevelSensor,
    eHotWaterOpen,
    eHotWaterTimeOut,
    eSteamOpen,
    eSteamTimeOut,
    ePumpPulseFast,
    ePumpPulseSlow,
    eNoPumpPulse,
    eCoffeOutletDetect,
    eHomeReturn
};
enum warningList
{
    wRonTimeLife, wBladeLife, wTempHotwater

};
typedef struct
{
    bool *ErrorFlag;
    uint8_t id;
    char *ErrorMsg;

} Error_t;
Error_t ErrorMachine[16];
uint8_t ErrorMachine_id[16];
extern uint16_t eVrTimer[8];
typedef enum eVrTimerList_e
{
    eVrCoffeOutlet,
    eVrPumpPulseFast,
    eVrPumpPulseSlow,
    eVrNoPumpPulse,
    eVrHomeReturn,
    eVrHotWaterHeatingTimeOut,
    eVrSteamHeatingTimeOut,
    eVrPumpingPulse
} eVrTimerList;

enum CountStorage
{
    CupsE1, CupsE2, CupsS1, CupsS2, BladeL, BladeR, RonTimes
};

#define DataCountSaveAddress   0xA0
extern bool TestModuleTrigger;
bool TestModuleStart;



#endif /* COFFEE_MACHINE_H_ */
