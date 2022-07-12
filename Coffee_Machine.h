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

#define HighLvSensor    2.6
#define LowLvSensor     1

#define SensorOutlet        1
#define NonSensorOutlet     0
#define Outlet  NonSensorOutlet

#define NumOfParInEachMode  8
#define NumberOfParameter 48
#define NumOfObjWindow 4
#define NumOfParTemperature  5
struct AmountofWater
{
    uint32_t stage_1;   //  Low flow
    uint32_t stage_2;   //  High flow
};
typedef enum MotorInfusion_e
{
    noMotor = 0, haveMotor = 1
} MotorInfusion;
typedef struct Mode_Parameter
{
    uint32_t PreInfusion;           // Pre-Infusion Time. Uint - second
    uint32_t PosInfusion;
    float SpeedMotorInfusion;
    float SpeedMotorExtract;
    uint32_t Time;                  // Time for extraction  Uinit - second

    struct AmountofWater AmountOfWaterPumping;
    float GrindingDuration;         // Grinding interval. Uint - 0.1s
    uint32_t WeigtOfPowder;         //  Weight of coffe powder - g
    bool DirGrinding;               // CW or CCW Grinding
    uint32_t Pitch;
    uint16_t Cups;                  // Number of Cups
    bool smallSize;
    bool MotorPreInfusion;
    uint8_t IndexMode;

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
    ExtractBtimes,
    GroupTemp,
    BoilerTemp,
    PreinfusionTime,
    GrindTime
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
enum warninglist
{
    wSteamOpen, wSteamTimeOut
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
Error_t ErrorMachine[16], WarningMachine[5];
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
    eVrPumpingSteamTimeOut,
} eVrTimerList;

enum CountStorage
{
    CupsE1, CupsE2, CupsS1, CupsS2, BladeL, BladeR, RonTimes
};
enum VirtualTimer
{
    refreshPage,
    holdButtonHome,
    displayBootPage,
    displayTemperature,
    holdButtonUp,
    displayError,
    holdButtonDown,
    displayWarning

};
typedef enum ManualTest_e
{
    Ma_GrindMotor,
    Ma_PumpMotor,
    Ma_CompressMotor,
    Ma_BrewValve,
    Ma_CoffeeOutletValve,
    Ma_BackRinseValve,
    Ma_SteamInValve,
    Ma_DrainBoilerValve
} ManualTest;

#define DataCountSaveAddress   0xC4
extern bool TestModuleTrigger;

bool TestModuleStart;
void ParameterDefaultSetting();
void AssignParameterForMode(Mode_Parameter_t *thisMode, uint32_t **vPar);
void AssignGobalParSetting(uint32_t **vPar);
void AssignErrorList(void);
void AssginWarningList(void);
#endif /* COFFEE_MACHINE_H_ */
