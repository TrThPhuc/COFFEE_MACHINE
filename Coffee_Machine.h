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

// System define
//#define SysCltEeprom
//#define SaveEeprom

#define BOARD 1
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
    float GrindingDuration;      // Grinding interval. Uint - 0.1s
    uint32_t WeigtOfPowder;         //  Weight of coffe powder - g
    bool DirGrinding;               // CW or CCW Grinding

    uint16_t Cups;                  // Number of Cups

} Mode_Parameter_t;
void ParameterDefaultSetting();

enum GuiParamter
{
    cupsEspresso_1,
    cupsEspresso_2,
    cupsSpecial_1,
    cupsSpecial_2,
    HotWaterTemp,
    ExtractionTime,
    BladeNofTimesUsed,
    RonNofTimesUsed,
    Brand,
    SeriNumber,
    Model,
    ProductDate,
    tempExtrude,
    PulseCount,

};

#endif /* COFFEE_MACHINE_H_ */
