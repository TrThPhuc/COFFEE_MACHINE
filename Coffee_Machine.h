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
#define SysCltEeprom
//#define SaveEeprom
struct AmountofWater
{
    uint16_t stage_1;   //  Low flow
    uint16_t stage_2;   //  High flow
};
typedef struct Mode_Parameter
{
    uint16_t PreInfusion;    // Pre-Infusion Time - second
    uint16_t Time;           // Time for extraction - second

    struct AmountofWater AmountOfWaterPumping;
    uint16_t GrindingDuration;  // Grinding interval
    bool DirGrinding; // CW or CCW Grinding

} Mode_Parameter_t;
void ParameterDefaultSetting();
#endif /* COFFEE_MACHINE_H_ */
