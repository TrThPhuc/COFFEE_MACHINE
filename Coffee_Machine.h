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
struct AmountofWater
{
    uint16_t stage_1;   //  Low flow
    uint16_t stage_2;   //  High flow
};
typedef struct Mode_Parameter
{
    uint16_t Water;
    struct AmountofWater AmountOfWaterPumping;
    uint16_t GrindingDuration;
    bool DirGrinding;

} Mode_Parameter_t;
void ParameterDefaultSetting();
#endif /* COFFEE_MACHINE_H_ */
