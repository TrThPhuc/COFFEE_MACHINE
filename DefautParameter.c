/*
 * DefautParameter.c
 *
 *  Created on: Jul 13, 2021
 *      Author: 16126
 */
#include "Coffee_Machine.h"
extern Mode_Parameter_t Mode_Espresso_1, Mode_Espresso_2, Mode_Decatt_1,
        Mode_Decatt_2;
void ParameterDefaultSetting()
{
    Mode_Decatt_1.GrindingDuration = 80;
    Mode_Decatt_1.AmountOfWaterPumping.stage_1 = 300;
    Mode_Decatt_1.AmountOfWaterPumping.stage_2 = 300;

    Mode_Decatt_2.GrindingDuration = 80;
    Mode_Decatt_2.AmountOfWaterPumping.stage_1 = 300;
    Mode_Decatt_2.AmountOfWaterPumping.stage_2 = 300;

    Mode_Espresso_1.GrindingDuration = 80;
    Mode_Espresso_1.AmountOfWaterPumping.stage_1 = 300;
    Mode_Espresso_1.AmountOfWaterPumping.stage_2 = 300;

    Mode_Espresso_2.GrindingDuration = 80;
    Mode_Espresso_2.AmountOfWaterPumping.stage_1 = 300;
    Mode_Espresso_2.AmountOfWaterPumping.stage_2 = 300;

}

