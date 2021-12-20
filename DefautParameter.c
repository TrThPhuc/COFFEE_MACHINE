/*
 * DefautParameter.c
 *
 *  Created on: Jul 13, 2021
 *      Author: 16126
 */
#include "Coffee_Machine.h"
extern Mode_Parameter_t Mode_Espresso_1, Mode_Espresso_2, Mode_Decatt_1,
        Mode_Decatt_2;
//extern float Steam_Temperature_Ref;
extern float HotWater_Temperature_Ref;
extern uint16_t PitchOfpress;
void ParameterDefaultSetting()
{
    Mode_Decatt_1.PreInfusion = 4;
    Mode_Decatt_1.Time = 25;

    Mode_Decatt_1.GrindingDuration = 500;
    Mode_Decatt_1.AmountOfWaterPumping.stage_1 = 320;
    Mode_Decatt_1.AmountOfWaterPumping.stage_2 = 300;

    Mode_Decatt_2.PreInfusion = 4;
    Mode_Decatt_2.Time = 25;
    Mode_Decatt_2.GrindingDuration = 500;
    Mode_Decatt_2.AmountOfWaterPumping.stage_1 = 320;
    Mode_Decatt_2.AmountOfWaterPumping.stage_2 = 300;

    Mode_Espresso_1.PreInfusion = 4;
    Mode_Espresso_1.Time = 25;
    Mode_Espresso_1.GrindingDuration = 500;
    Mode_Espresso_1.AmountOfWaterPumping.stage_1 = 320;
    Mode_Espresso_1.AmountOfWaterPumping.stage_2 = 300;

    Mode_Espresso_2.PreInfusion = 4;
    Mode_Espresso_2.Time = 25;
    Mode_Espresso_2.GrindingDuration = 500;
    Mode_Espresso_2.AmountOfWaterPumping.stage_1 = 320;
    Mode_Espresso_2.AmountOfWaterPumping.stage_2 = 300;

    HotWater_Temperature_Ref = 91;
    PitchOfpress = 15;

}

