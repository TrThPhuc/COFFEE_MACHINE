/*
 * DefautParameter.c
 *
 *  Created on: Jul 13, 2021
 *      Author: 16126
 */
#include "Coffee_Machine.h"
extern Mode_Parameter_t Mode_Espresso_1, Mode_Espresso_2, Mode_Special_1,
        Mode_Special_2;
//extern float Steam_Temperature_Ref;
extern float HotWater_Temperature_Ref;
extern uint16_t PitchOfpress;
void ParameterDefaultSetting()
{
    Mode_Special_1.PreInfusion = 4;
    Mode_Special_1.Time = 25;

    Mode_Special_1.GrindingDuration = 86;   //100ms
    Mode_Special_1.WeigtOfPowder = 0;
    Mode_Special_1.AmountOfWaterPumping.stage_1 = 700;


    Mode_Special_1.Cups = 0;
//-----------------------------------------------------
    Mode_Special_2.PreInfusion = 4;
    Mode_Special_2.Time = 25;

    Mode_Special_2.GrindingDuration = 86;  // 100ms
    Mode_Special_2.WeigtOfPowder = 0;
    Mode_Special_2.AmountOfWaterPumping.stage_1 = 700;


    Mode_Special_2.Cups = 0;
    //-----------------------------------------------------
    Mode_Espresso_1.PreInfusion = 4;
    Mode_Espresso_1.Time = 25;

    Mode_Espresso_1.GrindingDuration = 86;     //100ms
    Mode_Espresso_1.WeigtOfPowder = 0;
    Mode_Espresso_1.AmountOfWaterPumping.stage_1 = 700;

    Mode_Espresso_1.Cups = 0;
    //-----------------------------------------------------
    Mode_Espresso_2.PreInfusion = 4;
    Mode_Espresso_2.Time = 25;

    Mode_Espresso_2.GrindingDuration = 86;     //100ms
    Mode_Espresso_2.WeigtOfPowder = 0;
    Mode_Espresso_2.AmountOfWaterPumping.stage_1 = 700;

    Mode_Espresso_2.Cups = 0;
    //-----------------------------------------------------
    HotWater_Temperature_Ref = 92;
    PitchOfpress = 15;

}

