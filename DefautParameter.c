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
extern float HotWater_Temperature_Ref, Steam_Temperature_Ref;
extern uint16_t PitchOfpress;
extern float PulWeightRatio;
extern _Bool Pr_PacketCopyMask[];
void ParameterDefaultSetting()
{
    // Small size
    Mode_Special_1.PreInfusion = 4;
    Mode_Special_1.Time = 25;                            // Not used

    Mode_Special_1.GrindingDuration = 9.3;               // 9.3 s
    Mode_Special_1.WeigtOfPowder = 25;                   // Not used
    Mode_Special_1.AmountOfWaterPumping.stage_1 = 142;
    Mode_Special_1.Pitch = 13;
    Mode_Special_1.Cups = 0;

    Mode_Special_1.smallSize = 1;
//-----------------------------------------------------
    // Big size
    Mode_Special_2.PreInfusion = 4;
    Mode_Special_2.Time = 25;                           // Not used

    Mode_Special_2.GrindingDuration = 11.5;               // 9.3s
    Mode_Special_2.WeigtOfPowder = 35;                  // Not used
    Mode_Special_2.AmountOfWaterPumping.stage_1 = 195;
    Mode_Special_2.Pitch = 13;
    Mode_Special_2.Cups = 0;

    Mode_Special_2.smallSize = 0;
    //-----------------------------------------------------

    // Small size
    Mode_Espresso_1.PreInfusion = 4;
    Mode_Espresso_1.Time = 25;                            // Not used

    Mode_Espresso_1.GrindingDuration = 9.3;               //9.3s
    Mode_Espresso_1.WeigtOfPowder = 25;                   // Not used
    Mode_Espresso_1.AmountOfWaterPumping.stage_1 = 142;
    Mode_Espresso_1.Pitch = 13;
    Mode_Espresso_1.Cups = 0;

    Mode_Espresso_1.smallSize = 1;
    //-----------------------------------------------------

    // Big size
    Mode_Espresso_2.PreInfusion = 4;
    Mode_Espresso_2.Time = 25;                              // Not used

    Mode_Espresso_2.GrindingDuration = 11.5;                  //9.3s
    Mode_Espresso_2.WeigtOfPowder = 35;                     // Not used
    Mode_Espresso_2.AmountOfWaterPumping.stage_1 = 195;
    Mode_Espresso_2.Pitch = 13;
    Mode_Espresso_2.Cups = 0;

    Mode_Espresso_2.smallSize = 0;
    //-----------------------------------------------------
    HotWater_Temperature_Ref = 92;
    Steam_Temperature_Ref = 120;
    PitchOfpress = 15;

    PulWeightRatio = 30;

}

