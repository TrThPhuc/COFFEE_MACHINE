/*
 * DefautParameter.c
 *
 *  Created on: Jul 13, 2021
 *      Author: 16126
 */
#include "Coffee_Machine.h"
#include "TCA9539.h"
#include "ADS1118.h"
extern Mode_Parameter_t Mode_Espresso_1, Mode_Espresso_2, Mode_Special_1,
        Mode_Special_2;
extern TCA9539Regs TCA9539_IC1, TCA9539_IC2, TCA9539_IC3;
extern bool Hyteresis, ErSteam, ErHotWater, ErOutletDetect, ErHomeReturn,
        ErSteamTimeOut, ErHotWaterTimeOut, ErNoPumpPulse;
//extern float Steam_Temperature_Ref;
extern float HotWater_Temperature_Ref, Steam_Temperature_Ref;
extern uint16_t PitchOfpress;
extern _Bool Pr_PacketCopyMask[];
extern uint32_t wBladeATimes, wBladeBTimes, wExtractionATimes,
        wExtractionBTimes, wGroupDuty, wGroupShutdownDuty;
extern uint32_t wExtract_MaxTime, wExtract_MinTime;
extern uint16_t BladeA, BladeB, Ron;
void ParameterDefaultSetting()
{
    // Small size
    Mode_Special_1.PreInfusion = 4;
    Mode_Special_1.PosInfusion = 2;
    Mode_Special_1.SpeedMotorInfusion = 0.28;
    Mode_Special_1.Time = 25;                            // Not used

    Mode_Special_1.GrindingDuration = 9.3;               // 9.3 s
    Mode_Special_1.WeigtOfPowder = 25;                   // Not used
    Mode_Special_1.AmountOfWaterPumping.stage_1 = 142;
    Mode_Special_1.Pitch = 13;
    Mode_Special_1.Cups = 0;

    Mode_Special_1.smallSize = 1;
    Mode_Special_1.IndexMode = 2;
    Mode_Special_1.MotorPreInfusion = haveMotor;
//-----------------------------------------------------
    // Big size
    Mode_Special_2.PreInfusion = 4;
    Mode_Special_2.PosInfusion = 2;
    Mode_Special_2.SpeedMotorInfusion = 0.28;
    Mode_Special_2.Time = 25;                           // Not used

    Mode_Special_2.GrindingDuration = 11.5;               // 9.3s
    Mode_Special_2.WeigtOfPowder = 35;                  // Not used
    Mode_Special_2.AmountOfWaterPumping.stage_1 = 195;
    Mode_Special_2.Pitch = 13;
    Mode_Special_2.Cups = 0;

    Mode_Special_2.smallSize = 0;
    Mode_Special_2.IndexMode = 3;
    Mode_Special_2.MotorPreInfusion = haveMotor;
    //-----------------------------------------------------

    // Small size
    Mode_Espresso_1.PreInfusion = 4;
    Mode_Espresso_1.PosInfusion = 2;
    Mode_Espresso_1.SpeedMotorInfusion = 0.28;

    Mode_Espresso_1.Time = 25;                            // Not used

    Mode_Espresso_1.GrindingDuration = 9.3;               //9.3s
    Mode_Espresso_1.WeigtOfPowder = 25;                   // Not used
    Mode_Espresso_1.AmountOfWaterPumping.stage_1 = 142;
    Mode_Espresso_1.Pitch = 13;
    Mode_Espresso_1.Cups = 0;

    Mode_Espresso_1.smallSize = 1;
    Mode_Espresso_1.IndexMode = 0;
    Mode_Espresso_1.MotorPreInfusion = haveMotor;
    //-----------------------------------------------------

    // Big size
    Mode_Espresso_2.PreInfusion = 4;
    Mode_Espresso_2.PosInfusion = 2;
    Mode_Espresso_2.SpeedMotorInfusion = 0.28;
    Mode_Espresso_2.Time = 25;                              // Not used

    Mode_Espresso_2.GrindingDuration = 11.5;                  //9.3s
    Mode_Espresso_2.WeigtOfPowder = 35;                     // Not used
    Mode_Espresso_2.AmountOfWaterPumping.stage_1 = 195;
    Mode_Espresso_2.Pitch = 13;
    Mode_Espresso_2.Cups = 0;

    Mode_Espresso_2.smallSize = 0;
    Mode_Espresso_2.IndexMode = 1;
    Mode_Espresso_2.MotorPreInfusion = haveMotor;
    //-----------------------------------------------------
    HotWater_Temperature_Ref = 92;
    Steam_Temperature_Ref = 115;
    PitchOfpress = 15;
    wBladeATimes = 20000;
    wBladeBTimes = 20000;
    wExtractionATimes = 80000;
    wExtractionBTimes = 80000;

    wExtract_MaxTime = 50;
    wExtract_MinTime = 25;

    BladeA = 0;
    BladeB = 0;
    Ron = 0;
    wGroupDuty = 50;
    wGroupShutdownDuty = 5;
}
void AssignParameterForMode(Mode_Parameter_t *thisMode, uint32_t **vPar)
{
    if (thisMode->IndexMode > 3)
        return;
    uint8_t frameOfEachMode = 8;
    uint8_t index = thisMode->IndexMode * frameOfEachMode;
    vPar[index + 0] = (uint32_t*) &thisMode->PreInfusion;
    vPar[index + 1] = (uint32_t*) &thisMode->PosInfusion;
    vPar[index + 2] = (uint32_t*) &thisMode->MotorPreInfusion;
    vPar[index + 3] = (uint32_t*) &thisMode->SpeedMotorInfusion;
    vPar[index + 4] = (uint32_t*) &thisMode->AmountOfWaterPumping.stage_1;
    vPar[index + 5] = (uint32_t*) &thisMode->GrindingDuration;
    vPar[index + 6] = (uint32_t*) &thisMode->Pitch;
}
void AssignGobalParSetting(uint32_t **vPar)
{
    uint8_t IndexGobalPar = 32;
    vPar[IndexGobalPar + 0] = (uint32_t*) &wBladeATimes;
    vPar[IndexGobalPar + 1] = (uint32_t*) &wBladeBTimes;
    vPar[IndexGobalPar + 2] = (uint32_t*) &wExtractionATimes;
    vPar[IndexGobalPar + 3] = (uint32_t*) &wExtractionBTimes;
    vPar[IndexGobalPar + 4] = (uint32_t*) &wExtract_MaxTime;
    vPar[IndexGobalPar + 5] = (uint32_t*) &wExtract_MinTime;
    vPar[IndexGobalPar + 6] = (uint32_t*) &HotWater_Temperature_Ref;
    vPar[IndexGobalPar + 7] = (uint32_t*) &Steam_Temperature_Ref;
    vPar[IndexGobalPar + 8] = (uint32_t*) &wGroupDuty;
    vPar[IndexGobalPar + 9] = (uint32_t*) &wGroupShutdownDuty;



}
void AssignErrorList(void)
{
    uint8_t i;
    for (i = 0; i < 16; i++)
        ErrorMachine[i].ErrorFlag = NULL;
//---------------------------------------------------------------------------------------------
    ErrorMachine[eTCA_Ic1].ErrorFlag = &TCA9539_IC1.ErrorFlag; // Loi giao tiep i2c ic mo rong 1 tca
    ErrorMachine[eTCA_Ic2].ErrorFlag = &TCA9539_IC2.ErrorFlag; // Loi giao tiep i2c ic mo rong 2 tca
    ErrorMachine[eTCA_Ic3].ErrorFlag = &TCA9539_IC3.ErrorFlag; // Loi giao tiep i2c ic  mo rong 3 tca
    ErrorMachine[eAds11118].ErrorFlag = &eCom_Ads1118; //Loi giao tiep ic cam bien ads118
    ErrorMachine[eFaultMotor].ErrorFlag = NULL;
    ErrorMachine[eLevelSensor].ErrorFlag = &Hyteresis;  // Loi cam bien muc
    ErrorMachine[eHotWaterOpen].ErrorFlag = &ErHotWater; // Loi cam bien nhiet binh nc nong
    ErrorMachine[eHotWaterTimeOut].ErrorFlag = &ErHotWaterTimeOut; // Loi ko dun binh nc nong
    ErrorMachine[eSteamOpen].ErrorFlag = &ErSteam;  // loi cam bien binh hoi
    ErrorMachine[eSteamTimeOut].ErrorFlag = &ErSteamTimeOut; // Loi ko dun binh hoi
    ErrorMachine[ePumpPulseFast].ErrorFlag = NULL;
    ErrorMachine[ePumpPulseSlow].ErrorFlag = NULL;
    ErrorMachine[eNoPumpPulse].ErrorFlag = &ErNoPumpPulse; // Loi ko co xung luu luong
    ErrorMachine[eCoffeOutletDetect].ErrorFlag = &ErOutletDetect; // Loi cam bien ca phe ra
    ErrorMachine[eHomeReturn].ErrorFlag = &ErHomeReturn;   // loi ve home ko bat
//---------------------------------------------------------------------------------------------

    ErrorMachine[eTCA_Ic1].ErrorMsg = "Loi giao tiep tca1";
    ErrorMachine[eTCA_Ic1].ErrorMsg = "Loi giao tiep tca2";
    ErrorMachine[eTCA_Ic1].ErrorMsg = "Loi giao tiep tca3";
    ErrorMachine[eAds11118].ErrorMsg = "Loi giao tiep ads1118";
    ErrorMachine[eFaultMotor].ErrorMsg = "Loi dong co ep";
    ErrorMachine[eLevelSensor].ErrorMsg = "Loi Cam bien muc";
    ErrorMachine[eHotWaterOpen].ErrorMsg = "Loi cam bien nhiet nc";
    ErrorMachine[eHotWaterTimeOut].ErrorMsg = "Loi ko dun nuoc";
    ErrorMachine[eSteamOpen].ErrorMsg = "Loi cam bien nhiet hoi";
    ErrorMachine[eSteamTimeOut].ErrorMsg = "Loi ko dun hoi";
    ErrorMachine[ePumpPulseFast].ErrorMsg = "";
    ErrorMachine[ePumpPulseSlow].ErrorMsg = "";
    ErrorMachine[eNoPumpPulse].ErrorMsg = "Loi ko xung luu luong";
    ErrorMachine[eCoffeOutletDetect].ErrorMsg = "loi cam bien cafe ra";
    ErrorMachine[eHomeReturn].ErrorMsg = "Loi ve home";

}
