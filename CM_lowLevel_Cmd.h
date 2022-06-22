/*
 * CM_lowLevel_Cmd.h
 *
 *  Created on: Dec 24, 2021
 *      Author: 16126
 */

#ifndef CM_LOWLEVEL_CMD_H_
#define CM_LOWLEVEL_CMD_H_

enum status
{
    St_Null, St_Finish, St_Running, St_Pending
};

typedef enum
{
    M_ReturnHome1,
    M_GrindPos,
    M_GrindingCoffee,
    M_ExtractPos,
    M_ExtractCoffee,
    M_PosCompress,
    M_ReturnHome2,
    M_RemoveGround,
    M_finishProcess

} StepInMakeCoffeProcess;
StepInMakeCoffeProcess M_Step;
typedef enum
{
    Cl_ReturnHome1,
    Cl_CompressPos1,
    Cl_InsertPodClean,
    Cl_CompressPos2,
    Cl_Pumping1,
    Cl_CompressPos3,
    Cl_ReturnHome2,
    Cl_Pumping2,
    Cl_FinishCLean,
} StepInCleanningProcess;
StepInCleanningProcess Cl_Step;

typedef enum
{
    Wa_ReturnHome1,
    Wa_CompressPos1,
    Wa_WarmingBrew,
    Wa_CompressPos2,
    Wa_ReturnHome2,
    Wa_FinishWarm
} StepInWarmingBrewProcess;
StepInWarmingBrewProcess Wa_Step;

typedef enum
{
    Wm_TurnOnHeating, Wm_finish
} StepInWarmingProcess;
StepInWarmingProcess Wm_Step;

// ------------------------ User Setting for process make coffee ---------------------------

#define M_Build 0

//Unit timer for virtual timer - Based on timer 4
#define UnitTimer 0.02 // 0.02s - 20ms

// Compress process

#define dirCompress_M2          true
#define dirCompressBack_M2      false

#define dirReturnHome_M2        false
#define ReleasePressure 1

// stepPos1 - grindpos
// stepPos2 - pre-extract
// stepPos3 - remove grounds
// stepPos4 - pos-compress
// stepPos5 - pos-extraction
uint32_t pos1 = 118, pos2 = 149;  // position for compress process
uint32_t stepPos1 = 4100, stepPos2 = 4000, stepPos3 = 800, stepPos4 = 200,
        stepPos5 = 100; // Extract Pos  //600
// unit X (mm) * 100 = stepPos

//175 210
// Pumping timming
uint8_t t1 = 200, t2 = 10;     // for test pre-infusion

// Grinding process
uint32_t PWMIncrement = 50;   // Ram speed grinding motor
float K_VrTimer_Grinding = 25.0; //21.28
uint32_t speedTemp, defaulSpeed = 11000;
// Pumping process
#define PreInfusion_pump    0.28     // 1000
#define HighPressure_Pump   0.99    // 5000
#define CleanPressure_Pump  0.99

#define MaxSpeed 6000  //6700
#define MaxVolume 1000
#define CleanVolume 200
uint32_t speedgrind = 50000;

// Coffee extraction time

extern float Gui_CoffeeExtractionTime, Gui_CoffeePreinfusionTime,
        Gui_CoffeeGrindTime;
extern uint8_t countGrounds;
extern bool fullOfGroundsDrawer, PreGrindFlag;
bool triggerCount_ExtractionTime, triggerCount_PreinfusionTime;
uint32_t CoffeeExtractionTime, CoffeePreinfusionTime;
extern float vel;
uint32_t sp1 = 2900;
uint32_t sp2 = 3200;
uint32_t sp3 = 3400;
#define k1 0.35
#define k2 0.5
#define k3 1
float infu = 2, infu_cl = 60;
uint32_t speedstep = 12000;
uint32_t ppo;

float buffervel[8], tempv, avgvel;

#define timeRinse 6
#define timeClear 60
extern uint8_t idModeRunning;
extern uint16_t *CountDataStorage[];

uint8_t TargetTest;
float speedPumpTest;
bool dirGrindTest, dirCompressTest, InModuleTest, TestModuleTrigger;

#define PosExtract 1

void ModuleTest(void);
void CheckFinsih_ModuleTest(void);
void ModuleTestMachine(void);
#define Timming(x)  ((x)/0.02f)
float ManualRun_DutyPump;

#endif /* CM_LOWLEVEL_CMD_H_ */
