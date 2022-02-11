/*
 * CM_lowLevel_Cmd.h
 *
 *  Created on: Dec 24, 2021
 *      Author: 16126
 */

#ifndef CM_LOWLEVEL_CMD_H_
#define CM_LOWLEVEL_CMD_H_

// ------------------------ User Setting for process make coffee ---------------------------
//Unit timer for virtual timer - Based on timer 4
#define UnitTimer 0.02 // 0.02s - 20ms

// Compress process
uint32_t pos1 = 118, pos2 = 149;  // position for compress process
uint32_t stepPos1 = 4500, stepPos2 = 5200, stepPos3 = 800;
//175 210
// Pumping timming
uint8_t t1 = 200, t2 = 10;     // for test pre-infusion

// Grinding process
uint32_t PWMIncrement = 10;   // Ram speed grinding motor
float K_VrTimer_Grinding = 25.0; //21.28
uint32_t speedTemp;
// Pumping process
#define PreInfusion_pump   700
#define HighPressure_Pump   5000
#define MaxSpeed 6000  //6700
uint32_t speedgrind = 50000;
#define MaxVolume 1000;

// Coffee extraction time
uint32_t CoffeeExtractionTime;
extern float Gui_CoffeExtractionTime;
bool triggerCount_ExtractionTime;

extern float vel;
uint32_t sp1 = 3000;
uint32_t sp2 = 3200;
uint32_t sp3 = 3400;
float infu = 2;
uint32_t speedstep = 12000;
uint32_t pp1, pp2, pp3, ppo;
float ppi = 0;
float buffervel[8], tempv, avgvel;
#define k1 0.3
#define k2 1
#define k3 1
#endif /* CM_LOWLEVEL_CMD_H_ */
