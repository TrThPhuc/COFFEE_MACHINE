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
uint32_t pos1 = 270, pos2 = 260;  // position for compress process

// Pumping timming
uint8_t t1 = 200, t2 = 10;     // for test pre-infusion

// Grinding process
uint32_t GringPWMIncrement = 100;   // Ram speed grinding motor
float K_VrTimer_Grinding = 25.0; //21.28

// Pumping process
#define PreInfusion_pump        2999
#define HighPressure_Pump   7999
#define MaxSpeedGring 9999

#define MaxVolume 1000;

// Coffee extraction time
uint32_t CoffeeExtractionTime;
extern float Gui_CoffeExtractionTime;
bool triggerCount_ExtractionTime;
#endif /* CM_LOWLEVEL_CMD_H_ */
