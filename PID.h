/*
 * PID.h
 *
 *  Created on: Jun 25, 2021
 *      Author: 16126
 */

#ifndef PID_H_
#define PID_H_
#include "stdbool.h"
#include "stdint.h"
typedef struct CNTL_2P2Z_DBUFF
{
    float U2;
    float U1;
    float e;
    float e1;
    float e2;

} CNTL_2P2Z_DBUFF_t;
typedef struct CNTL_2P2Z_CoefStruct
{
    float b2;
    float b1;
    float b0;
    float a2;
    float a1;
    float max;
    float i_min;
    float min;

} CNTL_2P2Z_Coef_t;
typedef struct CNTL_2P2Z_Terminal
{
    volatile float *Ref;
    volatile float *Fdbk;
    volatile float *Out;
    CNTL_2P2Z_Coef_t Coef;
    CNTL_2P2Z_DBUFF_t DBUFF;
} CNTL_2P2Z_Terminal_t;

void CNTL_2P2Z(CNTL_2P2Z_Terminal_t*);
void CNTL_Pole_Zero_Cal(CNTL_2P2Z_Terminal_t *CNTL, uint32_t Pgain_Gui,
        uint32_t Igain_Gui, uint32_t Dgain_Gui, float max,
        float i_min, float min);
#endif /* PID_H_ */
