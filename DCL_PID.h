/*
 * DCL_PID.h
 *
 *  Created on: Apr 29, 2022
 *      Author: 16126
 */

#ifndef DCL_PID_H_
#define DCL_PID_H_

#include "stdint.h"
#include "stdbool.h"
typedef struct PID_DCL_DBUFF
{
    bool isat;
    float i1save;
    float d1save;
    float d2save;

} PID_DCL_DBUFF_t;

typedef struct CNTL_PID_DCL_Coef
{
    float Kp;   // Propotional gain
    float Ki;   // Interal gain
    float Kd;   // Derivative gain
    float kr;   // Set point weight
    float c1;   // Derivative fillter coeficient
    float c2;   // Derivative fillter coeficient
    float Max;
    float Min;

} CNTL_PID_DCL_Coef_t;

typedef struct CNTL_PID_DCL_Terminal
{
    float *Ref;
    float *Fdbk;
    float *Out;
    bool ik;
    CNTL_PID_DCL_Coef_t Coef;
    PID_DCL_DBUFF_t DUFF_Str;
} CNTL_PID_DCL_Terminal_t;
void PID_DCL_cal(CNTL_PID_DCL_Terminal_t *CNTL, float kp, float kd, float ki,
                 float kr, float max, float min);
void CNTL_PID_DCL(CNTL_PID_DCL_Terminal_t *CNTL);
#endif /* DCL_PID_H_ */
