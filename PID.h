/*
 * PID.h
 *
 *  Created on: Jun 25, 2021
 *      Author: 16126
 */
//out = e(n-2)*b2 + e(n-1)*b1 + e*b0 + u(n-2)*a2 + u(n-1)*a1
#ifndef PID_H_
#define PID_H_
#include "stdbool.h"
#include "stdint.h"
typedef struct CNTL_2P2Z_DBUFF
{
    float U2;   // u(n-2)
    float U1;   // u(n-1)
    float e;    // e(n)
    float e1;   // e(n-1)
    float e2;   // e(n-2)

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
    bool coef_change;
    CNTL_2P2Z_Coef_t Coef;
    CNTL_2P2Z_DBUFF_t DBUFF;
} CNTL_2P2Z_Terminal_t;

void CNTL_2P2Z(CNTL_2P2Z_Terminal_t*);
void CNTL_Pole_Zero_Cal(CNTL_2P2Z_Terminal_t *CNTL, uint32_t Pgain_Gui,
                        uint32_t Igain_Gui, uint32_t Dgain_Gui, float max,
                        float i_min, float min);
#endif /* PID_H_ */
