/*
 * DCL_PID.c
 *
 *  Created on: Apr 29, 2022
 *      Author: 16126
 */
#include "DCL_PID.h"
void PID_DCL_cal(CNTL_PID_DCL_Terminal_t *CNTL, float kp, float kd, float ki,
                 float kr, float max, float min)
{
    CNTL->Coef.Kp = kp;
    CNTL->Coef.Kd = kd;
    CNTL->Coef.Ki = ki;
    CNTL->Coef.kr = kr;
    CNTL->Coef.Max = max;
    CNTL->Coef.Min = min;
}

void CNTL_PID_DCL(CNTL_PID_DCL_Terminal_t *CNTL)
{
    uint32_t satWindup;
    _Bool vk = false;
    float Out;
    //*** proportional path ***
    float v1 = (*CNTL->Ref * CNTL->Coef.kr) - *CNTL->Fdbk; // v1 = Kr*r(k) - y(k)

    //*** interal path ***
    if (CNTL->DUFF_Str.isat)
        satWindup = 0;       // satWindup = 0 when ik(k-1) = 0 and vk(k - 1) = 0
    else
        satWindup = 1;        // satWindup = 0 when ik(k-1) = 1 or vk(k - 1) = 1

    // v2 = v2(k-1) + satWindup*kp*ki*(r(k) - i(k))
    float v2 = CNTL->DUFF_Str.i1save
            + satWindup * CNTL->Coef.Kp * CNTL->Coef.Ki
                    * (*CNTL->Ref - *CNTL->Fdbk);
    CNTL->DUFF_Str.i1save = v2; // save v2 to use v2(k-1)
    //*** derivative path ***
    // v3  = kd*c1*y(k)
    float v3 = CNTL->Coef.Kd * CNTL->Coef.c1 * (*CNTL->Fdbk);
    // v4 = v3 - d2(k) - d3(k);
    float v4 = v3 - CNTL->DUFF_Str.d1save - CNTL->DUFF_Str.d2save;
    // d1(k) = v3(k-1)
    CNTL->DUFF_Str.d1save = v3; // save v3 to use v3(k-1)
    // d2(k) = c2 * v4(k-1)
    CNTL->DUFF_Str.d2save = CNTL->Coef.c2 * v4; // save v4 to use v4(k-1)

    //*** output path ***
    // v5 = Kp * (v1 - v4)
    float v5 = CNTL->Coef.Kp * (v1 - v4);
    // out = kp * (Kr*r(k) - y(k) - v4)) + v2(k-1) + satWindup*kp*ki*(r(k) - i(k))
    Out = v5 + v2;

    // Out > max
    if (Out > CNTL->Coef.Max)
    {
        Out = CNTL->Coef.Max;
        vk = true;
    }else if (Out < CNTL->Coef.Min)
    {
        Out = CNTL->Coef.Min;
        vk = true;
    }
    else
        vk = false;
    // anti swing up
    if (vk || (CNTL->ik))
        CNTL->DUFF_Str.isat = true;
    else
        CNTL->DUFF_Str.isat = false;
    *CNTL->Out = Out;

}
