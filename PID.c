/*
 * PID.c
 *
 *  Created on: Jun 25, 2021
 *      Author: 16126
 */

#include "PID.h"
extern uint8_t coef_change; // Pgain_Gui, Igain_Gui, Dgain_Gui;
void CNTL_Pole_Zero_Cal(CNTL_2P2Z_Terminal_t *CNTL, uint32_t Pgain_Gui,
                        uint32_t Igain_Gui, uint32_t Dgain_Gui, float max,
                        float i_min, float min)
{
    coef_change = 1;
    float Pgain = Pgain_Gui * 0.001;      // Q26  *0.001
    float Igain = Igain_Gui * 0.001;      // Q26  *0.001
    float Dgain = Dgain_Gui * 0.001;      // Q26  *0.001
    CNTL->Coef.b2 = Dgain;
    CNTL->Coef.b1 = (Igain - Pgain - Dgain - Dgain);
    CNTL->Coef.b0 = (Pgain + Igain + Dgain);
    CNTL->Coef.a2 = 0.0;
    CNTL->Coef.a1 = 1.0;
    CNTL->Coef.max = max;
    CNTL->Coef.min = min;
    CNTL->Coef.i_min = i_min;
    coef_change = 0;
}
void CNTL_2P2Z(CNTL_2P2Z_Terminal_t *CNTL)
{
    // The controll law:
//out = e(n-2)*b2 + e(n-1)*b1 + e*b0 + u(n-2)*a2 + u(n-1)*a1
    float ACC;
    CNTL->DBUFF.e = *CNTL->Ref - *CNTL->Fdbk;   //e(n) = (Ref - Fdbk)
    ACC = CNTL->DBUFF.e2 * CNTL->Coef.b2;       // ACC = e(n-2)*b2
    CNTL->DBUFF.e2 = CNTL->DBUFF.e1;            // e(n-2) = e(n-1)
    ACC += CNTL->DBUFF.e1 * CNTL->Coef.b1;      // ACC = e(n-2)*b2 + e(n-1)*b1
    CNTL->DBUFF.e1 = CNTL->DBUFF.e;             // e(n-1) = e
    ACC += CNTL->DBUFF.e * CNTL->Coef.b0;  // ACC = e(n-2)*b2 + e(n-1)*b1 + e*b0
    ACC += CNTL->DBUFF.U2 * CNTL->Coef.a2; // ACC = e(n-2)*b2 + e(n-1)*b1 + e*b0 + u(n-2)*a2
    CNTL->DBUFF.U2 = CNTL->DBUFF.U1;            // u(n-2) = u(n-1);
    ACC += CNTL->DBUFF.U1 * CNTL->Coef.a1; // ACC = e(n-2)*b2 + e(n-1)*b1 + e*b0 + u(n-2)*a2 + u(n-1)*a1
    if (ACC > CNTL->Coef.max)                   // (max > ACC > i_min)
        ACC = CNTL->Coef.max;
    if (ACC < CNTL->Coef.i_min)
        ACC = CNTL->Coef.i_min;
    CNTL->DBUFF.U1 = ACC;                       // u(n-1)  = acc = u(n)
    if (ACC < CNTL->Coef.min)                   //  (max > u(n) > min)
        ACC = CNTL->Coef.min;
    *CNTL->Out = ACC;                           // out = u(n)
}
