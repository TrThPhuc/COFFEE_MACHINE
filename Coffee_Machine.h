/*
 * Coffee_Machine.h
 *
 *  Created on: Jun 23, 2021
 *      Author: 16126
 */

#ifndef COFFEE_MACHINE_H_
#define COFFEE_MACHINE_H_

struct AmountofWater
{
    uint16_t stage_1;   //  Low flow
    uint16_t stage_2;   //  High flow
};
typedef struct Mode_Parameter
{
    uint16_t Water;
    struct AmountofWater AmountOfWaterPumping;
    uint16_t GrindingDuration;
    bool DirGrinding;

} Mode_Parameter_t;

#endif /* COFFEE_MACHINE_H_ */
