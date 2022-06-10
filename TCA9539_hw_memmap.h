/*
 * TCA9539_hw_memmap.h
 *
 *  Created on: Jun 23, 2021
 *      Author: 16126
 */

#ifndef TCA9539_HW_MEMMAP_H_
#define TCA9539_HW_MEMMAP_H_

// TCA9539 IC 1
// Port 0
#define Menu_Bt                 0x01        // Bit 0
#define Up_Bt                   0x02        // Bit 1
#define Down_Bt                 0x04        // Bit 2
#define Expresso1_Bt            0x08        // Bit 3

#define Special1_Bt             0x10        // Bit 4
#define Expresso2_Bt            0x20        // Bit 5
#define Special2_Bt             0x40        // Bit 6
#define Cancel_Task             0x80        // Bit 7

// Port 1
#define LED_BT8                 0x100       // Bit 0
#define LED_BT9                 0x200       // Bit 1
#define Clean_Bt                0x400       // Bit 2
#define LED_BT7                 0x800       // Bit 3

#define LED_BT6                 0x1000      // Bit 4
#define LED_BT5                 0x2000      // Bit 5
#define LED_BT4                 0x4000      // Bit 6
#define Rinse_Bt                0x8000      // Bit 7
//----------------------- TCA9539 IC 2 ---------------------------------//

// Port 0
#define LCD_back_Light          0x10        // Bit 4
#define Warming_Bt              0x20        // Bit 5
#define Brake_BLDC3             0x40        // Bit 6
#define SpeedFeedBack_BLDC3     0x80        // Bit 7

// Port 1
#define Alarm_BLDC_3            0x100       // Bit 0
#define Enable_BLDC3            0x200       // Bit 1
#define Direction_BLDC3         0x400       // Bit 2
#define SpeedFeedBack_BLDC2     0x800       // Bit 3

#define Direction_BLDC2         0x1000      // Bit 4
#define Enable_BLDC2            0x2000      // Bit 5
#define Valve_5                 0x4000      // Bit 6
#define Alarm_BLDC_2            0x8000      // Bit 7

//-----------------------TCA9539 IC 3 ---------------------------------//
// Port 0
#define Valve_2                 0x10        //  Bit 4
#define Alarm_BLDC_1            0x20        //  Bit 5
#define Valve_4                 0x40        //  Bit 6
#define Enable_BLDC1            0x80        //  Bit 7
//Port 1
#define Direction_BLDC1         0x100       // Bit 0
#define Valve_3                 0x200       // Bit 1
#define SpeedFeedBack_BLDC1     0x400       // Bit 2
#define Valve_1                 0x800       // Bit 3

#define LinitSwitch             0x1000      // Bit 4
#define OutletSensor            0x2000      // Bit 5
#define WarningPressMotor       0x4000      // Bit 6
#define Input_4                 0x8000      // Bit 7

#endif /* TCA9539_HW_MEMMAP_H_ */
