/*
 * TCA9539_hw_memmap.h
 *
 *  Created on: Jun 23, 2021
 *      Author: 16126
 */

#ifndef TCA9539_HW_MEMMAP_H_
#define TCA9539_HW_MEMMAP_H_

// TCA9539 IC 1
#define Menu_Bt                 0x01
#define Up_Bt                   0x02
#define Down_Bt                 0x04
#define Expresso1_Bt            0x08
#define Expresso2_Bt            0x20
#define Special1_Bt              0x10
#define Special2_Bt              0x40
#define Cancel_Task             0x80

#define LED_BT8                 0x100
#define LED_BT9                 0x200
#define BT9                     0x400
#define LED_BT7                 0x800
#define LED_BT6                 0x1000
#define LED_BT5                 0x2000
#define LED_BT4                 0x4000
#define BT11                    0x8000

// TCA9539 IC 2
#define LCD_back_Light          0x10
#define BT10                    0x20
#define Brake_BLDC3             0x40
#define SpeedFeedBack_BLDC3     0x80

#define Alarm_BLDC_3            0x100
#define Enable_BLDC3            0x200
#define Direction_BLDC3         0x400
#define SpeedFeedBack_BLDC2     0x800
#define Direction_BLDC2         0x1000
#define Enable_BLDC2            0x2000
#define Valve_5                 0x4000
#define Alarm_BLDC_2            0x8000

// TCA9539 IC 3
#define Valve_1                 0x800 // Valve 10x800
#define Alarm_BLDC_1            0x20
#define Valve_4                 0x40
#define Enable_BLDC1            0x80

#define Direction_BLDC1         0x100
#define Valve_3                 0x200 // Valve_3
#define SpeedFeedBack_BLDC1     0x400
#define Valve_2                 0x10 // Valve_2
#define LinitSwitch             0x1000
#define OutletSensor            0x2000
#define WarningPressMotor       0x4000
#define Input_4                 0x8000

#endif /* TCA9539_HW_MEMMAP_H_ */
