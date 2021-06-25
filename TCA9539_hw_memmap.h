/*
 * TCA9539_hw_memmap.h
 *
 *  Created on: Jun 23, 2021
 *      Author: 16126
 */

#ifndef TCA9539_HW_MEMMAP_H_
#define TCA9539_HW_MEMMAP_H_

#define Bt1                     0x01
#define Bt2                     0x02
#define Bt3                     0x04
#define Expresso1_Bt            0x08
#define Expresso2_Bt            0x10
#define Decatt1_Bt              0x20
#define Decatt2_Bt              0x40
#define Bt8                     0x80

#define LED_BT8                 0
#define LED_BT9                 1
#define BT9                     2
#define LED_BT7                 3
#define LED_BT6                 4
#define LED_BT5                 5
#define LED_BT4                 6
#define Bt11                    7

#define LCD_back_Light          4
#define Bt10                    5
#define Brake_BLDC3             6
#define SpeedFeedBack_BLDC3     7

#define Alarm_BLDC_3            0
#define Enable_BLDC3            1
#define Direction_BLDC3         2
#define SpeedFeedBack_BLDC2     3
#define Direction_BLDC2         4
#define Enable_BLDC2            5
#define Brake_BLDC2             6
#define Alarm_BLDC_2            7

#define HotWater                4
#define Alarm_BLDC_1            5
#define Brake_BLDC1             6
#define Enable_BLDC1            7

#define Direction_BLDC1         0
#define OutSpare                1
#define SpeedFeedBack_BLDC1     2
#define SteamValve              3
#define LinitSwitch             4
#define LevelSensor1            5
#define LevelSensor2            6
#define Input_4                 7

#endif /* TCA9539_HW_MEMMAP_H_ */
