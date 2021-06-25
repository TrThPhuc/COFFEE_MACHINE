/*
 * ADS11118.h
 *
 *  Created on: Jun 15, 2021
 *      Author: 16126
 */

#ifndef ADS1118_H_
#define ADS1118_H_
#include "stdint.h"
#include "stdbool.h"
extern volatile unsigned int flag;     //global flag.
typedef struct ADS1118
{
    uint16_t Code;
    bool swBit;
    int32_t cold_data, hot_data;
    int16_t Actual_temperature;
} ADS1118_t;

#define ADS1118_OS             (0x8000)         //      single shot
#define ADS1118_CH0            (0x0000)         //      Differential AN0 - AN1
#define ADS1118_CH1            (0x3000)         //      Differential AN2 - AN3
#define ADS1118_GAIN0          (0x0000)         //      FSR is ±6.144 V
#define ADS1118_GAIN1          (0x0200)         //      FSR is ±4.096 V
#define ADS1118_GAIN2          (0x0400)         //      FSR is ±2.048 V
#define ADS1118_GAIN4          (0x0600)         //      FSR is ±1.024 V
#define ADS1118_GAIN8          (0x0800)         //      FSR is ±0.512 V
#define ADS1118_GAIN16         (0x0A00)         //      FSR is ±0.256 V
#define ADS1118_PWRDOWN        (0x0100)         //

#define ADS1118_RATE8SPS       (0x0000)         //
#define ADS1118_RATE16SPS      (0x0020)         //
#define ADS1118_RATE32SPS      (0x0040)         //
#define ADS1118_RATE64SPS      (0x0060)         //
#define ADS1118_RATE128SPS     (0x0080)         //
#define ADS1118_RATE250SPS     (0x00A0)         //
#define ADS1118_RATE475SPS     (0x00C0)         //
#define ADS1118_RATE860SPS     (0x00E0)         //

#define ADS1118_TS             (0x0010)         //      Internal temperature sensor set
#define ADS1118_PULLUP         (0x0008)         //      PULLUP DOUT/~DRDY
#define ADS1118_NOP            (0x0002)         //      Valid data update Config register
#define ADS1118_CNVRDY         (0x0001)         //

//Set the configuration to AIN0/AIN1, FS=+/-0.256, SS, DR=128sps, PULLUP on DOUT
#define ADSCON_CH0      (0x8B8A)
//Set the configuration to AIN2/AIN3, FS=+/-0.256, SS, DR=128sps, PULLUP on DOUT
#define ADSCON_CH1      (0xBB8A)

void ADS_Config(unsigned int mode);
void WriteSPI(uint16_t config, int16_t mode);
void ADS_Read(uint16_t mode, int32_t *_result, uint16_t code);

int ADC_code2temp(int16_t code); // transform ADC code for far-end to temperature.
int local_compensation(int16_t local_code); // transform from local sensor code to thermocouple's compensation code.

#endif /* ADS1118_H_ */
