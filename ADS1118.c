/*
 * ADS1118.c
 *
 *  Created on: Jun 15, 2021
 *      Author: 16126
 */

/******************************************************************************
 * function: ADS_Read(unsigned int mode)
 * introduction: read the ADC result and tart a new conversion.
 * parameters:
 * mode = 0, ADS1118 is set to convert the voltage of integrated temperature sensor(cold junction).
 * mode = 1, ADS1118 is set to convert the voltage of thermocouple.
 ******************************************************************************/
#include "ADS1118.h"
#include "stdint.h"
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
extern void ADS1118_Coms(uint16_t config, int mode);

void ADS_Read(uint16_t mode, uint16_t code)
{
    //volatile uint8_t DRDY = GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_2);

    unsigned int tmp;
    if (mode == 1) // Set the configuration to AIN0/AIN1, FS=+/-0.256, SS, DR=128sps, PULLUP on DOUT
        tmp = code;
    else
        tmp = code + ADS1118_TS; // temperature sensor mode.DR=8sps, PULLUP on DOUT
    // Write Config

    ADS1118_Coms(tmp, 1);
    // Set CS high to end transaction

}
/******************************************************************************
 * Funtion: local_compensation. Return: voltage code
 * Description:
 *  * Tranfer to voltage(code) of cold juntion
 *  * Local_data is at the first 14bits of the 16bits (left align) data register
 *  * One in 14-bit LSB equals 0.03125°C
 *
 * Volate (code) generate in reference or cold junction: Reverse lookup table
 *                                                   (Tin -T[n-1])
 *  * comp codes = Code[n-1] + (Code[n] - Code[n-1])* {---------------}
 *                                                  (T[n] - T[n-1])
 * Please note that thermocouple calculations must always be made in voltage.
 * A common error is to look for the table value for the measured voltage and
 * add the cold junction temperature.
 ******************************************************************************/
int local_compensation(int16_t local_code)
{
    float tmp, local_temp;
    int comp;
    local_code = local_code / 4;
    local_temp = (float) local_code / 32;    //

    if (local_temp >= 0 && local_temp <= 5)       //0~5
    {
        tmp = (0x0019 * local_temp) / 5;
        comp = tmp;
    }
    else if (local_temp > 5 && local_temp <= 10)  //5~10
    {
        tmp = (0x001A * (local_temp - 5)) / 5 + 0x0019;
        comp = tmp;
    }
    else if (local_temp > 10 && local_temp <= 20) //10~20
    {
        tmp = (0x0033 * (local_temp - 10)) / 10 + 0x0033;
        comp = tmp;
    }
    else if (local_temp > 20 && local_temp <= 30) //20~30
    {
        tmp = (0x0034 * (local_temp - 20)) / 10 + 0x0066;
        comp = tmp;
    }
    else if (local_temp > 30 && local_temp <= 40) //30~40
    {
        tmp = (0x0034 * (local_temp - 30)) / 10 + 0x009A;
        comp = tmp;
    }
    else if (local_temp > 40 && local_temp <= 50) //40~50
    {
        tmp = (0x0035 * (local_temp - 40)) / 10 + 0x00CE;
        comp = tmp;
    }

    else if (local_temp > 50 && local_temp <= 60) //50~60
    {
        tmp = (0x0035 * (local_temp - 50)) / 10 + 0x0103;
        comp = tmp;
    }
    else if (local_temp > 60 && local_temp <= 80) //60~80
    {
        tmp = (0x006A * (local_temp - 60)) / 20 + 0x0138;
        comp = tmp;
    }
    else if (local_temp > 80 && local_temp <= 125) //80~125
    {
        tmp = (0x00EE * (local_temp - 80)) / 45 + 0x01A2;
        comp = tmp;
    }
    else
    {
        comp = 0;
    }
    return comp;
}

/******************************************************************************
 * Funtion: ADC_code2temp(int code). Return: Temperature
 * Description:
 *  * Tranfer to temperature from code voltage
 *  * Converted temperature range is 0 to 500 Celsius degree
 *  * Type K thermocouple lookup table is used
 *  * ADC input range is +/-256mV. 16bits. so 1 LSB = 7.8125uV.
 * Temperature :
 *                                (Codes - Code[n-1])
 * T = T[n-1] + (T[n]-T[n-1]) * {---------------------}
 *                               (Code[n] - Code[n-1])
 * Please note that thermocouple calculations must always be made in voltage.
 * A common error is to look for the table value for the measured voltage and
 * add the cold junction temperature.
 ******************************************************************************/
float ADC_code2temp(int16_t code)
{
    float temp;

    temp = (float) code;

    if (code > 0x0066 && code <= 0x009A)   //20~30
    {
        temp = (float) (10 * (temp - 0x0066)) / 0x0034 + 20.0f;
    }
    else if (code > 0x009A && code <= 0x00CE)   //30~40
    {
        temp = (float) (10 * (temp - 0x009A)) / 0x0034 + 30.0f;
    }
    else if (code > 0x00CE && code <= 0x0103)  //40~50
    {
        temp = (float) (10 * (temp - 0x00CE)) / 0x0035 + 40.0f;
    }
    else if (code > 0x0103 && code <= 0x0138)  //50~60
    {
        temp = (float) (10 * (temp - 0x0103)) / 0x0035 + 50.0f;
    }
    else if (code > 0x0138 && code <= 0x01A2)    //60~80
    {
        temp = (float) (20 * (temp - 0x0138)) / 0x006A + 60.0f;
    }
    else if (code > 0x01A2 && code <= 0x020C)   //80~100
    {
        temp = (float) ((temp - 0x01A2) * 20) / 0x06A + 80.0f;
    }
    else if (code > 0x020C && code <= 0x02DE)   //100~140
    {
        temp = (float) ((temp - 0x020C) * 40) / 0x0D2 + 100.0f;
    }
    else if (code > 0x02DE && code <= 0x03AC)   //140~180
    {
        temp = (float) ((temp - 0x02DE) * 40) / 0x00CE + 140.0f;
    }
    else if (code > 0x03AC && code <= 0x0478)   //180~220
    {
        temp = (float) ((temp - 0x03AB) * 40) / 0x00CD + 180.0f;
    }
    else if (code > 0x0478 && code <= 0x0548)   //220~260
    {
        temp = (float) ((temp - 0x0478) * 40) / 0x00D0 + 220.0f;
    }
    else if (code > 0x0548 && code <= 0x061B)   //260~300
    {
        temp = (float) ((temp - 0x0548) * 40) / 0x00D3 + 260.0f;
    }
    else if (code > 0x061B && code <= 0x06F2)   //300~340
    {
        temp = (float) ((temp - 0x061B) * 40) / 0x00D7 + 300.0f;
    }
    else if (code > 0x06F2 && code <= 0x07C7)   //340~400
    {
        temp = (float) ((temp - 0x06F2) * 40) / 0x00D5 + 340.0f;
    }
    else if (code > 0x07C7 && code <= 0x089F)   //380~420
    {
        temp = (float) ((temp - 0x07C7) * 40) / 0x00D8 + 380.0f;
    }

    else if (code > 0x089F && code <= 0x0978)   //420~460
    {
        temp = (float) ((temp - 0x089F) * 40) / 0x00D9 + 420.0f;
    }
    else if (code > 0x0978 && code <= 0x0A52)    //460~500
    {
        temp = (float) ((temp - 0x0978) * 40) / 0x00DA + 460.0f;
    }
    else
    {
        temp = 0xA5A5;
    }

    //temp = (int) (10 * temp);

    return temp;

}
