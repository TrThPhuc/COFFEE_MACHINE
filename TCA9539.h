/*
 * TCA9539.h
 *
 *  Created on: Jun 19, 2021
 *      Author: 16126
 */

#ifndef TCA9539_H_
#define TCA9539_H_

#include "stdbool.h"
#define interrupt_method
//#define Polling_method
/************************** I2C Address ***************************************/
#define TCA9539_ADDRESS     0x76        // I2C Address 1101 A1A2 + R/W
// A1  = 1, A2 = 0
/************************** I2C Registers *************************************/
#define TCA9539_INPUT_PORT                      0X00
#define TCA9539_INPUT_PORT0_CMD                 0X00
#define TCA9539_INPUT_PORT1_CMD                 0X01

#define TCA9539_OUTPUT_PORT                     0x02
#define TCA9539_OUTPUT_PORT0_CMD                0X02
#define TCA9539_OUTPUT_PORT1_CMD                0X03

#define TCA9539_POL_INVERSE_PORT                0x04
#define TCA9539_POL_INVERSE_PORT0_CMD           0X04
#define TCA9539_POL_INVERSE_PORT1_CMD           0X05

#define TCA9539_CONFIG_PORT                     0x06
#define TCA9539_CONFIG_PORT0_CMD                0X06
#define TCA9539_CONFIG_PORT1_CMD                0X07

struct TCA9539_BitField
{
    unsigned char B0 :1;
    unsigned char B1 :1;
    unsigned char B2 :1;
    unsigned char B3 :1;
    unsigned char B4 :1;
    unsigned char B5 :1;
    unsigned char B6 :1;
    unsigned char B7 :1;

};
//****************Input Port****************
union TCA9539_Input
{
    unsigned char all;
    struct TCA9539_BitField bit;

};
struct TCA9539_Input_Port
{
    union TCA9539_Input InputPort0; // 1 Byte(8bit)
    union TCA9539_Input InputPort1; // 1 Byte(8bit)

};
union TCA9539_Input_Port_Regs
{
    unsigned short all;           // 2 Byte
    struct TCA9539_Input_Port port;

};
//****************Output Port****************
union TCA9539_Output
{
    unsigned char all;
    struct TCA9539_BitField bit;
};
struct TCA9539_Output_Port
{
    union TCA9539_Output OutputPort0;
    union TCA9539_Output OutputPort1;

};
union TCA9539_Output_Port_Regs
{
    unsigned short all;
    struct TCA9539_Output_Port port;

};
//****************Porlaity inversion Port****************
union TCA9539_PolInv
{
    unsigned char all;
    struct TCA9539_BitField bit;
};
struct TCA9539_PolInv_Port
{
    union TCA9539_PolInv PolInv_Port0;
    union TCA9539_PolInv PolInv_Port1;

};
union TCA9539_PolInv_Port_Regs
{
    unsigned short all;
    struct TCA9539_PolInv_Port port;

};
//****************Porlaity inversion Port****************
union TCA9539_Configuration
{
    unsigned char all;
    struct TCA9539_BitField bit;
};
struct TCA9539_Configuration_Port
{
    union TCA9539_Configuration Config_Port0;
    union TCA9539_Configuration Config_Port1;

};
union TCA9539_Configuration_Port_Regs
{
    unsigned short all;
    struct TCA9539_Configuration_Port port;

};
typedef struct
{
    unsigned char _Id;
    union TCA9539_Input_Port_Regs TCA9539_Input;
    union TCA9539_Output_Port_Regs TCA9539_Onput;
    union TCA9539_PolInv_Port_Regs TCA9539_PolInv;
    union TCA9539_Configuration_Port_Regs TCA9539_Config;
    bool updateOutputFlag;

} TCA9539Regs;
void TCA9539WriteConfig(TCA9539Regs *Regs);
void TCA9539WriteOutput(TCA9539Regs *Regs);
void TCA9539WritePolarity(TCA9539Regs *Regs);

void TCA9539InitDefault(TCA9539Regs *Regs);
void TCA9539Init(TCA9539Regs *Regs);
void TCA9539ReadInputReg(TCA9539Regs *Regs);

unsigned char I2C_Read_Buffer(unsigned char Slave_Add, unsigned char Res_Add,
                              unsigned char *data, unsigned char count);
unsigned char I2C_Write_Buffer(unsigned char Slave_Add, unsigned char Res_Add,
                               unsigned char *data, unsigned char count);
void I2CSlaveIntHandler(void);
unsigned char I2C_Interrupt_Handler();
#endif /* TCA9539_H_ */