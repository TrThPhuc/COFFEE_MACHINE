/*
 * TCA9539.c
 *
 *  Created on: Jun 19, 2021
 *      Author: 16126
 */
#include "TCA9539.h"

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/i2c.h"
#include "inc/hw_i2c.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/pwm.h"

#include "stdbool.h"
#define error   1
#define Nerror  0
unsigned char *I2C_Tx_ptr;
unsigned char *I2C_Rx_ptr;
volatile int PtrTransmit;
_Bool startRepeat_FollowedReceive;
#define Write   0
#define Read    1
unsigned char ex_count = 0;
unsigned char Slave_Address_ptr;
void I2C0_TCA9539_Configuration(void);
extern void Read_INT_Handler(void);
uint32_t g_ui32DataRx;

/*******************************************************************************
 *    USI interrupt service routine used for I2C State Machine                   *
 *******************************************************************************/
/*
 Transmit Register in Device

 Start       7 bit Address         W   ACK         Device Address                Register Address
 _    _______________________    _   _    ___________________________     ___________________________
 _   _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ |
 SCL  |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_|

 Followed by either Transmit of Data to the Register:

 Device High Byte                Device Low Byte
 ___________________________     ___________________________
 _   _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _
 SCL  |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_|

 or Receive Data from the Register:

 Re-Start       7 bit Address         R   ACK      Device High Byte                Device Low Byte
 _    _______________________    _   _    ___________________________     ___________________________
 _   _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ | _ |
 SCL  |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_|

 */

/*
 *  USCI_B0 I2C Transmit ISR. Enable USCI_B0 I2C transmit interrupt and set USCIAB0TX_ISR as the interrupt handler.
 *  Then set the transmit interrupt to Manual on interrupt return.
 */
unsigned char I2C_Write_Buffer(unsigned char Slave_Add, unsigned char Res_Add,
                               unsigned char *data, unsigned char count)
{

    //  I2CMasterIntEnable(I2C0_BASE);
    // Wait for i2c bus idle
    while (I2CMasterBusBusy(I2C0_BASE))
        ; // Just use for multy master
    I2CMasterSlaveAddrSet(I2C0_BASE, Slave_Add, Write);
    I2CMasterDataPut(I2C0_BASE, Res_Add);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START); // Start + slave add(w)+ Ack + res_add_

#ifdef Polling_method
    unsigned char index = 0;
    do
    {
        while (I2CMasterBusy(I2C0_BASE))
            ;
        uint32_t errorbit = I2CMasterErr(I2C0_BASE);
        if (errorbit)
        {

            if (errorbit == I2C_MASTER_ERR_ARB_LOST)
                I2CMasterControl(I2C0_BASE,
                I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
            return error;
        }

        I2CMasterDataPut(I2C0_BASE, data[index]);
        index++;
        if (index < count)
        {
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT); // Ack_slave + data 1
            continue;
        }
        else
            break;

    }
    while (1);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH); // Ack_slave + data2 + stop
    while (I2CMasterBusy(I2C0_BASE))
        ;
    if (I2CMasterErr(I2C0_BASE))
        return error;
    return Nerror;
#endif
#ifdef interrupt_method
    startRepeat_FollowedReceive = PtrTransmit = 0;
    I2C_Tx_ptr = data;
    ex_count = count;

    return Nerror;
#endif

}
unsigned char I2C_Read_Buffer(unsigned char Slave_Add, unsigned char Res_Add,
                              unsigned char *data, unsigned char count)
{

// wait for i2c bus idle
    //  I2CMasterIntEnable(I2C0_BASE);
    while (I2CMasterBusBusy(I2C0_BASE))
        ;
    I2CMasterSlaveAddrSet(I2C0_BASE, Slave_Add, Write);     // write cmd
    I2CMasterDataPut(I2C0_BASE, Res_Add);
//  Start + slave add(w) +  (ack_slave) + res_add_config
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

#ifdef Polling_method
    unsigned char index = 0;
    // Wait controller modun busy
    while (I2CMasterBusy(I2C0_BASE))
        ;
    if (I2CMasterErr(I2C0_BASE))
        return error;
    I2CMasterSlaveAddrSet(I2C0_BASE, Slave_Add, Read);      // read cmd
    //Start + slave add(r) + (ack_slave) + D0 + (ack_master)
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    do
    {
        while (I2CMasterBusy(I2C0_BASE) != 0)
            ;
        uint32_t errorBit = I2CMasterErr(I2C0_BASE);
        if (errorBit)
        {
            if (errorBit == I2C_MASTER_ERR_ARB_LOST)
                I2CMasterControl(I2C0_BASE,
                I2C_MASTER_CMD_BURST_RECEIVE_ERROR_STOP);
            return error;
        }
        data[index] = I2CMasterDataGet(I2C0_BASE);      // read data 0

        index++;
        if (index < count - 1)
        {
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT); //D+1 + ACK_master
            continue;
        }
        else
            break;
    }
    while (1);

    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH); // D+1 + stop
    while (I2CMasterBusy(I2C0_BASE))
        ;
    if (I2CMasterErr(I2C0_BASE))
        return error;
    data[index] = I2CMasterDataGet(I2C0_BASE); // read data 1
    return Nerror;
#endif
#ifdef interrupt_method

    startRepeat_FollowedReceive = PtrTransmit = 1;
    Slave_Address_ptr = Slave_Add;
    I2C_Rx_ptr = data;
    ex_count = count;
    return Nerror;
#endif
}
void TCA9539Init(TCA9539Regs *Regs)
{
    I2C0_TCA9539_Configuration();
    Regs->TCA9539_Config.all = 0xFFFF;          // Configurate all is input
    Regs->TCA9539_PolInv.all = 0;               // Not inverve polarity
    Regs->TCA9539_Onput.all = 0xFFFF;
    I2C_Write_Buffer(TCA9539_ADDRESS, TCA9539_CONFIG_PORT,
                     (unsigned char*) &Regs->TCA9539_Config, 2);
    I2C_Write_Buffer(TCA9539_ADDRESS, TCA9539_OUTPUT_PORT,
                     (unsigned char*) &Regs->TCA9539_Config, 2);
    I2C_Write_Buffer(TCA9539_ADDRESS, TCA9539_POL_INVERSE_PORT,
                     (unsigned char*) &Regs->TCA9539_Config, 2);

}
void TCA9539ReadInputReg(TCA9539Regs *Regs)
{

    I2C_Read_Buffer(Regs->_Id, TCA9539_INPUT_PORT,
                    (unsigned char*) &Regs->TCA9539_Input, 2);
}
void TCA9539WriteConfig(TCA9539Regs *Regs)
{
    I2C_Write_Buffer(Regs->_Id, TCA9539_CONFIG_PORT,
                     (unsigned char*) &Regs->TCA9539_Config, 2);

}
void TCA9539WriteOutput(TCA9539Regs *Regs)
{
    Regs->updateOutputFlag = false;
    I2C_Write_Buffer(Regs->_Id, TCA9539_OUTPUT_PORT,
                     (unsigned char*) &Regs->TCA9539_Onput, 2);

}
void TCA9539WritePolarity(TCA9539Regs *Regs)
{
    I2C_Write_Buffer(Regs->_Id, TCA9539_POL_INVERSE_PORT,
                     (unsigned char*) &Regs->TCA9539_PolInv, 2);
}
void I2C0_TCA9539_Configuration(void)
{
#ifndef I2C_CLOCK
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
#endif
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true); // rate 400Khz

    // I2CMasterIntEnable(I2C0_BASE);
    //I2CMasterIntEnableEx(I2C0_BASE, I2C_MASTER_INT_DATA); // TM4C129

    // Enable loop back for test
    I2CLoopbackEnable(I2C0_BASE);
    //IntEnable(I2C0_BASE);

    I2CSlaveIntEnableEx(I2C0_BASE, I2C_SLAVE_INT_DATA);
    I2CSlaveEnable(I2C0_BASE);
    I2CSlaveInit(I2C0_BASE, TCA9539_ADDRESS);

    // Asign intrrupt handler
    I2CIntRegister(I2C0_BASE, I2C_Interrupt_Handler);

    IntMasterEnable();

}
void I2C0_TCA9539_IterruptTrigger_Cnf()
{
    // GPIO interrupt PB1
    GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA,
    GPIO_PIN_TYPE_STD_WPU);
    GPIOIntDisable(GPIO_PORTB_BASE, GPIO_INT_PIN_1);

    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_FALLING_EDGE);
    GPIOIntRegister(GPIO_PORTB_BASE, &Read_INT_Handler);

    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_1);

}

//-------------------------Interrupt method---------------------------------*/
void I2C_Interrupt_Handler()
{

//----------- Slave interrupt-------------------//
    if (I2CSlaveIntStatus(I2C0_BASE, true))
    {
        I2CSlaveIntClear(I2C0_BASE);
        uint8_t status = I2CSlaveStatus(I2C0_BASE) & 0x03;
        switch (status)
        {
        case I2C_SLAVE_ACT_RREQ:

            g_ui32DataRx = I2CSlaveDataGet(I2C0_BASE);
            break;
        case I2C_SLAVE_ACT_TREQ:

            I2CSlaveDataPut(I2C0_BASE, 0xAA);
            break;
        };
    }
//----------- Master interrupt-------------------//
    if (I2CMasterIntStatus(I2C0_BASE, true))
    {
        I2CMasterIntClear(I2C0_BASE);
        uint32_t errorBit = I2CMasterErr(I2C0_BASE);
        if (errorBit)
        {
            if ((errorBit & I2C_MASTER_ERR_ARB_LOST) != 0)
                I2CMasterControl(I2C0_BASE,
                I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
            //I2CMasterIntDisable(I2C0_BASE);
            I2CMasterIntDisable(I2C0_BASE);
            return 1;
        }
        if ((HWREG(I2C0_BASE + I2C_O_MSA) & I2C_MSA_RS) == 0) // transmit operation
        {
            if (startRepeat_FollowedReceive)
            {
                I2CMasterSlaveAddrSet(I2C0_BASE, Slave_Address_ptr, Read);
                I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
                startRepeat_FollowedReceive = PtrTransmit = 0;
                return 0;
            }

            I2CMasterDataPut(I2C0_BASE, I2C_Tx_ptr[PtrTransmit]);

            PtrTransmit++;
            if (PtrTransmit < ex_count)
                I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
            else
            {
                //I2CMasterIntDisable(I2C0_BASE);
                I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
            }
        }
        else     // Recieve operation
        {
            uint8_t temp = I2CMasterDataGet(I2C0_BASE);
            I2C_Rx_ptr[PtrTransmit] = temp;
            PtrTransmit++;
            if (PtrTransmit < ex_count - 1)
            {
                I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT); //D+1 + ACK_master

            }
            else
            {
                //I2CMasterIntDisable(I2C0_BASE);
                I2CMasterControl(I2C0_BASE,
                I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
            }
        }
    }

    return 0;
}

