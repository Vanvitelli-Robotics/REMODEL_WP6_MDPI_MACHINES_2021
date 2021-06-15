/**
  I2C1 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    i2c1.c

  @Summary
    This is the generated driver implementation file for the I2C1 driver using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description
    This header file provides implementations for driver APIs for I2C1.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.78
        Device            :  PIC16F19176
        Driver Version    :  1.0.0
    The generated drivers are tested against the following:
        Compiler          :  XC8 2.05 and above or later
        MPLAB             :  MPLAB X 5.20
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#include <xc.h>
#include "../include/i2c1_master_simple.h"

void I2C1_Initialize()
{
    SSP1STAT = 0x00;
    SSP1CON1 = 0x08;
    SSP1CON2 = 0x00;
    SSP1ADD  = 0x13;
    SSP1CON1bits.SSPEN = 0;
}

void I2C1_SetInterruptHandler(void (* InterruptHandler)(void))
{
    MSSP1_InterruptHandler = InterruptHandler;
}

/* I2C1 Register Level interfaces */
inline bool I2C1_MasterOpen(void)
{
    if(!SSP1CON1bits.SSPEN)
    {
        I2C1_Initialize();
        SSP1CON1bits.SSPEN = 1;
        return true;
    }
    return false;
}

inline void I2C1_MasterClose(void)
{
    //Disable I2C1
    SSP1CON1bits.SSPEN = 0;
}

inline uint8_t I2C1_MasterGetRxData(void)
{
    return SSP1BUF;
}

inline void I2C1_MasterSendTxData(uint8_t data)
{
    SSP1BUF  = data;
}

inline void I2C1_MasterEnableRestart(void)
{
    SSP1CON2bits.RSEN = 1;
}

inline void I2C1_MasterDisableRestart(void)
{
    SSP1CON2bits.RSEN = 0;
}

inline void I2C1_MasterStartRx(void)
{
    SSP1CON2bits.RCEN = 1;
}

inline void I2C1_MasterStart(void)
{
    SSP1CON2bits.SEN = 1;
}

inline void I2C1_MasterStop(void)
{
    SSP1CON2bits.PEN = 1;
}

inline bool I2C1_MasterIsNack(void)
{
    return SSP1CON2bits.ACKSTAT;
}

inline void I2C1_MasterSendAck(void)
{
    SSP1CON2bits.ACKDT = 0;
    SSP1CON2bits.ACKEN = 1;
}

inline void I2C1_MasterSendNack(void)
{
    SSP1CON2bits.ACKDT = 1;
    SSP1CON2bits.ACKEN = 1;
}

inline void I2C1_MasterClearBusCollision(void)
{
    PIR3bits.BCL1IF = 0;
}


inline bool I2C1_MasterIsRxBufFull(void)
{
    return SSP1STATbits.BF;
}

inline void I2C1_MasterEnableIrq(void)
{
    PIE3bits.SSP1IE = 1;
}

inline bool I2C1_MasterIsIrqEnabled(void)
{
    return PIE3bits.SSP1IE;
}

inline void I2C1_MasterDisableIrq(void)
{
    PIE3bits.SSP1IE = 0;
}

inline void I2C1_MasterClearIrq(void)
{
    PIR3bits.SSP1IF = 0;
}

inline void I2C1_MasterSetIrq(void)
{
    PIR3bits.SSP1IF = 1;
}

inline void I2C1_MasterWaitForEvent(void)
{
    while(1)
    {
        if(PIR3bits.SSP1IF)
        {    
            break;
        }
    }
}