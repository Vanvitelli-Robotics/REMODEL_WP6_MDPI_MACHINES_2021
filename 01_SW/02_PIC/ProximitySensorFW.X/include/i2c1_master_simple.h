/**
  I2C1 Generated Driver API Header File

  @Company
    Microchip Technology Inc.

  @File Name
    i2c1.h

  @Summary
    This is the generated header file for the I2C1 driver using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description
    This header file provides APIs for driver for I2C1.
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

#ifndef I2C1_MASTER_H
#define I2C1_MASTER_H

/**
  Section: Included Files
*/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    I2C1_NOERR, // The message was sent.
    I2C1_BUSY,  // Message was not sent, bus was busy.
    I2C1_FAIL   // Message was not sent, bus failure
               // If you are interested in the failure reason,
               // Sit on the event call-backs.
} i2c1_error_t;

typedef uint8_t i2c1_address_t;

/**
 * \brief Initialize I2C1 interface
 *
 * \return Nothing
 */
void I2C1_Initialize(void);

/**
 * \brief Open the I2C1 for communication
 *
 * \param[in] address The slave address to use in the transfer
 *
 * \return Initialization status.
 * \retval I2C1_NOERR The I2C1 open was successful
 * \retval I2C1_BUSY  The I2C1 open failed because the interface is busy
 * \retval I2C1_FAIL  The I2C1 open failed with an error
 */
i2c1_error_t I2C1_Open(i2c1_address_t address);

/**
 * \brief Close the I2C1 interface
 *
 * \return Status of close operation.
 * \retval I2C1_NOERR The I2C1 open was successful
 * \retval I2C1_BUSY  The I2C1 open failed because the interface is busy
 * \retval I2C1_FAIL  The I2C1 open failed with an error
 */
i2c1_error_t I2C1_Close(void);

/**
 * \brief I2C1 Interrupt Handler
 *        This is a pointer to the function that will be called upon I2C1 interrupt
 * \param[in] None
 *
 * \return Nothing
 */
void (*MSSP1_InterruptHandler)(void);

/* I2C1 interfaces */
inline bool I2C1_MasterOpen(void);
inline void I2C1_MasterClose(void);    
inline uint8_t I2C1_MasterGetRxData(void);
inline void I2C1_MasterSendTxData(uint8_t data);
inline void I2C1_MasterEnableRestart(void);
inline void I2C1_MasterDisableRestart(void);
inline void I2C1_MasterStartRx(void);
inline void I2C1_MasterStart(void);
inline void I2C1_MasterStop(void);
inline bool I2C1_MasterIsNack(void);
inline void I2C1_MasterSendAck(void);
inline void I2C1_MasterSendNack(void);
inline void I2C1_MasterClearBusCollision(void);
inline bool I2C1_MasterIsRxBufFull(void);

/* Interrupt interfaces */
inline void I2C1_MasterEnableIrq(void);
inline bool I2C1_MasterIsIrqEnabled(void);
inline void I2C1_MasterDisableIrq(void);
inline void I2C1_MasterClearIrq(void);
inline void I2C1_MasterSetIrq(void);
inline void I2C1_MasterWaitForEvent(void);

#endif //I2C1_MASTER_H