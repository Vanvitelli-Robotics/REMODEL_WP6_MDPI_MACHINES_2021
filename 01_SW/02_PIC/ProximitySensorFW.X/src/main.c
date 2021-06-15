/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.78
        Device            :  PIC16F19176
        Driver Version    :  2.00
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

#include "../mcc_generated_files/mcc.h"
#include "../include/vl6180x_api.h"

#define APP_DEBUG 0

void SetA6(void)
{
  IO_RA6_SetHigh ();
}
void ResetA6(void)
{
  IO_RA6_SetLow();
}
void SetA7(void)
{
  IO_RA7_SetHigh();
}
void ResetA7(void)
{
  IO_RA7_SetLow();
}

void SetB0(void)
{
  IO_RB0_SetHigh();
}
void ResetB0(void)
{
  IO_RB0_SetLow();
}

void SetB1(void)
{
  IO_RB1_SetHigh();
}
void ResetB1(void)
{
  IO_RB1_SetLow();
}
/*
                         Main application
 */
void main(void)
{
  // initialize the device
  SYSTEM_Initialize();

  // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
  // Use the following macros to:

  // Enable the Global Interrupts
  INTERRUPT_GlobalInterruptEnable ();

  // Enable the Peripheral Interrupts
  INTERRUPT_PeripheralInterruptEnable ();

  // Disable the Global Interrupts
  //INTERRUPT_GlobalInterruptDisable();

  // Disable the Peripheral Interrupts
  //INTERRUPT_PeripheralInterruptDisable();

  /* VL6180X Initialization */
  NumChipEnable = 4U;
  VL6180X_MD_EnableFunction[0] = SetA6;
  VL6180X_MD_EnableFunction[1] = SetA7;
  VL6180X_MD_EnableFunction[2] = SetB0;
  VL6180X_MD_EnableFunction[3] = SetB1;
  VL6180X_MD_DisableFunction[0] = ResetA6;
  VL6180X_MD_DisableFunction[1] = ResetA7;
  VL6180X_MD_DisableFunction[2] = ResetB0;
  VL6180X_MD_DisableFunction[3] = ResetB1;

  RecognizedDevices = VL6180X_IdentifyDevices();
#if APP_DEBUG == 1
  printf ("RecognizedDevices: %d\r\n", RecognizedDevices);
#endif
  uint8_t k;
  for (k = 0U; k < RecognizedDevices; k++)
  {
    VL6180X_SetupRegisters(k);
  }

  /* Reset RB0 for APP timing purposes */
  //IO_RB0_SetLow();

  /* RX Buffer for USART */
  volatile uint8_t rxData = 0U;

  /* Private variables for VL6180X reading handling */
  uint8_t measure[10U] = {0U};
  uint8_t error_code[10U];

  /* Main APP loop */
  while(1)
  {
    /* Set RB0 for APP timing purposes */
    //IO_RB0_SetHigh();

    /* Check for Data presence on EUSART bus */
    if (EUSART1_is_rx_ready())
    {
      /* Read data */
      rxData = EUSART1_Read();
    }
    else
    {
      ;
    }

    /* IF 'a' is received THEN read VL6180X data */
    if (rxData == 'a')
    {
      /* VL6180X readings */
      VL6180X_MeasureRange (RecognizedDevices, measure, error_code);

      // Reset RB0 for APP timing purposes
      //IO_RB0_SetLow();

      for (uint8_t k = 0U; k < RecognizedDevices; k++)
      {
        if (NO_ERROR == error_code[k])
        {
#if APP_DEBUG == 1
          printf("Device %d: %d\r\n", k, measure[k]);
#else
          printf("%c", measure[k]);
#endif
        }
        else if (EARLY_CONVERGENCE_ESTIMATE == error_code[k])
        {
#if APP_DEBUG == 1
          printf("Device %d ERROR: object too close.\r\n", k);
#else
          printf("%c", 253);
#endif
        }
        else if (MAX_CONVERGENCE == error_code[k])
        {
#if APP_DEBUG == 1
          printf("Device %d ERROR: object too far.\r\n", k);
#else
          printf("%c", 254);
#endif
        }
        else
        {
#if APP_DEBUG == 1
          printf("Device %d ERROR: code number %d.\r\n", k, error_code[k]);
#else
          printf("%c", 255);
#endif
        }
      }
    }
    /* IF 'b' is received THEN send the Sensor Identifier over EUSART */
    else if (rxData == 'b')
    {
      while(!EUSART1_is_tx_ready())
      {
        ;
      }
      printf(_SENSOR_ID);
    }
    /* IF 'c' is received THEN send the Number of the VL6180X Sensors over EUSART */
    else if (rxData == 'c')
    {
      while(!EUSART1_is_tx_ready())
      {
        ;
      }
#if APP_DEBUG == 1
      printf("Recognized Devices: %d.\r\n", RecognizedDevices);
#else
      printf("%c", RecognizedDevices);
#endif
    }
    /* Otherwise ignore data */
    else
    {
      ;
    }

    /* Reset rxData buffer */
    rxData = 0U;
  }
}
/**
 End of File
 */