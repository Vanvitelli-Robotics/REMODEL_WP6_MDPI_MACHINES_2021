/**
  Author:
    Andrea Cirillo

  File Name:
    vl6180x_api.c

  Summary:
    This is the source file of the VL6180X library.

  Description:
    This source file provides implementations for driver APIs for VL6180X devices.
    Generation Information :
        Device             :  VL6180X
        Software Revision  :  0.1
*/

#include "../include/vl6180x_api.h"

bool VL6180X_WriteRegister(uint8_t address, uint16_t reg, uint8_t data)
{
    uint8_t reg_high, reg_low, addr, trials = 0;
    bool error = false;
    
    reg_high = (uint8_t)((reg & 0xFF00) >> 8);
    reg_low = (uint8_t)(reg & 0x00FF);
    
    /* Open I2C */
    while(!I2C1_MasterOpen());
    
    __delay_us(I2C_DELAY_OP_US);
    
    /* Send Start condition */
    I2C1_MasterStart();
    I2C1_MasterWaitForEvent();    
    I2C1_MasterClearIrq();
    
    __delay_us(I2C_DELAY_OP_US);
    
    /* Send address with write flag */
    addr = address << 1;
    I2C1_MasterSendTxData( addr );
    while(I2C1_MasterIsRxBufFull());
    while(I2C1_MasterIsNack())
    {
        trials++;
        if (trials == I2C_TIMEOUT)
        {
            error = true;
            
            /* Send Stop condition */
            I2C1_MasterStop();  

            __delay_us(I2C_DELAY_OP_US);

            /* Close I2C */
            I2C1_MasterClose();
    
            return error;
        }
        else
        {
            ;
        }
    }
    I2C1_MasterWaitForEvent();
    I2C1_MasterClearIrq();
    
    __delay_us(I2C_DELAY_OP_US);
    
    /* Send MSB of register address */
    I2C1_MasterSendTxData( reg_high );
    while(I2C1_MasterIsRxBufFull());
    while(I2C1_MasterIsNack());
    I2C1_MasterWaitForEvent();
    I2C1_MasterClearIrq();
    
    __delay_us(I2C_DELAY_OP_US);
    
    /* Send LSB of register address */
    I2C1_MasterSendTxData( reg_low );
    while(I2C1_MasterIsRxBufFull());
    while(I2C1_MasterIsNack());
    I2C1_MasterWaitForEvent();
    I2C1_MasterClearIrq();
    
    __delay_us(I2C_DELAY_OP_US);
    
    /* Send data to write */
    I2C1_MasterSendTxData( data );
    while(I2C1_MasterIsRxBufFull());
    while(I2C1_MasterIsNack());
    I2C1_MasterWaitForEvent();
    I2C1_MasterClearIrq();
    
    __delay_us(I2C_DELAY_OP_US);
    
    /* Send Stop condition */
    I2C1_MasterStop();  
    
    __delay_us(I2C_DELAY_OP_US);
    
    /* Close I2C */
    I2C1_MasterClose();
    
    __delay_us(I2C_DELAY_OP_US);
    
    return error;
}

uint8_t VL6180X_ReadRegister(uint8_t address, uint16_t reg, bool *error)
{
    uint8_t reg_high, reg_low, addr, data, trials = 0;
    *error = false;
    
    reg_high = (uint8_t)((reg & 0xFF00) >> 8);
    reg_low = (uint8_t)(reg & 0x00FF);
        
    /* Open I2C */
    while(!I2C1_MasterOpen());
    
    __delay_us(I2C_DELAY_OP_US);

    /* Phase 1 - Addressing the Register */
    /* Send Start condition */
    I2C1_MasterStart();
    I2C1_MasterWaitForEvent();    
    I2C1_MasterClearIrq();
    
    __delay_us(I2C_DELAY_OP_US);
    
    /* Send address with write flag */
    addr = address << 1;
    I2C1_MasterSendTxData( addr );
    while(I2C1_MasterIsRxBufFull());
    while(I2C1_MasterIsNack())
    {
        trials++;
        if (trials == I2C_TIMEOUT)
        {
            *error = true;
            
            /* Send Stop condition */
            I2C1_MasterStop();  

            __delay_us(I2C_DELAY_OP_US);

            /* Close I2C */
            I2C1_MasterClose();
            
            return 0;
        }
        else
        {
            ;
        }
    }
    I2C1_MasterWaitForEvent();
    I2C1_MasterClearIrq();
    
    __delay_us(I2C_DELAY_OP_US);
    
    /* Send MSB of register address */
    I2C1_MasterSendTxData( reg_high );
    while(I2C1_MasterIsRxBufFull());
    while(I2C1_MasterIsNack());
    I2C1_MasterWaitForEvent();
    I2C1_MasterClearIrq();
    
    __delay_us(I2C_DELAY_OP_US);
    
    /* Send LSB of register address */
    I2C1_MasterSendTxData( reg_low );
    while(I2C1_MasterIsRxBufFull());
    while(I2C1_MasterIsNack());
    I2C1_MasterWaitForEvent();
    I2C1_MasterClearIrq();
    
    /* Send Stop condition */
    I2C1_MasterStop();
    
    __delay_us(I2C_DELAY_OP_US);
    
    /* Phase 2 - Read Register Data */    
    /* Send Start condition */
    I2C1_MasterStart();
    I2C1_MasterWaitForEvent();
    I2C1_MasterClearIrq();
    
    __delay_us(I2C_DELAY_OP_US);
    
    /* Send Address with Read flag */
    addr = (address << 1) | 0x01;
    I2C1_MasterSendTxData( addr );
    I2C1_MasterWaitForEvent();
    I2C1_MasterClearIrq();
    
    __delay_us(I2C_DELAY_OP_US);
    
    // Receive data
    I2C1_MasterStartRx();
    while(!I2C1_MasterIsRxBufFull());
    I2C1_MasterWaitForEvent();
    I2C1_MasterClearIrq();
    data = I2C1_MasterGetRxData();
    SSP1STATbits.BF = 0;
    I2C1_MasterSendNack();
    I2C1_MasterWaitForEvent();
    I2C1_MasterClearIrq();
    
    __delay_us(I2C_DELAY_OP_US);
    
    /* Send Stop condition */
    I2C1_MasterStop();  
    
    __delay_us(I2C_DELAY_OP_US);
    
    /* Close I2C */
    I2C1_MasterClose();
    
    __delay_us(I2C_DELAY_OP_US);
    
    return data;
}

void VL6180X_SetupRegisters(uint8_t DeviceNumber)
{
    uint8_t device_address = VL6180X_Devices[DeviceNumber].address;
    bool error = false;
    
#if VL6180X_DEBUG == 1
    uint8_t VL6180X_data;
    printf("\r\nReading VL6180X Registers after settings for device: %d.\r\n", DeviceNumber);
#endif
    
    /* REGISTER_TUNING_SR03_270514_CustomerView.txt */
    VL6180X_WriteRegister(device_address, 0x0207, 0x01);
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x0207, &error);
    printf("0x0207, 0x01, - 0x0207, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x0208, 0x01);
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x0208, &error);
    printf("0x0208, 0x01, - 0x0208, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x0096, 0x00);
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x0096, &error);
    printf("0x0096, 0x00, - 0x0096, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x0097, 0xfd);
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x0097, &error);
    printf("0x0097, 0xfd, - 0x0097, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x00e3, 0x00);
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x00e3, &error);
    printf("0x00e3, 0x00, - 0x00e3, 0x%2x\r\n", VL6180X_data);
#endif
      
    VL6180X_WriteRegister(device_address, 0x00e4, 0x04);
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x00e4, &error);
    printf("0x00e4, 0x04, - 0x00e4, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x00e5, 0x02);
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x00e5, &error);
    printf("0x00e5, 0x02, - 0x00e5, 0x%2x\r\n", VL6180X_data);
#endif
        
    VL6180X_WriteRegister(device_address, 0x00e6, 0x01);
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x00e6, &error);
    printf("0x00e6, 0x01, - 0x00e6, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x00e7, 0x03);
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x00e7, &error);
    printf("0x00e7, 0x03, - 0x00e7, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x00f5, 0x02);
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x00f5, &error);
    printf("0x00f5, 0x02, - 0x00f5, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x00d9, 0x05);
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x00d9, &error);
    printf("0x00d9, 0x05, - 0x00d9, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x00db, 0xce);
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x00db, &error);
    printf("0x00db, 0xce, - 0x00db, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x00dc, 0x03);
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x00dc, &error);
    printf("0x00dc, 0x03, - 0x00dc, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x00dd, 0xf8);
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x00dd, &error);
    printf("0x00dd, 0xf8, - 0x00dd, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x009f, 0x00);
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x009f, &error);
    printf("0x009f, 0x00, - 0x009f, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x00a3, 0x3c);
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x00a3, &error);
    printf("0x00a3, 0x3c, - 0x00a3, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x00b7, 0x00);
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x00b7, &error);
    printf("0x00b7, 0x00, - 0x00b7, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x00bb, 0x3c);
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x00bb, &error);
    printf("0x00bb, 0x3c, - 0x00bb, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x00b2, 0x09);
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x00b2, &error);
    printf("0x00b2, 0x09, - 0x00b2, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x00ca, 0x09);
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x00ca, &error);
    printf("0x00ca, 0x09, - 0x00ca, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x0198, 0x01);
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x0198, &error);
    printf("0x0198, 0x01, - 0x0198, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x01b0, 0x17);
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x01b0, &error);
    printf("0x01b0, 0x17, - 0x01b0, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x01ad, 0x00);
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x01ad, &error);
    printf("0x01ad, 0x00, - 0x01ad, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x00ff, 0x05);
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x00ff, &error);
    printf("0x00ff, 0x05, - 0x00ff, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x0100, 0x05);
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x0100, &error);
    printf("0x0100, 0x05, - 0x0100, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x0199, 0x05);
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x0199, &error);
    printf("0x0199, 0x05, - 0x0199, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x01a6, 0x1b);
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x01a6, &error);
    printf("0x01a6, 0x1b, - 0x01a6, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x01ac, 0x3e);
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x01ac, &error);
    printf("0x01ac, 0x3e, - 0x01ac, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x01a7, 0x1f);
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x01a7, &error);
    printf("0x01a7, 0x1f, - 0x01a7, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x0030, 0x00);
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x0030, &error);
    printf("0x0030, 0x00, - 0x0030, 0x%2x\r\n", VL6180X_data);
#endif
    
    /* Recommended : Public registers - See data sheet for more detail */
    VL6180X_WriteRegister(device_address, 0x0011, 0x10); /* Enables polling for New Sample ready when measurement completes */
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x0011, &error);
    printf("0x0011, 0x10, - 0x0011, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x010a, 0x30); /* Set the averaging sample period (compromise between lower noise and increased execution time) */
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x010a, &error);
    printf("0x010a, 0x30, - 0x010a, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x003f, 0x46); /* Sets the light and dark gain (upper nibble). Dark gain should not be changed.*/
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x003f, &error);
    printf("0x003f, 0x46, - 0x003f, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x0031, 0xFF); /* sets the # of range measurements after which auto calibration of system is performed */
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x0031, &error);
    printf("0x0031, 0xFF, - 0x0031, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x0040, 0x63); /* Set ALS integration time to 100ms */
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x0040, &error);
    printf("0x0040, 0x63, - 0x0040, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x002e, 0x01); /* perform a single temperature calibration of the ranging sensor */
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x002e, &error);
    printf("0x002e, 0x01, - 0x002e, 0x%2x\r\n", VL6180X_data);
#endif
    
    /* Optional: Public registers - See data sheet for more detail */
    VL6180X_WriteRegister(device_address, 0x001b, 0x09); /* Set default ranging inter-measurement period to 100ms */
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x001b, &error);
    printf("0x001b, 0x09, - 0x001b, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x003e, 0x31); /* Set default ALS inter-measurement period to 500ms */
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x003e, &error);
    printf("0x003e, 0x31, - 0x003e, 0x%2x\r\n", VL6180X_data);
#endif
    
    VL6180X_WriteRegister(device_address, 0x0014, 0x24); /* Configures interrupt on New sample ready */
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(device_address, 0x0014, &error);
    printf("0x0014, 0x24, - 0x0014, 0x%2x\r\n", VL6180X_data);
#endif
    
    /* Part to Part offset settings. 
     * For RobotItaly BL6180X board #1, offset 35 mm (hex 0x23).
     */
/*    VL6180X_WriteRegister(VL6180X_BASE_ADDR, VL6180X_SYSRANGE_PART_TO_PART_RANGE_OFFSET, 0x23);
#if VL6180X_DEBUG == 1
    VL6180X_data = VL6180X_ReadRegister(VL6180X_BASE_ADDR, VL6180X_SYSRANGE_PART_TO_PART_RANGE_OFFSET, error);
    printf("0x0024, 0x23, - 0x0024, 0x%2x\r\n", VL6180X_data);
#endif
  */  
}

void VL6180X_MeasureRange(uint8_t NumDeviceToRead, uint8_t *measure, uint8_t *status)
{
    uint8_t device_address;
    bool error = false;
    uint8_t meas_ready, device_status, mm, k;

    for (k = 0; k < NumDeviceToRead; k++)
    {
        device_address = VL6180X_Devices[k].address;
        
        /* Wait for device ready. */
        do {
            device_status = VL6180X_ReadRegister(device_address, VL6180X_RESULT_RANGE_STATUS, &error);
        } while ((device_status & (1 << 0)) == 0);
        
        /* Start measurement. */
        VL6180X_WriteRegister(device_address, VL6180X_SYSRANGE_START, 0x01);
    }
    
    for (k = 0; k < NumDeviceToRead; k++)
    {
        device_address = VL6180X_Devices[k].address;
        
        /* Wait for measurement ready. */
        do {
            meas_ready = VL6180X_ReadRegister(device_address, VL6180X_RESULT_INTERRUPT_STATUS_GPIO, &error);
        } while ((meas_ready & (1 << 2)) == 0);
        
        /* Read result. */
        mm = VL6180X_ReadRegister(device_address, VL6180X_RESULT_RANGE_VAL, &error);
        measure[k] = mm;

        /* Clear interrupt flags. */
        VL6180X_WriteRegister(device_address, VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);

        /* Wait for device ready. */
        do {
            device_status = VL6180X_ReadRegister(device_address, VL6180X_RESULT_RANGE_STATUS, &error);
        } while ((device_status & (1 << 0)) == 0);
        
        /* Return error code. */
        status[k] = (device_status >> 4);
    }
}

inline void VL6180X_SetChipEnable(uint8_t DeviceNumber)
{
    VL6180X_Reset(DeviceNumber, 0);
    __delay_us(5000);
    VL6180X_Reset(DeviceNumber, 1);
    __delay_us(5000);
}

inline void VL6180X_Reset(uint8_t DeviceNumber, uint8_t state)
{
    /* RA6 for chip enable one the first VL6180X device */
    if( 1 == state )
    {
        //IO_RA6_SetHigh();
        VL6180X_MD_EnableFunction[DeviceNumber]();
    }
    else if( 0 == state )
    {
        //IO_RA6_SetLow();
        VL6180X_MD_DisableFunction[DeviceNumber]();
    }
    else
    {
        ;
    }
}

uint8_t VL6180X_IdentifyDevices(void)
{
    uint8_t model_id, k;
    uint8_t numDevices = 0;
    uint8_t new_address;
    bool error = false;
    
    for (k = 0; k < NumChipEnable; k++)
    {
        VL6180X_SetChipEnable(k); /* assert chip enable */
        
        model_id = VL6180X_ReadRegister(VL6180X_BASE_ADDR, VL6180X_IDENTIFICATION_MODEL_ID, &error);

        if (model_id == 0xB4 && error == false) /* 0xB4 = VL6180X */
        {
            numDevices++;
            new_address = VL6180X_NEW_BASE_ADDR+(numDevices-1)*VL6180X_NEW_ADDR_OFFSET;
            VL6180X_WriteRegister(VL6180X_BASE_ADDR, VL6180X_I2C_SLAVE_DEVICE_ADDRESS, new_address);
            
            VL6180X_Devices[numDevices-1].bus_id = 0;
            VL6180X_Devices[numDevices-1].address = new_address;
            VL6180X_Devices[numDevices-1].last_range_mm = 255;
        }
        else
        {
            ;
        }
    }
    return numDevices;
}


/**
 End of File
*/