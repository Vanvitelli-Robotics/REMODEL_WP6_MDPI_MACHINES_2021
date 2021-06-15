/**
  Author:
    Andrea Cirillo

  File Name:
    vl6180x_api.h

  Summary:
    This is the header file of the VL6180X library.

  Description:
    This header file provides implementations for driver APIs for VL6180X devices.
    Generation Information :
        Device             :  VL6180X
        Software Revision  :  0.1
*/
#ifndef _VL6180X_API_
#define _VL6180X_API_

#include "../mcc_generated_files/mcc.h"
#include "../mcc_generated_files/device_config.h"
#include <stdarg.h>

#include "vl6180x_registers.h"

#define VL6180X_DEBUG     0
#define I2C_TIMEOUT       50

#define I2C_DELAY_OP_US         50 
#define VL6180X_BASE_ADDR       0x29
#define VL6180X_NEW_BASE_ADDR   0x80
#define VL6180X_NEW_ADDR_OFFSET 0x02

uint8_t NumChipEnable = 0;
uint8_t RecognizedDevices = 0;
struct VL6180X_Devices_t {
    uint8_t bus_id;
    uint8_t address;
    uint8_t last_range_mm;
} VL6180X_Devices[10];

typedef void (*function_pointer_t)(void);
function_pointer_t VL6180X_MD_EnableFunction[10];
function_pointer_t VL6180X_MD_DisableFunction[10];

enum VL6180X_RangeError {
    NO_ERROR = 0,
    VCSEL_CONTINUITY_TEST,
    VCSEL_WATCHDOG_TEST,
    VCSEL_WATCHDOG,
    PLL1_LOCK,
    PLL2_LOCK,
    EARLY_CONVERGENCE_ESTIMATE,
    MAX_CONVERGENCE,
    NO_TARGET_IGNORE,
    NOT_USED1,
    NOT_USED2,
    MAX_SIGNAL_TO_NOISE_RATIO,
    RAW_RAMGING_ALGO_UNDERFLOW,
    RAW_RANGING_ALGO_OVERFLOW,
    RANGING_ALGO_UNDERFLOW,
    RANGING_ALGO_OVERFLOW   
} VL6180X_RangeError_t;

uint8_t VL6180X_IdentifyDevices(void);

inline void VL6180X_SetChipEnable(uint8_t DeviceNumber);
inline void VL6180X_Reset(uint8_t DeviceNumber, uint8_t state);

bool VL6180X_WriteRegister(uint8_t address, uint16_t reg, uint8_t data);
uint8_t VL6180X_ReadRegister(uint8_t address, uint16_t reg, bool *error);

void VL6180X_SetupRegisters(uint8_t DeviceNumber);
void VL6180X_MeasureRange(uint8_t NumDeviceToRead, uint8_t *measure, uint8_t *status);

#endif /* END OF _VL6180X_API_ */
/**
 End of File
*/