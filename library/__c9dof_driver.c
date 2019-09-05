/*
    __c9dof_driver.c

-----------------------------------------------------------------------------

  This file is part of mikroSDK.

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

#include "__c9dof_driver.h"
#include "__c9dof_hal.c"

/* ------------------------------------------------------------------- MACROS */



/* ---------------------------------------------------------------- VARIABLES */

#ifdef   __C9DOF_DRV_I2C__
static uint8_t _slaveAddress;
#endif

// J1 = 0 & J2 = 0
const uint8_t _C9DOF_ADDRESS_00          = 0x6A;   //  Device address
const uint8_t _C9DOF_ADDRESS_01          = 0x1C;   //  Address of magnetometer

// J1 = 1 & J2 = 1
const uint8_t _C9DOF_ADDRESS_10          = 0x6B;   //  Device address when
const uint8_t _C9DOF_ADDRESS_11          = 0x1E;   //  Address of magnetometer

// ACCEL & GYRO REGISTER
const uint8_t _C9DOF_ACT_THS             = 0x04;
const uint8_t _C9DOF_ACT_DUR             = 0x05;
const uint8_t _C9DOF_INT_GEN_CFG_XL      = 0x06;
const uint8_t _C9DOF_INT_GEN_THS_X_XL    = 0x07;
const uint8_t _C9DOF_INT_GEN_THS_Y_XL    = 0x08;
const uint8_t _C9DOF_INT_GEN_THS_Z_XL    = 0x09;
const uint8_t _C9DOF_INT_GEN_DUR_XL      = 0x0A;
const uint8_t _C9DOF_REFERENCE_G         = 0x0B;
const uint8_t _C9DOF_INT1_CTRL           = 0x0C;
const uint8_t _C9DOF_INT2_CTRL           = 0x0D;
const uint8_t _C9DOF_WHO_AM_I            = 0x0F;
const uint8_t _C9DOF_CTRL_REG1_G         = 0x10;
const uint8_t _C9DOF_CTRL_REG2_G         = 0x11;
const uint8_t _C9DOF_CTRL_REG3_G         = 0x12;
const uint8_t _C9DOF_ORIENT_CFG_G        = 0x13;
const uint8_t _C9DOF_INT_GEN_SRC_G       = 0x14;
const uint8_t _C9DOF_OUT_TEMP_L          = 0x15;
const uint8_t _C9DOF_OUT_TEMP_H          = 0x16;
const uint8_t _C9DOF_STATUS_REG          = 0x17;
const uint8_t _C9DOF_OUT_X_L_G           = 0x18;
const uint8_t _C9DOF_OUT_X_H_G           = 0x19;
const uint8_t _C9DOF_OUT_Y_L_G           = 0x1A;
const uint8_t _C9DOF_OUT_Y_H_G           = 0x1B;
const uint8_t _C9DOF_OUT_Z_L_G           = 0x1C;
const uint8_t _C9DOF_OUT_Z_H_G           = 0x1D;
const uint8_t _C9DOF_CTRL_REG4           = 0x1E;
const uint8_t _C9DOF_CTRL_REG5_XL        = 0x1F;
const uint8_t _C9DOF_CTRL_REG6_XL        = 0x20;
const uint8_t _C9DOF_CTRL_REG7_XL        = 0x21;
const uint8_t _C9DOF_CTRL_REG8           = 0x22;
const uint8_t _C9DOF_CTRL_REG9           = 0x23;
const uint8_t _C9DOF_CTRL_REG10          = 0x24;
const uint8_t _C9DOF_INT_GEN_SRC_XL      = 0x26;
const uint8_t _C9DOF_OUT_X_L_XL          = 0x28;
const uint8_t _C9DOF_OUT_X_H_XL          = 0x29;
const uint8_t _C9DOF_OUT_Y_L_XL          = 0x2A;
const uint8_t _C9DOF_OUT_Y_H_XL          = 0x2B;
const uint8_t _C9DOF_OUT_Z_L_XL          = 0x2C;
const uint8_t _C9DOF_OUT_Z_H_XL          = 0x2D;
const uint8_t _C9DOF_FIFO_CTRL           = 0x2E;
const uint8_t _C9DOF_FIFO_SRC            = 0x2F;
const uint8_t _C9DOF_INT_GEN_CFG_G       = 0x30;
const uint8_t _C9DOF_INT_GEN_THS_XH_G    = 0x31;
const uint8_t _C9DOF_INT_GEN_THS_XL_G    = 0x32;
const uint8_t _C9DOF_INT_GEN_THS_YH_G    = 0x33;
const uint8_t _C9DOF_INT_GEN_THS_YL_G    = 0x34;
const uint8_t _C9DOF_INT_GEN_THS_ZH_G    = 0x35;
const uint8_t _C9DOF_INT_GEN_THS_ZL_G    = 0x36;
const uint8_t _C9DOF_INT_GEN_DUR_G       = 0x37;

// MAGNETOMETAR REGISTER
const uint8_t _C9DOF_OFFSET_X_REG_L_M    = 0x05;
const uint8_t _C9DOF_OFFSET_X_REG_H_M    = 0x06;
const uint8_t _C9DOF_OFFSET_Y_REG_L_M    = 0x07;
const uint8_t _C9DOF_OFFSET_Y_REG_H_M    = 0x08;
const uint8_t _C9DOF_OFFSET_Z_REG_L_M    = 0x09;
const uint8_t _C9DOF_OFFSET_Z_REG_H_M    = 0x0A;
const uint8_t _C9DOF_CTRL_REG1_M         = 0x20;
const uint8_t _C9DOF_CTRL_REG2_M         = 0x21;
const uint8_t _C9DOF_CTRL_REG3_M         = 0x22;
const uint8_t _C9DOF_CTRL_REG4_M         = 0x23;
const uint8_t _C9DOF_CTRL_REG5_M         = 0x24;
const uint8_t _C9DOF_STATUS_REG_M        = 0x27;
const uint8_t _C9DOF_OUT_X_L_M           = 0x28;
const uint8_t _C9DOF_OUT_X_H_M           = 0x29;
const uint8_t _C9DOF_OUT_Y_L_M           = 0x2A;
const uint8_t _C9DOF_OUT_Y_H_M           = 0x2B;
const uint8_t _C9DOF_OUT_Z_L_M           = 0x2C;
const uint8_t _C9DOF_OUT_Z_H_M           = 0x2D;
const uint8_t _C9DOF_INT_CFG_M           = 0x30;
const uint8_t _C9DOF_INT_SRC_M           = 0x31;
const uint8_t _C9DOF_INT_THS_L_M         = 0x32;
const uint8_t _C9DOF_INT_THS_H_M         = 0x33;

// WHO AM I DEFAULE VALUE
const uint8_t _C9DOF_XG_ID               = 0x68;
const uint8_t _C9DOF_MAG_ID              = 0x3D;

// DEFINE PRESSURE & TEMPERATURE
const uint8_t _C9DOF_CTRL_REG4_CONFIG    = 0x38;
const uint8_t _C9DOF_CTRL_REG1_G_CONFIG  = 0x82;
const uint8_t _C9DOF_CTRL_REG5_XL_CONFIG = 0x38;
const uint8_t _C9DOF_CTRL_REG6_XL_CONFIG = 0x83;
const uint8_t _C9DOF_CTRL_REG8_CONFIG    = 0x44;
const uint8_t _C9DOF_CTRL_REG1_M_CONFIG  = 0xD0;
const uint8_t _C9DOF_CTRL_REG2_M_CONFIG  = 0x00;
const uint8_t _C9DOF_CTRL_REG3_M_CONFIG  = 0x00;
const uint8_t _C9DOF_CTRL_REG4_M_CONFIG  = 0x08;
const uint8_t _C9DOF_CTRL_REG5_M_CONFIG  = 0x40;


/* -------------------------------------------- PRIVATE FUNCTION DECLARATIONS */



/* --------------------------------------------- PRIVATE FUNCTION DEFINITIONS */



/* --------------------------------------------------------- PUBLIC FUNCTIONS */

#ifdef   __C9DOF_DRV_SPI__

void c9dof_spiDriverInit(T_C9DOF_P gpioObj, T_C9DOF_P spiObj)
{
    hal_spiMap( (T_HAL_P)spiObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif
#ifdef   __C9DOF_DRV_I2C__

void c9dof_i2cDriverInit(T_C9DOF_P gpioObj, T_C9DOF_P i2cObj, uint8_t slave)
{
    _slaveAddress = slave;
    hal_i2cMap( (T_HAL_P)i2cObj );
    hal_gpioMap( (T_HAL_P)gpioObj );
    
    hal_gpio_rstSet(1);

    // ... power ON
    // ... configure CHIP
}

#endif
#ifdef   __C9DOF_DRV_UART__

void c9dof_uartDriverInit(T_C9DOF_P gpioObj, T_C9DOF_P uartObj)
{
    hal_uartMap( (T_HAL_P)uartObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif

// GPIO Only Drivers - remove in other cases
void c9dof_gpioDriverInit(T_C9DOF_P gpioObj)
{
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
}

/* ----------------------------------------------------------- IMPLEMENTATION */



/* Generic write data function LSM9DS1XG */
void c9dof_writeData( uint8_t address, uint8_t writeCommand )
{
    uint8_t buffer[2];
    buffer[0]= address;
    buffer[1]= writeCommand;

    hal_i2cStart();
    hal_i2cWrite( _slaveAddress, buffer, 2, END_MODE_STOP );
}

/* Generic write data function LSM9DS1M */
void c9dof_writeDataMag( uint8_t address, uint8_t writeCommand )
{
    uint8_t buffer[2];
    buffer[0]= address;
    buffer[1]= writeCommand;

    hal_i2cStart();
    hal_i2cWrite( _C9DOF_ADDRESS_01, buffer, 2, END_MODE_STOP );
}

/* Generic read data function LSM9DS1XG */
uint8_t c9dof_readData( uint8_t address )
{
    uint8_t writeReg[1];
    uint8_t readReg[1];

    writeReg[0] = address;

    hal_i2cStart();
    hal_i2cWrite( _slaveAddress, writeReg, 1, END_MODE_RESTART );
    Delay_10ms();
    hal_i2cRead( _slaveAddress, readReg, 1, END_MODE_STOP );
    Delay_10ms();

    return readReg[0];
}

/* Generic read data function LSM9DS1M */
uint8_t c9dof_readDataMag( uint8_t address )
{
    uint8_t writeReg[1];
    uint8_t readReg[1];

    writeReg[0] = address;

    hal_i2cStart();
    hal_i2cWrite( _C9DOF_ADDRESS_01, writeReg, 1, END_MODE_RESTART );
    Delay_10ms();
    hal_i2cRead( _C9DOF_ADDRESS_01, readReg, 1, END_MODE_STOP );
    Delay_10ms();

    return readReg[0];
}

/* Function get data from two LSM9DS1XG register */
int16_t c9dof_getAxis( uint8_t adrRegLow )
{
    uint16_t result;
    uint8_t buffer[2];

    buffer[0] = c9dof_readData( adrRegLow + 1 );
    buffer[1] = c9dof_readData( adrRegLow );

    result = buffer[0];
    result <<= 8;
    result |= buffer[1];

    return result;
}

/* Function get data from two LSM9DS1M register */
int16_t c9dof_getAxisMag( uint8_t adrRegLow )
{
    uint16_t result;
    uint8_t buffer[2];

    buffer[0] = c9dof_readDataMag( adrRegLow + 1 );
    buffer[1] = c9dof_readDataMag( adrRegLow );

    result = buffer[0];
    result <<= 8;
    result |= buffer[1];

    return result;
}

/* Function read Gyro X-axis, Y-axis and Z-axis axis */
void c9dof_readGyro( int16_t *gyroX, int16_t *gyroY, int16_t *gyroZ )
{
    *gyroX = c9dof_getAxis( _C9DOF_OUT_X_L_G );
    *gyroY = c9dof_getAxis( _C9DOF_OUT_Y_L_G );
    *gyroZ = c9dof_getAxis( _C9DOF_OUT_Z_L_G );
}

/* Function read Accel X-axis, Y-axis and Z-axis */
void c9dof_readAccel( int16_t *accelX, int16_t *accelY, int16_t *accelZ )
{
    *accelX = c9dof_getAxis( _C9DOF_OUT_X_L_XL );
    *accelY = c9dof_getAxis( _C9DOF_OUT_Y_L_XL );
    *accelZ = c9dof_getAxis( _C9DOF_OUT_Z_L_XL );
}

/* Function read Magnetometar X-axis, Y-axis and Z-axis */
void c9dof_readMag( int16_t *magX, int16_t *magY, int16_t *magZ )
{
    *magX = c9dof_getAxisMag( _C9DOF_OUT_X_L_M );
    *magY = c9dof_getAxisMag( _C9DOF_OUT_Y_L_M );
    *magZ = c9dof_getAxisMag( _C9DOF_OUT_Z_L_M );
}

/* Function read Temperature data */
float c9dof_readTemperature()
{
    uint16_t result;
    float temperature;
    temperature = 0.00;

    result = c9dof_getAxis( _C9DOF_OUT_TEMP_L );
    Delay_10ms();

    temperature =  ( float ) result;
    temperature = ( temperature / 25.00 ) + 25.00;

    return temperature;
}



/* -------------------------------------------------------------------------- */
/*
  __c9dof_driver.c

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:
   This product includes software developed by the MikroElektonika.

4. Neither the name of the MikroElektonika nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY MIKROELEKTRONIKA ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL MIKROELEKTRONIKA BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------- */