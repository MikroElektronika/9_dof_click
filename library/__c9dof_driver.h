/*
    __c9dof_driver.h

-----------------------------------------------------------------------------

  This file is part of mikroSDK.
  
  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

/**
@file   __c9dof_driver.h
@brief    9DOF Driver
@mainpage 9DOF Click
@{

@image html libstock_fb_view.jpg

@}

@defgroup   C9DOF
@brief      9DOF Click Driver
@{

| Global Library Prefix | **C9DOF** |
|:---------------------:|:-----------------:|
| Version               | **1.0.0**    |
| Date                  | **Aug 2018.**      |
| Developer             | **Nenad Filipovic**     |

*/
/* -------------------------------------------------------------------------- */

#include "stdint.h"

#ifndef _C9DOF_H_
#define _C9DOF_H_

/** 
 * @macro T_C9DOF_P
 * @brief Driver Abstract type 
 */
#define T_C9DOF_P    const uint8_t*

/** @defgroup C9DOF_COMPILE Compilation Config */              /** @{ */

//  #define   __C9DOF_DRV_SPI__                            /**<     @macro __C9DOF_DRV_SPI__  @brief SPI driver selector */
   #define   __C9DOF_DRV_I2C__                            /**<     @macro __C9DOF_DRV_I2C__  @brief I2C driver selector */                                          
// #define   __C9DOF_DRV_UART__                           /**<     @macro __C9DOF_DRV_UART__ @brief UART driver selector */ 

                                                                       /** @} */
/** @defgroup C9DOF_VAR Variables */                           /** @{ */


// J1 = 0 & J2 = 0
extern const uint8_t _C9DOF_ADDRESS_00;   //  Device address
extern const uint8_t _C9DOF_ADDRESS_01;   //  Address of magnetometer

// J1 = 1 & J2 = 1
extern const uint8_t _C9DOF_ADDRESS_10;   //  Device address
extern const uint8_t _C9DOF_ADDRESS_11;   //  Address of magnetometer

// ACCEL & GYRO REGISTER
extern const uint8_t _C9DOF_ACT_THS;
extern const uint8_t _C9DOF_ACT_DUR;
extern const uint8_t _C9DOF_INT_GEN_CFG_XL;
extern const uint8_t _C9DOF_INT_GEN_THS_X_XL;
extern const uint8_t _C9DOF_INT_GEN_THS_Y_XL;
extern const uint8_t _C9DOF_INT_GEN_THS_Z_XL;
extern const uint8_t _C9DOF_INT_GEN_DUR_XL;
extern const uint8_t _C9DOF_REFERENCE_G;
extern const uint8_t _C9DOF_INT1_CTRL;
extern const uint8_t _C9DOF_INT2_CTRL;
extern const uint8_t _C9DOF_WHO_AM_I;
extern const uint8_t _C9DOF_CTRL_REG1_G;
extern const uint8_t _C9DOF_CTRL_REG2_G;
extern const uint8_t _C9DOF_CTRL_REG3_G;
extern const uint8_t _C9DOF_ORIENT_CFG_G;
extern const uint8_t _C9DOF_INT_GEN_SRC_G;
extern const uint8_t _C9DOF_OUT_TEMP_L;
extern const uint8_t _C9DOF_OUT_TEMP_H;
extern const uint8_t _C9DOF_STATUS_REG;
extern const uint8_t _C9DOF_OUT_X_L_G;
extern const uint8_t _C9DOF_OUT_X_H_G;
extern const uint8_t _C9DOF_OUT_Y_L_G;
extern const uint8_t _C9DOF_OUT_Y_H_G;
extern const uint8_t _C9DOF_OUT_Z_L_G;
extern const uint8_t _C9DOF_OUT_Z_H_G;
extern const uint8_t _C9DOF_CTRL_REG4;
extern const uint8_t _C9DOF_CTRL_REG5_XL;
extern const uint8_t _C9DOF_CTRL_REG6_XL;
extern const uint8_t _C9DOF_CTRL_REG7_XL;
extern const uint8_t _C9DOF_CTRL_REG8;
extern const uint8_t _C9DOF_CTRL_REG9;
extern const uint8_t _C9DOF_CTRL_REG10;
extern const uint8_t _C9DOF_INT_GEN_SRC_XL;
extern const uint8_t _C9DOF_OUT_X_L_XL;
extern const uint8_t _C9DOF_OUT_X_H_XL;
extern const uint8_t _C9DOF_OUT_Y_L_XL;
extern const uint8_t _C9DOF_OUT_Y_H_XL;
extern const uint8_t _C9DOF_OUT_Z_L_XL;
extern const uint8_t _C9DOF_OUT_Z_H_XL;
extern const uint8_t _C9DOF_FIFO_CTRL;
extern const uint8_t _C9DOF_FIFO_SRC;
extern const uint8_t _C9DOF_INT_GEN_CFG_G;
extern const uint8_t _C9DOF_INT_GEN_THS_XH_G;
extern const uint8_t _C9DOF_INT_GEN_THS_XL_G;
extern const uint8_t _C9DOF_INT_GEN_THS_YH_G;
extern const uint8_t _C9DOF_INT_GEN_THS_YL_G;
extern const uint8_t _C9DOF_INT_GEN_THS_ZH_G;
extern const uint8_t _C9DOF_INT_GEN_THS_ZL_G;
extern const uint8_t _C9DOF_INT_GEN_DUR_G;

// MAGNETOMETAR REGISTER
extern const uint8_t _C9DOF_OFFSET_X_REG_L_M;
extern const uint8_t _C9DOF_OFFSET_X_REG_H_M;
extern const uint8_t _C9DOF_OFFSET_Y_REG_L_M;
extern const uint8_t _C9DOF_OFFSET_Y_REG_H_M;
extern const uint8_t _C9DOF_OFFSET_Z_REG_L_M;
extern const uint8_t _C9DOF_OFFSET_Z_REG_H_M;
extern const uint8_t _C9DOF_CTRL_REG1_M;
extern const uint8_t _C9DOF_CTRL_REG2_M;
extern const uint8_t _C9DOF_CTRL_REG3_M;
extern const uint8_t _C9DOF_CTRL_REG4_M;
extern const uint8_t _C9DOF_CTRL_REG5_M;
extern const uint8_t _C9DOF_STATUS_REG_M;
extern const uint8_t _C9DOF_OUT_X_L_M;
extern const uint8_t _C9DOF_OUT_X_H_M;
extern const uint8_t _C9DOF_OUT_Y_L_M;
extern const uint8_t _C9DOF_OUT_Y_H_M;
extern const uint8_t _C9DOF_OUT_Z_L_M;
extern const uint8_t _C9DOF_OUT_Z_H_M;
extern const uint8_t _C9DOF_INT_CFG_M;
extern const uint8_t _C9DOF_INT_SRC_M;
extern const uint8_t _C9DOF_INT_THS_L_M;
extern const uint8_t _C9DOF_INT_THS_H_M;

//WHO AM I
extern const uint8_t _C9DOF_XG_ID;
extern const uint8_t _C9DOF_MAG_ID;

// DEFINE PRESSURE & TEMPERATURE
extern const uint8_t _C9DOF_CTRL_REG4_CONFIG;
extern const uint8_t _C9DOF_CTRL_REG1_G_CONFIG;
extern const uint8_t _C9DOF_CTRL_REG5_XL_CONFIG;
extern const uint8_t _C9DOF_CTRL_REG6_XL_CONFIG;
extern const uint8_t _C9DOF_CTRL_REG8_CONFIG;

extern const uint8_t _C9DOF_CTRL_REG1_M_CONFIG;
extern const uint8_t _C9DOF_CTRL_REG2_M_CONFIG;
extern const uint8_t _C9DOF_CTRL_REG3_M_CONFIG;
extern const uint8_t _C9DOF_CTRL_REG4_M_CONFIG;
extern const uint8_t _C9DOF_CTRL_REG5_M_CONFIG;


                                                                       /** @} */
/** @defgroup C9DOF_TYPES Types */                             /** @{ */



                                                                       /** @} */
#ifdef __cplusplus
extern "C"{
#endif

/** @defgroup C9DOF_INIT Driver Initialization */              /** @{ */

#ifdef   __C9DOF_DRV_SPI__
void c9dof_spiDriverInit(T_C9DOF_P gpioObj, T_C9DOF_P spiObj);
#endif
#ifdef   __C9DOF_DRV_I2C__
void c9dof_i2cDriverInit(T_C9DOF_P gpioObj, T_C9DOF_P i2cObj, uint8_t slave);
#endif
#ifdef   __C9DOF_DRV_UART__
void c9dof_uartDriverInit(T_C9DOF_P gpioObj, T_C9DOF_P uartObj);
#endif

// GPIO Only Drivers - remove in other cases
void c9dof_gpioDriverInit(T_C9DOF_P gpioObj);
                                                                       /** @} */
/** @defgroup C9DOF_FUNC Driver Functions */                   /** @{ */


/**
 * @brief Generic write data function
 *
 * @param[in] address         Register address
 *
 * @param[in] writeCommand    Command to write
 *
 * Function write byte of data to LSM9DS1XG
 */
void c9dof_writeData( uint8_t address, uint8_t writeCommand );

/**
 * @brief Generic write data function
 *
 * @param[in] address         Register address
 *
 * @param[in] writeCommand    Command to write
 *
 * Function write byte of data to LSM9DS1M
 */
void c9dof_writeDataMag( uint8_t address, uint8_t writeCommand );

/**
 * @brief Generic read data function
 *
 * @param[in] address         Register address
 *
 * @return    Data from addressed register in LSM9DS1XG
 *
 * Function read byte of data from register address of LSM9DS1XG
 */
uint8_t c9dof_readData( uint8_t address );

/**
 * @brief Generic read data function
 *
 * @param[in] address         Register address
 *
 * @return    Data from addressed register in LSM9DS1M
 *
 * Function read byte of data from register address of LSM9DS1M
 */
uint8_t c9dof_readDataMag( uint8_t address );

/**
 * @brief Function get low and high register data
 *
 * @param[in] adrRegLow         low data register address
 *
 * @param[in] adrRegHigh         high data register address
 *
 * @return         16-bit value ( low and high data )
 *
 * Function get data from two LSM9DS1XG register
 */
int16_t c9dof_getAxis( uint8_t adrRegLow );

/**
 * @brief Function get low and high register data
 *
 * @param[in] adrRegLow         low data register address
 *
 * @param[in] adrRegHigh         high data register address
 *
 * @return         16-bit value ( low and high data )
 *
 * Function get data from two LSM9DS1M register
 */
int16_t c9dof_getAxisMag( uint8_t adrRegLow );

/**
 * @brief Function read axis
 *
 * @param[out] gyroX             pointer to read Gyro X-axis data
 * @param[out] gyroY             pointer to read Gyro Y-axis data
 * @param[out] gyroZ             pointer to read Gyro Z-axis data
 *
 * Function read Gyro X-axis, Y-axis and Z-axis axis.
 *
 */
void c9dof_readGyro( int16_t *gyroX, int16_t *gyroY, int16_t *gyroZ );

/**
 * @brief Function read axis
 *
 * @param[out] accelX             pointer to read Accel X-axis data
 * @param[out] accelY             pointer to read Accel Y-axis data
 * @param[out] accelZ             pointer to read Accel Z-axis data
 *
 * Function read Accel X-axis, Y-axis and Z-axis axis.
 *
 */
void c9dof_readAccel( int16_t *accelX, int16_t *accelY, int16_t *accelZ );

/**
 * @brief Function read axis
 *
 * @param[out] magX             pointer to read Accel X-axis data
 * @param[out] magY             pointer to read Accel Y-axis data
 * @param[out] magZ             pointer to read Accel Z-axis data
 *
 * Function read Magnetometar X-axis, Y-axis and Z-axis axis.
 *
 */
void c9dof_readMag( int16_t *magX, int16_t *magY, int16_t *magZ );

/**
 * @brief Function read temperature data in degrees [ °C ]
 *
 * @return         temperature in degrees [ °C ]
 *
 * Function read temperature data
 */
float c9dof_readTemperature();




                                                                       /** @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif

/**
    @example Click_9DOF_STM.c
    @example Click_9DOF_TIVA.c
    @example Click_9DOF_CEC.c
    @example Click_9DOF_KINETIS.c
    @example Click_9DOF_MSP.c
    @example Click_9DOF_PIC.c
    @example Click_9DOF_PIC32.c
    @example Click_9DOF_DSPIC.c
    @example Click_9DOF_AVR.c
    @example Click_9DOF_FT90x.c
    @example Click_9DOF_STM.mbas
    @example Click_9DOF_TIVA.mbas
    @example Click_9DOF_CEC.mbas
    @example Click_9DOF_KINETIS.mbas
    @example Click_9DOF_MSP.mbas
    @example Click_9DOF_PIC.mbas
    @example Click_9DOF_PIC32.mbas
    @example Click_9DOF_DSPIC.mbas
    @example Click_9DOF_AVR.mbas
    @example Click_9DOF_FT90x.mbas
    @example Click_9DOF_STM.mpas
    @example Click_9DOF_TIVA.mpas
    @example Click_9DOF_CEC.mpas
    @example Click_9DOF_KINETIS.mpas
    @example Click_9DOF_MSP.mpas
    @example Click_9DOF_PIC.mpas
    @example Click_9DOF_PIC32.mpas
    @example Click_9DOF_DSPIC.mpas
    @example Click_9DOF_AVR.mpas
    @example Click_9DOF_FT90x.mpas
*/                                                                     /** @} */
/* -------------------------------------------------------------------------- */
/*
  __c9dof_driver.h

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