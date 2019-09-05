/*
Example for 9DOF Click

    Date          : Aug 2018.
    Author        : Nenad Filipovic

Test configuration FT90x :
    
    MCU                : FT900
    Dev. Board         : EasyFT90x v7 
    FT90x Compiler ver : v2.3.0.0

---

Description :

The application is composed of three sections :

- System Initialization - Initializes I2C, set INT pin as input.
- Application Initialization - Initialization driver enable's - I2C, initialize LSM9DS1 and start write log.
- Application Task - (code snippet) This is a example which demonstrates the use of 9DOF Click board.
     Measured accel, gyro and magnetometar coordinates values (X,Y,Z) and temperature value in degrees C are being sent to the uart where you can track their changes.
     All data logs on usb uart for aproximetly every 1 sec.

Additional Functions :

- UART
- Conversions

*/

#include "Click_9DOF_types.h"
#include "Click_9DOF_config.h"


int16_t accelX;
int16_t accelY;
int16_t accelZ;
int16_t gyroX;
int16_t gyroY;
int16_t gyroZ;
int16_t magX;
int16_t magY;
int16_t magZ;
float temperature;
uint8_t temp[2]    = {0};
char logText[ 15 ] = {0};

void systemInit()
{
    mikrobus_gpioInit( _MIKROBUS1, _MIKROBUS_INT_PIN, _GPIO_INPUT );
    mikrobus_gpioInit( _MIKROBUS1, _MIKROBUS_RST_PIN, _GPIO_OUTPUT );
    mikrobus_i2cInit( _MIKROBUS1, &_C9DOF_I2C_CFG[0] );
    mikrobus_logInit( _MIKROBUS2, 9600 );
    Delay_100ms();
}

void applicationInit()
{
    c9dof_i2cDriverInit( (T_C9DOF_P)&_MIKROBUS1_GPIO, (T_C9DOF_P)&_MIKROBUS1_I2C, _C9DOF_ADDRESS_00 );

    /*  Initializes  */
    //ACCEL & GYRO
    c9dof_writeData( _C9DOF_CTRL_REG4, _C9DOF_CTRL_REG4_CONFIG );
    Delay_10ms();
    c9dof_writeData( _C9DOF_CTRL_REG1_G, _C9DOF_CTRL_REG1_G_CONFIG );
    Delay_10ms();
    c9dof_writeData( _C9DOF_CTRL_REG5_XL, _C9DOF_CTRL_REG5_XL_CONFIG );
    Delay_10ms();
    c9dof_writeData( _C9DOF_CTRL_REG6_XL, _C9DOF_CTRL_REG6_XL_CONFIG );
    Delay_10ms();
    c9dof_writeData( _C9DOF_CTRL_REG8, _C9DOF_CTRL_REG8_CONFIG );
    Delay_10ms();
    //MAGNETOMETAR
    c9dof_writeDataMag( _C9DOF_CTRL_REG1_M, _C9DOF_CTRL_REG1_M_CONFIG );
    Delay_10ms();
    c9dof_writeDataMag( _C9DOF_CTRL_REG2_M, _C9DOF_CTRL_REG2_M_CONFIG );
    Delay_10ms();
    c9dof_writeDataMag( _C9DOF_CTRL_REG3_M, _C9DOF_CTRL_REG3_M_CONFIG );
    Delay_10ms();
    c9dof_writeDataMag( _C9DOF_CTRL_REG4_M, _C9DOF_CTRL_REG4_M_CONFIG );
    Delay_10ms();
    c9dof_writeDataMag( _C9DOF_CTRL_REG5_M, _C9DOF_CTRL_REG5_M_CONFIG );
    Delay_10ms();
    
    mikrobus_logWrite("----------------------------------------------------------------------------", _LOG_LINE);
    mikrobus_logWrite("|     Accel       |       Gyro        |       Mag        |       Temp. °C  |",_LOG_LINE);
    mikrobus_logWrite("----------------------------------------------------------------------------", _LOG_LINE);
}

void applicationTask()
{
    c9dof_readAccel( &accelX, &accelY, &accelZ );
    Delay_10ms();
    c9dof_readGyro(  &gyroX,  &gyroY, &gyroZ );
    Delay_10ms();
    c9dof_readMag(  &magX,  &magY, &magZ );
    Delay_10ms();
    temperature = c9dof_readTemperature();
    Delay_10ms();

    mikrobus_logWrite( " Accel X :", _LOG_TEXT );
    IntToStr( accelX, logText );
    mikrobus_logWrite( logText, _LOG_TEXT );
    mikrobus_logWrite( "  |  ", _LOG_TEXT );
    mikrobus_logWrite( " Gyro X :", _LOG_TEXT );
    IntToStr( gyroX, logText );
    mikrobus_logWrite( logText, _LOG_TEXT );
    mikrobus_logWrite( "  |  ", _LOG_TEXT );
    mikrobus_logWrite( " Mag X :", _LOG_TEXT );
    IntToStr( magX, logText );
    mikrobus_logWrite( logText, _LOG_TEXT );
    mikrobus_logWrite( "  *", _LOG_TEXT );
    mikrobus_logWrite( "*****************", _LOG_LINE );

    mikrobus_logWrite( " Accel Y :", _LOG_TEXT );
    IntToStr( accelY, logText );
    mikrobus_logWrite( logText, _LOG_TEXT );
    mikrobus_logWrite( "  |  ", _LOG_TEXT );
    mikrobus_logWrite( " Gyro Y :", _LOG_TEXT );
    IntToStr( gyroY, logText );
    mikrobus_logWrite( logText, _LOG_TEXT );
    mikrobus_logWrite( "  |  ", _LOG_TEXT );
    mikrobus_logWrite( " Mag Y :", _LOG_TEXT );
    IntToStr( magY, logText );
    mikrobus_logWrite( logText, _LOG_TEXT );
    mikrobus_logWrite( "  *  ", _LOG_TEXT );
    mikrobus_logWrite( "Temp.:", _LOG_TEXT );
    IntToStr( temperature, logText );
    mikrobus_logWrite( logText, _LOG_TEXT );
    mikrobus_logWrite( "° *  ", _LOG_LINE );

    mikrobus_logWrite( " Accel Z :", _LOG_TEXT );
    IntToStr( accelZ, logText );
    mikrobus_logWrite( logText, _LOG_TEXT );
    mikrobus_logWrite( "  |  ", _LOG_TEXT );
    mikrobus_logWrite( " Gyro Z :", _LOG_TEXT );
    IntToStr( gyroZ, logText );
    mikrobus_logWrite( logText, _LOG_TEXT );
    mikrobus_logWrite( "  |  ", _LOG_TEXT );
    mikrobus_logWrite( " Mag Z :", _LOG_TEXT );
    IntToStr( magZ, logText );
    mikrobus_logWrite( logText, _LOG_TEXT );
    mikrobus_logWrite( "  *", _LOG_TEXT );
    mikrobus_logWrite( "*****************", _LOG_LINE );

    mikrobus_logWrite("----------------------------------------------------------------------------", _LOG_LINE);

    Delay_1sec();
}

void main()
{
    systemInit();
    applicationInit();

    while (1)
    {
            applicationTask();
    }
}