/***************************************************************************//**
 *   @file   ADXL345.c
 *   @brief  Implementation of ADXL345 Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: 684
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "ADXL345.h"
//#include "Communication.h"
#include "stm32f0xx_hal.h"

/******************************************************************************/
/************************ Variables Definitions *******************************/
/******************************************************************************/
#define ADXL345_COMMUNICATION I2C_COMMUNICATION

extern I2C_HandleTypeDef hi2c2;
/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/**************************************************************************//**
 * @brief Writes data into a register.
 *
 * @param registerAddress - Address of the register.
 * @param registerValue - Data value to write.
 *
 * @return None.
******************************************************************************/
void ADXL345_SetRegisterValue(unsigned char registerAddress,
                       unsigned char registerValue)
{
    unsigned char writeData[2] = {0, 0};
    unsigned char slaveDeviceId = 0x01;

    #if(ADXL345_COMMUNICATION == SPI_COMMUNICATION)
    {
        writeData[0] = registerAddress;
        writeData[1] = registerValue;
        //SPI_Write(slaveDeviceId, writeData, 2);
    }
    #else
    {
        //writeData[0] = registerAddress;
        //writeData[1] = registerValue;
        assert_param(HAL_OK == HAL_I2C_Mem_Write(&hi2c2, ADXL345_ADDRESS, registerAddress, I2C_MEMADD_SIZE_8BIT, &registerValue, 1, 1000)) ;
        printf("ADXL345 " "Register Writing Success, address->0x%02x, value->0x%02x\r\n", registerAddress, registerValue) ;
        //I2C_Write(ADXL345_ADDRESS, writeData, 2, 1);
    }
	#endif
}

/***************************************************************************//**
 * @brief Reads the value of a register.
 *
 * @param registerAddress - Address of the register.
 *
 * @return registerValue - Value of the register.
*******************************************************************************/
unsigned char ADXL345_GetRegisterValue(unsigned char registerAddress)
{
    unsigned char readData[2] = {0, 0};
    unsigned char writeData[2] = {0, 0};
    unsigned char registerValue = 0;
    unsigned char slaveDeviceId = 0x01;

    #if(ADXL345_COMMUNICATION == SPI_COMMUNICATION)
    {
        readData[0] = 0x80 + registerAddress;
        readData[1] = 0;
        //SPI_Read(slaveDeviceId, readData, 2);
        registerValue = readData[1];
    }
    #else
    {
        //writeData[0] = registerAddress;
        //I2C_Write(ADXL345_ADDRESS, writeData, 1, 0);
        //I2C_Read(ADXL345_ADDRESS, readData, 1, 1);
        //registerValue = readData[0];
        assert_param(HAL_OK == HAL_I2C_Mem_Read(&hi2c2, ADXL345_ADDRESS, registerAddress, I2C_MEMADD_SIZE_8BIT, &registerValue, 1, 1000)) ;
        printf("ADXL345 " "Register Reading Success, address->0x%02x, value->0x%02x\r\n", registerAddress, registerValue) ;
    }
	#endif

    return(registerValue);
}

/***************************************************************************//**
 * @brief Initializes the communication peripheral and checks if the ADXL345
 *          part is present.
 *
 * @return status - Result of the initialization procedure.
 *                  Example: 0x0 - I2C peripheral was not initialized or
 *                                 ADXL345 part is not present.
 *                           0x1 - I2C peripheral is initialized and ADXL345
 *                                 part is present.
*******************************************************************************/
unsigned char ADXL345_Init(void)
{
    unsigned char status = 1;
	
	
	uint8_t devid = 0 ;

    #if(ADXL345_COMMUNICATION == SPI_COMMUNICATION)
    {
        status = SPI_Init(0, 1000000, 1, 0);
    }
    #else
    {
        //status = I2C_Init(100000);
    }
	#endif
	
    devid = ADXL345_GetRegisterValue(ADXL345_DEVID) ;
    if(devid != ADXL345_ID)
    {
        printf("ADXL345 " "Can't get valid device id, desired->0x%02x, but now->0x%02x\r\n", ADXL345_ID, devid) ;
        status = 0;
    }
	
	
	ADXL345_SetPowerMode(0) ;
	
	ADXL345_SetDataFormat(0x0B) ;
	
	ADXL345_SetPowerMode(1) ;

    return(status);
}

/***************************************************************************//**
 * @brief Places the device into standby/measure mode.
 *
 * @param pwrMode - Power mode.
 *                    Example: 0x0 - standby mode.
 *                             0x1 - measure mode.
 *
 * @return None.
*******************************************************************************/
void ADXL345_SetPowerMode(unsigned char pwrMode)
{
    unsigned char oldPowerCtl = 0;
    unsigned char newPowerCtl = 0;
    
    oldPowerCtl = ADXL345_GetRegisterValue(ADXL345_POWER_CTL);
    newPowerCtl = oldPowerCtl & ~ADXL345_PCTL_MEASURE;
    newPowerCtl = newPowerCtl | (pwrMode * ADXL345_PCTL_MEASURE);
    ADXL345_SetRegisterValue(ADXL345_POWER_CTL, newPowerCtl);
}

/***************************************************************************//**
 * @brief Reads the output data of each axis.
 *
 * @param x - X-axis's output data.
 * @param y - Y-axis's output data.
 * @param z - Z-axis's output data.
 *
 * @return None.
*******************************************************************************/
void ADXL345_GetXyz(unsigned short* x,
                    unsigned short* y,
                    unsigned short* z)
{
    *x = ADXL345_GetRegisterValue(ADXL345_DATAX1) << 8;
    *x += ADXL345_GetRegisterValue(ADXL345_DATAX0);
    *y = ADXL345_GetRegisterValue(ADXL345_DATAY1) << 8;
    *y += ADXL345_GetRegisterValue(ADXL345_DATAY0);
    *z = ADXL345_GetRegisterValue(ADXL345_DATAZ1) << 8;
    *z += ADXL345_GetRegisterValue(ADXL345_DATAZ0);
    printf("ADXL345 " "AXIS Gravity Detected, x->%05d, y->%05d, z->%05d\r\n", *x, *y, *z) ;
}

/***************************************************************************//**
 * @brief Enables/disables the tap detection.
 *
 * @param tapType - Tap type (none, single, double).
 *                    Example: 0x0 - disables tap detection.
 *                             ADXL345_SINGLE_TAP - enables single tap detection.
 *                             ADXL345_DOUBLE_TAP - enables double tap detection.
 * @param tapAxes - Axes which participate in tap detection.
 *                    Example: 0x0 - disables axes participation.
 *                             ADXL345_TAP_X_EN - enables x-axis participation.
 *                             ADXL345_TAP_Y_EN - enables y-axis participation.
 *                             ADXL345_TAP_Z_EN - enables z-axis participation.
 * @param tapDur - Tap duration.
 * @param tapLatent - Tap latency.
 * @param tapWindow - Tap window. 
 * @param tapThresh - Tap threshold.
 * @param tapInt - Interrupts pin.
 *                   Example: 0x0 - interrupts on INT1 pin.
 *                            ADXL345_SINGLE_TAP - single tap interrupts on
 *                                                 INT2 pin.
 *                            ADXL345_DOUBLE_TAP - double tap interrupts on
 *                                                 INT2 pin.
 *
 * @return None.
*******************************************************************************/
void ADXL345_SetTapDetection(unsigned char tapType,
                             unsigned char tapAxes,
                             unsigned char tapDur,
                             unsigned char tapLatent,
                             unsigned char tapWindow,
                             unsigned char tapThresh,
                             unsigned char tapInt)
{
    unsigned char oldTapAxes    = 0;
    unsigned char newTapAxes    = 0;
    unsigned char oldIntMap     = 0;
    unsigned char newIntMap     = 0;
    unsigned char oldIntEnable  = 0;
    unsigned char newIntEnable  = 0;
    
    oldTapAxes = ADXL345_GetRegisterValue(ADXL345_TAP_AXES);
    newTapAxes = oldTapAxes & ~(ADXL345_TAP_X_EN |
                                ADXL345_TAP_Y_EN |
                                ADXL345_TAP_Z_EN);
    newTapAxes = newTapAxes | tapAxes;
    ADXL345_SetRegisterValue(ADXL345_TAP_AXES, newTapAxes);
    ADXL345_SetRegisterValue(ADXL345_DUR, tapDur);
    ADXL345_SetRegisterValue(ADXL345_LATENT, tapLatent);
    ADXL345_SetRegisterValue(ADXL345_WINDOW, tapWindow);
    ADXL345_SetRegisterValue(ADXL345_THRESH_TAP, tapThresh);
    oldIntMap = ADXL345_GetRegisterValue(ADXL345_INT_MAP);
    newIntMap = oldIntMap & ~(ADXL345_SINGLE_TAP | ADXL345_DOUBLE_TAP);
    newIntMap = newIntMap | tapInt;
    ADXL345_SetRegisterValue(ADXL345_INT_MAP, newIntMap);
    oldIntEnable = ADXL345_GetRegisterValue(ADXL345_INT_ENABLE);
    newIntEnable = oldIntEnable & ~(ADXL345_SINGLE_TAP | ADXL345_DOUBLE_TAP);
    newIntEnable = newIntEnable | tapType;
    ADXL345_SetRegisterValue(ADXL345_INT_ENABLE, newIntEnable);
}

/***************************************************************************//**
 * @brief Enables/disables the activity detection.
 *
 * @param actOnOff - Enables/disables the activity detection.
 *                     Example: 0x0 - disables the activity detection.
 *                              0x1 - enables the activity detection.
 * @param actAxes - Axes which participate in detecting activity.
 *                    Example: 0x0 - disables axes participation.
 *                             ADXL345_ACT_X_EN - enables x-axis participation.
 *                             ADXL345_ACT_Y_EN - enables y-axis participation.
 *                             ADXL345_ACT_Z_EN - enables z-axis participation.
 * @param actAcDc - Selects dc-coupled or ac-coupled operation.
 *                    Example: 0x0 - dc-coupled operation.
 *                             ADXL345_ACT_ACDC - ac-coupled operation.
 * @param actThresh - Threshold value for detecting activity.
 * @patam actInt - Interrupts pin.
 *                   Example: 0x0 - activity interrupts on INT1 pin.
 *                            ADXL345_ACTIVITY - activity interrupts on INT2 pin.
 *
 * @return None.
*******************************************************************************/
void ADXL345_SetActivityDetection(unsigned char actOnOff,
                                  unsigned char actAxes,
                                  unsigned char actAcDc,
                                  unsigned char actThresh,
                                  unsigned char actInt)
{
    unsigned char oldActInactCtl    = 0;
    unsigned char newActInactCtl    = 0;
    unsigned char oldIntMap         = 0;
    unsigned char newIntMap         = 0;
    unsigned char oldIntEnable      = 0;
    unsigned char newIntEnable      = 0;
    
    oldActInactCtl = ADXL345_GetRegisterValue(ADXL345_INT_ENABLE);
    newActInactCtl = oldActInactCtl & ~(ADXL345_ACT_ACDC |
                                        ADXL345_ACT_X_EN |
                                        ADXL345_ACT_Y_EN |
                                        ADXL345_ACT_Z_EN);
    newActInactCtl = newActInactCtl | (actAcDc | actAxes);
    ADXL345_SetRegisterValue(ADXL345_ACT_INACT_CTL, newActInactCtl);
    ADXL345_SetRegisterValue(ADXL345_THRESH_ACT, actThresh);
    oldIntMap = ADXL345_GetRegisterValue(ADXL345_INT_MAP);
    newIntMap = oldIntMap & ~(ADXL345_ACTIVITY);
    newIntMap = newIntMap | actInt;
    ADXL345_SetRegisterValue(ADXL345_INT_MAP, newIntMap);
    oldIntEnable = ADXL345_GetRegisterValue(ADXL345_INT_ENABLE);
    newIntEnable = oldIntEnable & ~(ADXL345_ACTIVITY);
    newIntEnable = newIntEnable | (ADXL345_ACTIVITY * actOnOff);
    ADXL345_SetRegisterValue(ADXL345_INT_ENABLE, newIntEnable);
}

/***************************************************************************//**
 * @brief Enables/disables the inactivity detection.
 *
 * @param inactOnOff - Enables/disables the inactivity detection.
 *                       Example: 0x0 - disables the inactivity detection.
 *                                0x1 - enables the inactivity detection.
 * @param inactAxes - Axes which participate in detecting inactivity.
 *                      Example: 0x0 - disables axes participation.
 *                               ADXL345_INACT_X_EN - enables x-axis.
 *                               ADXL345_INACT_Y_EN - enables y-axis.
 *                               ADXL345_INACT_Z_EN - enables z-axis.
 * @param inactAcDc - Selects dc-coupled or ac-coupled operation.
 *                      Example: 0x0 - dc-coupled operation.
 *                               ADXL345_INACT_ACDC - ac-coupled operation.
 * @param inactThresh - Threshold value for detecting inactivity.
 * @param inactTime - Inactivity time.
 * @patam inactInt - Interrupts pin.
 *                     Example: 0x0 - inactivity interrupts on INT1 pin.
 *                              ADXL345_INACTIVITY - inactivity interrupts on
 *                                                   INT2 pin.
 *
 * @return None.
*******************************************************************************/
void ADXL345_SetInactivityDetection(unsigned char inactOnOff,
                                    unsigned char inactAxes,
                                    unsigned char inactAcDc,
                                    unsigned char inactThresh,
                                    unsigned char inactTime,
                                    unsigned char inactInt)
{
    unsigned char oldActInactCtl    = 0;
    unsigned char newActInactCtl    = 0;
    unsigned char oldIntMap         = 0;
    unsigned char newIntMap         = 0;
    unsigned char oldIntEnable      = 0;
    unsigned char newIntEnable      = 0;
    
    oldActInactCtl = ADXL345_GetRegisterValue(ADXL345_INT_ENABLE);
    newActInactCtl = oldActInactCtl & ~(ADXL345_INACT_ACDC |
                                        ADXL345_INACT_X_EN |
                                        ADXL345_INACT_Y_EN |
                                        ADXL345_INACT_Z_EN);
    newActInactCtl = newActInactCtl | (inactAcDc | inactAxes);
    ADXL345_SetRegisterValue(ADXL345_ACT_INACT_CTL, newActInactCtl);
    ADXL345_SetRegisterValue(ADXL345_THRESH_INACT, inactThresh);
    ADXL345_SetRegisterValue(ADXL345_TIME_INACT, inactTime);
    oldIntMap = ADXL345_GetRegisterValue(ADXL345_INT_MAP);
    newIntMap = oldIntMap & ~(ADXL345_INACTIVITY);
    newIntMap = newIntMap | inactInt;
    ADXL345_SetRegisterValue(ADXL345_INT_MAP, newIntMap);
    oldIntEnable = ADXL345_GetRegisterValue(ADXL345_INT_ENABLE);
    newIntEnable = oldIntEnable & ~(ADXL345_INACTIVITY);
    newIntEnable = newIntEnable | (ADXL345_INACTIVITY * inactOnOff);
    ADXL345_SetRegisterValue(ADXL345_INT_ENABLE, newIntEnable);
}

/***************************************************************************//**
 * @brief Enables/disables the free-fall detection.
 *
 * @param ffOnOff - Enables/disables the free-fall detection.
 *                    Example: 0x0 - disables the free-fall detection.
 *                             0x1 - enables the free-fall detection.
 * @param ffThresh - Threshold value for free-fall detection.
 * @param ffTime - Time value for free-fall detection.
 * @param ffInt - Interrupts pin.
 *                  Example: 0x0 - free-fall interrupts on INT1 pin.
 *                           ADXL345_FREE_FALL - free-fall interrupts on INT2 pin.
 *
 * @return None.
*******************************************************************************/
void ADXL345_SetFreeFallDetection(unsigned char ffOnOff,
                                  unsigned char ffThresh,
                                  unsigned char ffTime,
                                  unsigned char ffInt)
{
    unsigned char oldIntMap     = 0;
    unsigned char newIntMap     = 0;
    unsigned char oldIntEnable  = 0;
    unsigned char newIntEnable  = 0;
    
    ADXL345_SetRegisterValue(ADXL345_THRESH_FF, ffThresh);
    ADXL345_SetRegisterValue(ADXL345_TIME_FF, ffTime);
    oldIntMap = ADXL345_GetRegisterValue(ADXL345_INT_MAP);
    newIntMap = oldIntMap & ~(ADXL345_FREE_FALL);
    newIntMap = newIntMap | ffInt;
    ADXL345_SetRegisterValue(ADXL345_INT_MAP, newIntMap);
    oldIntEnable = ADXL345_GetRegisterValue(ADXL345_INT_ENABLE);
    newIntEnable = oldIntEnable & ~ADXL345_FREE_FALL;
    newIntEnable = newIntEnable | (ADXL345_FREE_FALL * ffOnOff);
    ADXL345_SetRegisterValue(ADXL345_INT_ENABLE, newIntEnable);
}

/***************************************************************************//**
 * @brief Calibrates the accelerometer.
 *
 * @param xOffset - X-axis's offset.
 * @param yOffset - Y-axis's offset.
 * @param zOffset - Z-axis's offset.
 *
 * @return None.
*******************************************************************************/
void ADXL345_SetOffset(unsigned char xOffset,
                       unsigned char yOffset,
                       unsigned char zOffset)
{
    ADXL345_SetRegisterValue(ADXL345_OFSX, xOffset);
    ADXL345_SetRegisterValue(ADXL345_OFSY, yOffset);
    ADXL345_SetRegisterValue(ADXL345_OFSZ, yOffset);
}


void ADXL345_SetDataFormat(unsigned char format)
{
    ADXL345_SetRegisterValue(ADXL345_DATA_FORMAT, format);
}
