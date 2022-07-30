/* 
 * File:   BNO055.h
 * Author: Aaron Hunter
 * 
 * Software module to communicate with the IMU over I2C.
 * Provides access to each raw sensor axis along with raw temperature
 *
 * Created on April 11, 2022, 8:58 AM
 */

#ifndef BNO055_H
#define	BNO055_H

#include <stdint.h>

/**
 * @Function BNO055_Init(void)

 * @return 0 if error, 1 if succeeded
 * @brief  Initializes the BNO055 for usage. Sensors will be at Accel: 2g,Gyro:  250dps and Mag: 16-bit
 * @author Aaron Hunter */
char BNO055_Init(int i2c_handle);


/**
 * @Function BNO055_Read*()
 * @param None
 * @return Returns raw sensor reading
 * @brief reads sensor axis as given by name
 * @author Aaron Hunter*/
int BNO055_ReadAccel(int i2c_handle, int16_t data[]);


/**
 * @Function BNO055_Read*()
 * @param None
 * @return Returns raw sensor reading
 * @brief reads sensor data block as given by name
 * @author Aaron Hunter*/
int BNO055_ReadGyro(int i2c_handle, int16_t data[]);

/**
 * @Function BNO055_Read*()
 * @param None
 * @return Returns raw sensor reading
 * @brief reads sensor axis as given by name
 * @author Aaron Hunter*/
int BNO055_ReadMag(int i2c_handle, int16_t data[]);


/**
 * @Function BNO055_Read*()
 * @param None
 * @return Returns raw sensor reading
 * @brief reads sensor axis as given by name
 * @author Aaron Hunter*/
int BNO055_ReadTemp(void);

#endif