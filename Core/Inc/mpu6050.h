/*
 * mpu6050.h
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 */

#ifndef INC_GY521_H_
#define INC_GY521_H_

#endif /* INC_GY521_H_ */

#include <stdint.h>
#include "stm32f1xx_hal.h"

//#include "i2c.h"
#define RAD_TO_DEG 57.295779513082320876798154814105
// MPU6050 structure
typedef struct {

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    float Temperature;

    double KalmanAngleX;
    double KalmanAngleY;

    int16_t GyroX_offset; // ysys
    int16_t AccY_offset;  // ysys
    int16_t AccZ_offset;  // ysys
} MPU6050_t;


// Kalman structure
typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;


uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_GyroX_AccYZ_RAW(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct); // ysys  just need Gyro X, Y and Accel Z Raw values.
void MPU6050_Calib_GX_AYZ(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);         // ysys

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);
