/*
 * mpu6050.c
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 *
 *  Contact information
 *  -------------------
 *
 * e-mail   :  leech001@gmail.com
 */

/*
 * |---------------------------------------------------------------------------------
 * | Copyright (C) Bulanov Konstantin,2019
 * |
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |
 * | Kalman filter algorithm used from https://github.com/TKJElectronics/KalmanFilter
 * |---------------------------------------------------------------------------------
 */


#include <math.h>
#include "cmsis_os2.h"
#include "mpu6050.h"



#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define ACCEL_YOUT_H_REG 0x3D // ysys
#define ACCEL_ZOUT_H_REG 0x3F // ysys
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43

// Setup MPU6050
#define MPU6050_ADDR 0xD0
const uint16_t i2c_timeout = 100;
const double Accel_Z_corrector = 14418.0;

uint32_t timer;

Kalman_t KalmanX = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f
};

Kalman_t KalmanY = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f,
};

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx) {
    uint8_t check;
    uint8_t Data;
    // reset
    Data = 0x80;
    HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);
    osDelay(100);
#define CONFIG           0x1A
#define SMPLRT_DIV       0x19
#define ACCEL_CONFIG     0x1C
#define GYRO_CONFIG      0x1B

    /*
    // calib
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data

	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x24, 1, &Data, 1, i2c_timeout); // I2C_MST_CTRL
	Data = 0x00;  // Wake up
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x6a, 1, &Data, 1, i2c_timeout); // USER_CTRL
	Data = 0x0C;  // Wake up
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x6a, 1, &Data, 1, i2c_timeout); // USER_CTRL
	osDelay(20);

    Data = 0x01;  // Config Gyro, Accel
    HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, CONFIG, 1, &Data, 1, i2c_timeout);
    Data = 0x00;  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV, 1, &Data, 1, i2c_timeout);
    Data = 0x00;
    HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG, 1, &Data, 1, i2c_timeout);

    HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG, 1, &Data, 1, i2c_timeout);

    uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
    uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

    Data = 0x40;  // Enable FiFo
    HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x6a, 1, &Data, 1, i2c_timeout); // USER_CTRL
    Data = 0x78;  // Enable FiFo
    HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x23, 1, &Data, 1, i2c_timeout); // FIFO_EN
    osDelay(100);

    Data = 0x00;  // Enable FiFo
    HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x23, 1, &Data, 1, i2c_timeout); // FIFO_EN
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, 0x72, 1, &data[0], 2, i2c_timeout); // FIFO_COUNTH
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging


    for (ii = 0; ii < packet_count; ii++) {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        // readBytes(MPU6050_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
        HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, 0x74, 1, &data[0], 12, i2c_timeout);
        accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
        accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
        gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
        gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
        gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

        accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0]  += (int32_t) gyro_temp[0];
        gyro_bias[1]  += (int32_t) gyro_temp[1];
        gyro_bias[2]  += (int32_t) gyro_temp[2];

    }
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0]  /= (int32_t) packet_count;
	gyro_bias[1]  /= (int32_t) packet_count;
	gyro_bias[2]  /= (int32_t) packet_count;

	if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
	else {accel_bias[2] += (int32_t) accelsensitivity;}

	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1]/4)       & 0xFF;
	data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2]/4)       & 0xFF;


	#define XG_OFFS_USRH     0x13
	for(int i=0; i<6; i++)
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, XG_OFFS_USRH+i, 1, &data[i], 1, i2c_timeout);

	dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
	dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
	dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

	#define XA_OFFSET_H      0x06
	#define YA_OFFSET_H      0x08
	#define ZA_OFFSET_H      0x0A
	int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
	//readBytes(MPU6050_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, XA_OFFSET_H, 1, &data[0], 2, i2c_timeout);
	accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
	//readBytes(MPU6050_ADDRESS, YA_OFFSET_H, 2, &data[0]);
	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, YA_OFFSET_H, 1, &data[0], 2, i2c_timeout);
	accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
	//readBytes(MPU6050_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ZA_OFFSET_H, 1, &data[0], 2, i2c_timeout);
	accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

	  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

	  for(ii = 0; ii < 3; ii++) {
	    if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	  }

	  // Construct total accelerometer bias, including calculated average accelerometer bias from above
	  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	  accel_bias_reg[1] -= (accel_bias[1]/8);
	  accel_bias_reg[2] -= (accel_bias[2]/8);

	  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	  data[1] = (accel_bias_reg[0])      & 0xFF;
	  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	  data[3] = (accel_bias_reg[1])      & 0xFF;
	  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	  data[5] = (accel_bias_reg[2])      & 0xFF;
	  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

	  // Push accelerometer biases to hardware registers
	//  writeByte(MPU6050_ADDRESS, XA_OFFSET_H, data[0]);
	//  writeByte(MPU6050_ADDRESS, XA_OFFSET_L_TC, data[1]);
	//  writeByte(MPU6050_ADDRESS, YA_OFFSET_H, data[2]);
	//  writeByte(MPU6050_ADDRESS, YA_OFFSET_L_TC, data[3]);
	//  writeByte(MPU6050_ADDRESS, ZA_OFFSET_H, data[4]);
	//  writeByte(MPU6050_ADDRESS, ZA_OFFSET_L_TC, data[5]);
	#define XA_OFFSET_H      0x06
	  for(int i=0; i<6; i++)
	  		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, XA_OFFSET_H+i, 1, &data[i], 1, i2c_timeout);

	// Output scaled accelerometer biases for manual subtraction in the main program
	   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity;
	   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
	   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;

	// End of Calib
*/
    // init Start

	//check = Data & ~0xE0;
	//HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG, 1, &check, 1, i2c_timeout); // Clear self-test bits [7:5]
	//check = Data & ~0x18;
	//HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG, 1, &check, 1, i2c_timeout); // Clear AFS bits [4:3]
	//check = Data | 0 << 3;
	//HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG, 1, &check, 1, i2c_timeout); // Set full scale range for the gyro

/*
    Data = 0x01;  // Get Stable Time Source
    HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);

    Data = 0x03;  // Config Gyro, Accel
    HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, CONFIG, 1, &Data, 1, i2c_timeout);

    Data = 0x04;  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV, 1, &Data, 1, i2c_timeout);
*/

    // init End
/*
    Data = 0x80;
    HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);
    osDelay(100);
    Data = 0x07;
    HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x68, 1, &Data, 1, i2c_timeout);  // SIGNAL_PATH_RESET 0x68
    osDelay(100);

    Data = 0x40;
    HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x6c, 1, &Data, 1, i2c_timeout);  // PWR_MGMT_2_REG 0x6c
*/

    // check device ID WHO_AM_I

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);

    if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);
/*
        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x07;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, i2c_timeout);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);
*/
        Data = 0x00;  // Wake up
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);
        osDelay(100);

    	#define ACCEL_CONFIG     0x1C
    	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_CONFIG, 1, &Data, 1, i2c_timeout);
    	check = (Data & 0xE7) | (uint8_t)0 << 3; // 0:250 deg, 1:500, 2:1000, 3:2000
    	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG, 1, &check, 1, i2c_timeout);

    	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_CONFIG, 1, &Data, 1, i2c_timeout);
    	check = (Data & 0xE7) | (uint8_t)0 << 3; // 0:2g, 1:4g, 2:8g, 3:16g
    	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG, 1, &check, 1, i2c_timeout);
        return 0;
    }
    return 1;
}


void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into acceleration in 'g'
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 16384.0
         for more details check ACCEL_CONFIG Register              ****/

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;

}


// ysys  just need Gyro X, Y and Accel Z Raw values.
void MPU6050_Read_GyroX_AccYZ_RAW(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
	uint8_t Rec_Data[4];

	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 4, i2c_timeout);
    DataStruct->Gyro_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_YOUT_H_REG, 1, Rec_Data, 4, i2c_timeout);
    DataStruct->Accel_Y_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Z_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
}

void MPU6050_Calib_GX_AYZ(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
	uint8_t Rec_Data[4];
	int32_t sumGX=0, sumAY=0, sumAZ=0;
	int16_t tempX, tempY, tempZ;
	int i=0, j=0;

    DataStruct->GyroX_offset=DataStruct->AccY_offset=DataStruct->AccZ_offset=0;
    osDelay(300); // need little time to start up.
#define REPEAT 150
	while(i<REPEAT+1){ // ignore first value.
		j++;
		HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 2, i2c_timeout);
		tempX = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);

		HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_YOUT_H_REG, 1, Rec_Data, 4, i2c_timeout);
		tempY = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
		tempZ = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
		if(i==0){
			i++; // very first value is weird. should be dropped.
			continue;
		}
		if(tempX!=0 && tempY!=0 && tempZ!=0){
			i++;
			sumGX+=tempX;
			sumAY+=tempY;
			sumAZ+=tempZ;
		}
		osDelay(5);
	}
	DataStruct->GyroX_offset= sumGX/REPEAT/4;
	DataStruct->AccY_offset= sumAY/REPEAT/8;
	DataStruct->AccZ_offset= (int16_t)((16384-(sumAZ/REPEAT))/8);
}



void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from GYRO_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Gyro_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into dps (�/s)
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 131.0
         for more details check GYRO_CONFIG Register              ****/

    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;
}

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[2];
    int16_t temp;

    // Read 2 BYTES of data starting from TEMP_OUT_H_REG register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, i2c_timeout);

    temp = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);
}

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[14];
    int16_t temp;

    // Read 14 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);
    temp = (int16_t) (Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW = (int16_t) (Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t) (Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t) (Rec_Data[12] << 8 | Rec_Data[13]);

    if( (DataStruct->Gyro_X_RAW==DataStruct->Gyro_Y_RAW && DataStruct->Gyro_X_RAW==DataStruct->Gyro_Z_RAW)
      ||(DataStruct->Accel_X_RAW==DataStruct->Accel_Y_RAW && DataStruct->Accel_X_RAW==DataStruct->Accel_Z_RAW))
    	return;
    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
    DataStruct->Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;

    // Kalman angle solve
    double dt = (double) (HAL_GetTick() - timer) / 1000;
    timer = HAL_GetTick();
    double roll;
    double roll_sqrt = sqrt(
            DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0) {
        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
    } else {
        roll = 0.0;
    }
    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90)) {
        KalmanY.angle = pitch;
        DataStruct->KalmanAngleY = pitch;
    } else {
        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    }
    if (fabs(DataStruct->KalmanAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gy, dt);

}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt) {
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};
