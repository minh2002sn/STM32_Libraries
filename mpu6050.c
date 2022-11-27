/*
 * p_mpu6050.c
 *
 */

#include <math.h>

#ifdef __STM32F4XX_H
#include "stm32f4xx_hal.h"
#endif

#ifdef __STM32F1XX_H
#include "stm32f1xx_hal.h"
#endif

#ifdef __STM32F0XX_H
#include "stm32f0xx_hal.h"
#endif

#include "MPU6050.h"

extern I2C_HandleTypeDef hi2c2;
#define MPU6050_I2C hi2c2
#define I2C_TIMEOUT 100

#define MPU6050_ADDR 	    0x68
#define XG_OFFS_TC          0x00
#define YG_OFFS_TC          0x01
#define ZG_OFFS_TC          0x02
#define X_FINE_GAIN         0x03
#define Y_FINE_GAIN         0x04
#define Z_FINE_GAIN         0x05
#define XA_OFFS_H           0x06
#define XA_OFFS_L_TC        0x07
#define YA_OFFS_H           0x08
#define YA_OFFS_L_TC        0x09
#define ZA_OFFS_H           0x0A
#define ZA_OFFS_L_TC        0x0B
#define XG_OFFS_USRH        0x13
#define XG_OFFS_USRL        0x14
#define YG_OFFS_USRH        0x15
#define YG_OFFS_USRL        0x16
#define ZG_OFFS_USRH        0x17
#define ZG_OFFS_USRL        0x18
#define SMPLRT_DIV          0x19
#define CONFIG              0x1A
#define GYRO_CONFIG         0x1B
#define ACCEL_CONFIG        0x1C
#define FF_THR              0x1D
#define FF_DUR              0x1E
#define MOT_THR             0x1F
#define MOT_DUR             0x20
#define ZRMOT_THR           0x21
#define ZRMOT_DUR           0x22
#define FIFO_EN             0x23
#define I2C_MST_CTRL        0x24
#define I2C_SLV0_ADDR       0x25
#define I2C_SLV0_REG        0x26
#define I2C_SLV0_CTRL       0x27
#define I2C_SLV1_ADDR       0x28
#define I2C_SLV1_REG        0x29
#define I2C_SLV1_CTRL       0x2A
#define I2C_SLV2_ADDR       0x2B
#define I2C_SLV2_REG        0x2C
#define I2C_SLV2_CTRL       0x2D
#define I2C_SLV3_ADDR       0x2E
#define I2C_SLV3_REG        0x2F
#define I2C_SLV3_CTRL       0x30
#define I2C_SLV4_ADDR       0x31
#define I2C_SLV4_REG        0x32
#define I2C_SLV4_DO         0x33
#define I2C_SLV4_CTRL       0x34
#define I2C_SLV4_DI         0x35
#define I2C_MST_STATUS      0x36
#define INT_PIN_CFG         0x37
#define INT_ENABLE          0x38
#define DMP_INT_STATUS      0x39
#define INT_STATUS          0x3A
#define ACCEL_XOUT_H        0x3B
#define ACCEL_XOUT_L        0x3C
#define ACCEL_YOUT_H        0x3D
#define ACCEL_YOUT_L        0x3E
#define ACCEL_ZOUT_H        0x3F
#define ACCEL_ZOUT_L        0x40
#define TEMP_OUT_H          0x41
#define TEMP_OUT_L          0x42
#define GYRO_XOUT_H         0x43
#define GYRO_XOUT_L         0x44
#define GYRO_YOUT_H         0x45
#define GYRO_YOUT_L         0x46
#define GYRO_ZOUT_H         0x47
#define GYRO_ZOUT_L         0x48
#define EXT_SENS_DATA_00    0x49
#define EXT_SENS_DATA_01    0x4A
#define EXT_SENS_DATA_02    0x4B
#define EXT_SENS_DATA_03    0x4C
#define EXT_SENS_DATA_04    0x4D
#define EXT_SENS_DATA_05    0x4E
#define EXT_SENS_DATA_06    0x4F
#define EXT_SENS_DATA_07    0x50
#define EXT_SENS_DATA_08    0x51
#define EXT_SENS_DATA_09    0x52
#define EXT_SENS_DATA_10    0x53
#define EXT_SENS_DATA_11    0x54
#define EXT_SENS_DATA_12    0x55
#define EXT_SENS_DATA_13    0x56
#define EXT_SENS_DATA_14    0x57
#define EXT_SENS_DATA_15    0x58
#define EXT_SENS_DATA_16    0x59
#define EXT_SENS_DATA_17    0x5A
#define EXT_SENS_DATA_18    0x5B
#define EXT_SENS_DATA_19    0x5C
#define EXT_SENS_DATA_20    0x5D
#define EXT_SENS_DATA_21    0x5E
#define EXT_SENS_DATA_22    0x5F
#define EXT_SENS_DATA_23    0x60
#define MOT_DETECT_STATUS   0x61
#define I2C_SLV0_DO         0x63
#define I2C_SLV1_DO         0x64
#define I2C_SLV2_DO         0x65
#define I2C_SLV3_DO         0x66
#define I2C_MST_DELAY_CTRL  0x67
#define SIGNAL_PATH_RESET   0x68
#define MOT_DETECT_CTRL     0x69
#define USER_CTRL           0x6A
#define PWR_MGMT_1          0x6B
#define PWR_MGMT_2          0x6C
#define BANK_SEL            0x6D
#define MEM_START_ADDR      0x6E
#define MEM_R_W             0x6F
#define DMP_CFG_1           0x70
#define DMP_CFG_2           0x71
#define FIFO_COUNTH         0x72
#define FIFO_COUNTL         0x73
#define FIFO_R_W            0x74
#define WHO_AM_I            0x75

#define ACCEL_SENSITIVITY	16384.0
#define GYRO_SENSITIVITY 	65.536

float alpha = 0.98;
float dt = 0.01;

Kalman_t kalman_roll = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f
}, kalman_pitch = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f
};

static float Kalman_filter(Kalman_t *p_Kalman, float p_newAngle, float p_newRate);

bool MPU6050_init(void)
{
    uint8_t t_who_am_i;
    HAL_I2C_Mem_Read(&MPU6050_I2C, MPU6050_ADDR << 1, WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &t_who_am_i, 1, I2C_TIMEOUT);

    if (t_who_am_i == 0x68)
    {
    	uint8_t t_Data;
    	t_Data = 0;
        HAL_I2C_Mem_Write(&MPU6050_I2C, MPU6050_ADDR << 1, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &t_Data, 1, I2C_TIMEOUT);

        t_Data = 0x07;
        HAL_I2C_Mem_Write(&MPU6050_I2C, MPU6050_ADDR << 1, SMPLRT_DIV, I2C_MEMADD_SIZE_8BIT, &t_Data, 1, I2C_TIMEOUT);

        t_Data = 0x00;
        HAL_I2C_Mem_Write(&MPU6050_I2C, MPU6050_ADDR << 1, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &t_Data, 1, I2C_TIMEOUT);

        t_Data = 0x00;
        HAL_I2C_Mem_Write(&MPU6050_I2C, MPU6050_ADDR << 1, GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &t_Data, 1, I2C_TIMEOUT);

        t_Data = 0x01;
        HAL_I2C_Mem_Write(&MPU6050_I2C, MPU6050_ADDR << 1, INT_ENABLE, I2C_MEMADD_SIZE_8BIT, &t_Data, 1, I2C_TIMEOUT);

        t_Data = 0x00;
        HAL_I2C_Mem_Write(&MPU6050_I2C, MPU6050_ADDR << 1, USER_CTRL, I2C_MEMADD_SIZE_8BIT, &t_Data, 1, I2C_TIMEOUT);

        return true;
    }

    return false;
}

void MPU6050_read_accel(MPU6050_t *p_mpu6050)
{
    uint8_t t_Data[6];
    HAL_I2C_Mem_Read(&MPU6050_I2C, MPU6050_ADDR << 1, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, t_Data, sizeof(t_Data), I2C_TIMEOUT);

    int16_t t_accel_X_RAW = (int16_t)(t_Data[0] << 8 | t_Data[1]);
    int16_t t_accel_Y_RAW = (int16_t)(t_Data[2] << 8 | t_Data[3]);
    int16_t t_accel_Z_RAW = (int16_t)(t_Data[4] << 8 | t_Data[5]);
    p_mpu6050->accel_X = t_accel_X_RAW / ACCEL_SENSITIVITY - p_mpu6050->accel_bias[0];
    p_mpu6050->accel_Y = t_accel_Y_RAW / ACCEL_SENSITIVITY - p_mpu6050->accel_bias[1];
    p_mpu6050->accel_Z = t_accel_Z_RAW / ACCEL_SENSITIVITY - p_mpu6050->accel_bias[2];
}

void MPU6050_read_gyro(MPU6050_t *p_mpu6050)
{
    uint8_t t_Data[6];
    HAL_I2C_Mem_Read(&MPU6050_I2C, MPU6050_ADDR << 1, GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, t_Data, sizeof(t_Data), I2C_TIMEOUT);

    int16_t t_gyro_X_RAW = (int16_t)(t_Data[0] << 8 | t_Data[1]);
    int16_t t_gyro_Y_RAW = (int16_t)(t_Data[2] << 8 | t_Data[3]);
    int16_t t_gyro_Z_RAW = (int16_t)(t_Data[4] << 8 | t_Data[5]);
    p_mpu6050->gyro_X = t_gyro_X_RAW / GYRO_SENSITIVITY - p_mpu6050->gyro_bias[0];
    p_mpu6050->gyro_Y = t_gyro_Y_RAW / GYRO_SENSITIVITY - p_mpu6050->gyro_bias[1];
    p_mpu6050->gyro_Z = t_gyro_Z_RAW / GYRO_SENSITIVITY - p_mpu6050->gyro_bias[2];
}

void MPU6050_read_all(MPU6050_t *p_mpu6050)
{
    uint8_t t_Data[14];
    HAL_I2C_Mem_Read(&MPU6050_I2C, MPU6050_ADDR << 1, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, t_Data, sizeof(t_Data), I2C_TIMEOUT);

    int16_t t_accel_X_RAW = (int16_t)(t_Data[0] << 8 | t_Data[1]);
    int16_t t_accel_Y_RAW = (int16_t)(t_Data[2] << 8 | t_Data[3]);
    int16_t t_accel_Z_RAW = (int16_t)(t_Data[4] << 8 | t_Data[5]);
    int16_t t_temp_RAW = (int16_t)(t_Data[6] << 8 | t_Data[7]);
    int16_t t_gyro_X_RAW = (int16_t)(t_Data[8] << 8 | t_Data[9]);
    int16_t t_gyro_Y_RAW = (int16_t)(t_Data[10] << 8 | t_Data[11]);
    int16_t t_gyro_Z_RAW = (int16_t)(t_Data[12] << 8 | t_Data[13]);

    p_mpu6050->accel_X = t_accel_X_RAW / ACCEL_SENSITIVITY - p_mpu6050->accel_bias[0];
	p_mpu6050->accel_Y = t_accel_Y_RAW / ACCEL_SENSITIVITY - p_mpu6050->accel_bias[1];
	p_mpu6050->accel_Z = t_accel_Z_RAW / ACCEL_SENSITIVITY - p_mpu6050->accel_bias[2];

	p_mpu6050->gyro_X = t_gyro_X_RAW / GYRO_SENSITIVITY - p_mpu6050->gyro_bias[0];
	p_mpu6050->gyro_Y = t_gyro_Y_RAW / GYRO_SENSITIVITY - p_mpu6050->gyro_bias[1];
	p_mpu6050->gyro_Z = t_gyro_Z_RAW / GYRO_SENSITIVITY - p_mpu6050->gyro_bias[2];
}

void MPU6050_calib(MPU6050_t *p_mpu6050)
{
	float t_temp_accel[3] = {0, 0, 0};
	float t_temp_gyro[3] = {0, 0, 0};

	for (uint16_t i = 0; i < 256; i++)
	{
		MPU6050_t t_raw_data = {
				.accel_bias[0] = 0, .accel_bias[1] = 0, .accel_bias[2] = 0,
				.gyro_bias[0] = 0, .gyro_bias[1] = 0, .gyro_bias[2] = 0
		};
		MPU6050_read_all(&t_raw_data);
		t_temp_accel[0] += t_raw_data.accel_X;
		t_temp_accel[1] += t_raw_data.accel_Y;
		t_temp_accel[2] += t_raw_data.accel_Z;
		t_temp_gyro[0] += t_raw_data.gyro_X;
		t_temp_gyro[1] += t_raw_data.gyro_Y;
		t_temp_gyro[2] += t_raw_data.gyro_Z;
	}

	p_mpu6050->accel_X = p_mpu6050->accel_bias[0] = t_temp_accel[0] / 256;
	p_mpu6050->accel_Y = p_mpu6050->accel_bias[1] = t_temp_accel[1] / 256;
//	p_mpu6050->accel_Z = p_mpu6050->accel_bias[2] = temp_accel[2] / 256;
	p_mpu6050->gyro_X = p_mpu6050->gyro_bias[0] = t_temp_gyro[0] / 256;
	p_mpu6050->gyro_Y = p_mpu6050->gyro_bias[1] = t_temp_gyro[1] / 256;
	p_mpu6050->gyro_Z = p_mpu6050->gyro_bias[2] = t_temp_gyro[2] / 256;
}

static float Kalman_filter(Kalman_t *p_Kalman, float p_newAngle, float p_newRate)
{
	float t_rate = p_newRate - p_Kalman->bias;
	p_Kalman->angle += dt * t_rate;

	p_Kalman->P[0][0] += dt * (dt * p_Kalman->P[1][1] - p_Kalman->P[0][1] - p_Kalman->P[1][0] + p_Kalman->Q_angle);
	p_Kalman->P[0][1] -= dt * p_Kalman->P[1][1];
	p_Kalman->P[1][0] -= dt * p_Kalman->P[1][1];
	p_Kalman->P[1][1] += p_Kalman->Q_bias * dt;

	float t_S = p_Kalman->P[0][0] + p_Kalman->R_measure;
	float t_K[2];
	t_K[0] = p_Kalman->P[0][0] / t_S;
	t_K[1] = p_Kalman->P[1][0] / t_S;

	float t_y = p_newAngle - p_Kalman->angle;
	p_Kalman->angle += t_K[0] * t_y;
	p_Kalman->bias += t_K[1] * t_y;

	float t_P00_temp = p_Kalman->P[0][0];
	float t_P01_temp = p_Kalman->P[0][1];

	p_Kalman->P[0][0] -= t_K[0] * t_P00_temp;
	p_Kalman->P[0][1] -= t_K[0] * t_P01_temp;
	p_Kalman->P[1][0] -= t_K[1] * t_P00_temp;
	p_Kalman->P[1][1] -= t_K[1] * t_P01_temp;

	return p_Kalman->angle;
}

void MPU6050_callback(void *context)
{
	MPU6050_t *t_mpu6050 = (MPU6050_t *)context;
	MPU6050_read_all(t_mpu6050);

	float t_accel_roll  = atan2(t_mpu6050->accel_Y, t_mpu6050->accel_Z) * 180 / M_PI;
	float t_accel_pitch = atan2(-t_mpu6050->accel_X, sqrt(t_mpu6050->accel_Y * t_mpu6050->accel_Y + t_mpu6050->accel_Z * t_mpu6050->accel_Z)) * 180 / M_PI;

	float t_roll_rate = t_mpu6050->gyro_X * dt;
	float t_pitch_rate = t_mpu6050->gyro_Y * dt;
	float t_yaw_rate = t_mpu6050->gyro_Z * dt;

	#ifndef KALMAN_FILTER
		// Complementary filter
		t_mpu6050->roll = alpha * (t_mpu6050->roll + t_roll_rate) + (1 - alpha) * t_accel_roll;
		t_mpu6050->pitch = alpha * (t_mpu6050->pitch + t_pitch_rate) + (1 - alpha) * t_accel_pitch;
	#else
		// Kalman filter
		t_mpu6050->roll = Kalman_filter(&kalman_roll, t_accel_roll, t_roll_rate);
		t_mpu6050->pitch = Kalman_filter(&kalman_pitch, t_accel_pitch, t_pitch_rate);
	#endif

	t_mpu6050->yaw = t_mpu6050->yaw + t_yaw_rate;
}
