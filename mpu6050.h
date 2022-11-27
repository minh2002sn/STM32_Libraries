/*
 * mpu6050.h
 *
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

// MPU6050 structure
typedef struct {
    float accel_X;
    float accel_Y;
    float accel_Z;

    float gyro_X;
    float gyro_Y;
    float gyro_Z;

    float roll;
    float pitch;
    float yaw;

    float gyro_bias[3];
    float accel_bias[3];
} MPU6050_t;

typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

// Function prototypes
bool MPU6050_init(void);
void MPU6050_read_accel(MPU6050_t *p_mpu6050);
void MPU6050_read_gyro(MPU6050_t *p_mpu6050);
void MPU6050_read_all(MPU6050_t *p_mpu6050);
void MPU6050_calib(MPU6050_t *p_mpu6050);
void MPU6050_callback(void *p_context);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H_ */
