/*
*************************
*                       *
* 		TRACTO TEC      *
*                       *
*     Noviembre 2023    *
*                       *
*************************
*/

#include <math.h>
#include "imuMPU925.h"

#define RAD_TO_DEG 57.295779513082320876798154814105

#define WHO_AM_I_REG 0x75 //registro 117 de registro who am i
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B //59
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43

// Setup MPU6050
#define MPU6050_ADDR 0x75 //registro 117 en el address who am i
//const uint16_t i2c_timeout = 100;
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

void mpu9250_write_register( mpu9250* mpu, uint8_t address, uint8_t data){
	HAL_GPIO_WritePin(mpu->cs_gpio_port, mpu->cs_gpio_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(mpu->hspi, &address, 1, 100);
	HAL_SPI_Transmit(mpu->hspi, &data, 1, 100);
	HAL_GPIO_WritePin(mpu->cs_gpio_port, mpu->cs_gpio_pin, GPIO_PIN_SET);
}

void mpu9250_read_register( mpu9250* mpu, uint8_t address, uint8_t* buffer, uint8_t len){
	uint8_t _address = 0x80|address; // Set MSB to 1 for reading
	HAL_GPIO_WritePin(mpu->cs_gpio_port, mpu->cs_gpio_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(mpu->hspi, &_address , 1, 100);
	HAL_SPI_Receive(mpu->hspi, buffer, len, 100);
	HAL_GPIO_WritePin(mpu->cs_gpio_port, mpu->cs_gpio_pin, GPIO_PIN_SET);
}


void MPU9250_Read_All(MPU925_t *mpu) {
    uint8_t imu_accel[14];
    uint8_t imu_gyro[14];

    // Read 14 BYTES of data starting from ACCEL_XOUT_H register

    mpu9250_write_register(mpu, ACCEL_XOUT_H_REG, imu_accel, 14);
	//mpu9250_read_reg(ACCEL_XOUT_H_REG, imu_accel, sizeof(imu_accel));
    mpu->Accel_X_RAW = (int16_t) (imu_accel[0] << 8 | imu_accel[1]);
    mpu->Accel_Y_RAW = (int16_t) (imu_accel[2] << 8 | imu_accel[3]);
    mpu->Accel_Z_RAW = (int16_t) (imu_accel[4] << 8 | imu_accel[5]);

    mpu9250_write_register(mpu, GYRO_XOUT_H_REG,imu_gyro, 14);
	//mpu9250_read_reg(GYRO_XOUT_H_REG, imu_gyro, sizeof(imu_gyro));
    mpu->Gyro_X_RAW = (int16_t) (imu_gyro[8] << 8 | imu_gyro[9]);
    mpu->Gyro_Y_RAW = (int16_t) (imu_gyro[10] << 8 | imu_gyro[11]);
    mpu->Gyro_Z_RAW = (int16_t) (imu_gyro[12] << 8 | imu_gyro[13]);

    mpu->Ax = mpu->Accel_X_RAW / 16384.0;
    mpu->Ay = mpu->Accel_Y_RAW / 16384.0;
    mpu->Az = mpu->Accel_Z_RAW / Accel_Z_corrector;
    mpu->Gx = mpu->Gyro_X_RAW / 131.0;
    mpu->Gy = mpu->Gyro_Y_RAW / 131.0;
    mpu->Gz = mpu->Gyro_Z_RAW / 131.0;

    // Kalman angle solve
    double dt = (double) (HAL_GetTick() - timer) / 1000;
    timer = HAL_GetTick();
    double roll;
    double roll_sqrt = sqrt(
            mpu->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0) {
        roll = atan(mpu->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
    } else {
        roll = 0.0;
    }
    double pitch = atan2(-mpu->Accel_X_RAW, mpu->Accel_Z_RAW) * RAD_TO_DEG;
    if ((pitch < -90 && mpu->KalmanAngleY > 90) || (pitch > 90 && mpu->KalmanAngleY < -90)) {
        KalmanY.angle = pitch;
        mpu->KalmanAngleY = pitch;
    } else {
        mpu->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, mpu->Gy, dt);
    }
    if (fabs(mpu->KalmanAngleY) > 90)
        mpu->Gx = -mpu->Gx;
    mpu->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, mpu->Gy, dt);

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
