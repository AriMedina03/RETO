/*
*************************
*                       *
* 	   TRACTO TEC 	    *
*                       *
*     Noviembre 2023    *
*                       *
*************************
*/

#include <stdint.h>

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

    double KalmanAngleX;
    double KalmanAngleY;
    GPIO_TypeDef* cs_gpio_port;
    uint16_t cs_gpio_pin;
    SPI_HandleTypeDef* hspi;

} MPU925_t;


// Kalman structure
typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;


void mpu9250_write_register( mpu9250* mpu, uint8_t address, uint8_t data);
void mpu9250_read_register( mpu9250* mpu, uint8_t address, uint8_t* buffer, uint8_t len);
void MPU9250_Read_All(MPU925_t *mpu);

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);
