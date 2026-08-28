#ifndef ICM42605_H_
#define ICM42605_H_

#include <stdint.h>
#include "main.h" // For I2C_HandleTypeDef definition

typedef enum {
    GYRO_FS_DPS2000 = 0x00,
    GYRO_FS_DPS1000 = 0x01,
    GYRO_FS_DPS500  = 0x02,
    GYRO_FS_DPS250  = 0x03,
    GYRO_FS_DPS125  = 0x04,
    GYRO_FS_DPS62_5 = 0x05,
    GYRO_FS_DPS31_25 = 0x06,
    GYRO_FS_DPS15_625 = 0x07
} ICM42605_GyroFS;

typedef enum {
    ACCEL_FS_GPM16 = 0x00,
    ACCEL_FS_GPM8  = 0x01,
    ACCEL_FS_GPM4  = 0x02,
    ACCEL_FS_GPM2  = 0x03
} ICM42605_AccelFS;

typedef enum {
    ODR_32KHZ   = 0x01,
    ODR_16KHZ   = 0x02,
    ODR_8KHZ    = 0x03,
    ODR_4KHZ    = 0x04,
    ODR_2KHZ    = 0x05,
    ODR_1KHZ    = 0x06,
    ODR_200HZ   = 0x07,
    ODR_100HZ   = 0x08,
    ODR_50HZ    = 0x09,
    ODR_25HZ    = 0x0A,
    ODR_12_5HZ  = 0x0B,
    ODR_500HZ   = 0x0F
} ICM42605_ODR;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t dev_addr;          // Left-shifted 8-bit I2C address
    uint8_t bank;
    
    ICM42605_AccelFS accel_fs;
    ICM42605_GyroFS  gyro_fs;
    
    float accel_scale;
    float gyro_scale;
    
    float acc_b[3];            // Accel bias
    float acc_s[3];            // Accel scale factors
    float gyr_b[3];            // Gyro bias
    
    float acc[3];              // Converted acceleration (g)
    float gyr[3];              // Converted angular velocity (dps)
    float temp;                // Temperature (°C)
    
    int16_t raw_meas[7];       // Temp, Accel XYZ, Gyro XYZ
} ICM42605_t;

int icm42605_init(ICM42605_t *dev, I2C_HandleTypeDef *hi2c, uint8_t i2c_addr);
int icm42605_set_accel_fs(ICM42605_t *dev, ICM42605_AccelFS fssel);
int icm42605_set_gyro_fs(ICM42605_t *dev, ICM42605_GyroFS fssel);
int icm42605_set_accel_odr(ICM42605_t *dev, ICM42605_ODR odr);
int icm42605_set_gyro_odr(ICM42605_t *dev, ICM42605_ODR odr);
int icm42605_get_agt(ICM42605_t *dev);
int icm42605_calibrate_gyro(ICM42605_t *dev);

#endif // ICM42605_H_