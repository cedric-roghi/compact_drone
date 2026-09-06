#include "icm42605.h"
#include "icm42605_registers.h"
#include <string.h>

#define TEMP_DATA_REG_SCALE 132.48f
#define TEMP_OFFSET         25.0f
#define NUM_CALIB_SAMPLES   1000

extern void usb_send(const char *format, ...);

static int write_register(ICM42605_t *dev, uint8_t reg, uint8_t data) {
    if (HAL_I2C_Mem_Write(dev->hi2c, dev->dev_addr, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100) != HAL_OK) {
        return -1;
    }
    HAL_Delay(10);
    
    uint8_t check = 0;
    if (HAL_I2C_Mem_Read(dev->hi2c, dev->dev_addr, reg, I2C_MEMADD_SIZE_8BIT, &check, 1, 100) != HAL_OK) {
        return -1;
    }
    return (check == data) ? 1 : -1;
}

static int read_registers(ICM42605_t *dev, uint8_t reg, uint8_t count, uint8_t *dest) {
    if (HAL_I2C_Mem_Read(dev->hi2c, dev->dev_addr, reg, I2C_MEMADD_SIZE_8BIT, dest, count, 100) == HAL_OK) {
        return 1;
    }
    return -1;
}

static int set_bank(ICM42605_t *dev, uint8_t bank) {
    if (dev->bank == bank) return 1;
    dev->bank = bank;
    return write_register(dev, REG_BANK_SEL, bank);
}

static void reset_device(ICM42605_t *dev) {
    set_bank(dev, 0);
    write_register(dev, UB0_REG_DEVICE_CONFIG, 0x01);
    HAL_Delay(1);
}

static uint8_t who_am_i(ICM42605_t *dev) {
    set_bank(dev, 0);
    uint8_t buf = 0;
    if (read_registers(dev, UB0_REG_WHO_AM_I, 1, &buf) < 0) return 0xFF;
    return buf;
}

int icm42605_set_accel_fs(ICM42605_t *dev, ICM42605_AccelFS fssel) {
    set_bank(dev, 0);
    uint8_t reg = 0;
    if (read_registers(dev, UB0_REG_ACCEL_CONFIG0, 1, &reg) < 0) return -1;
    
    reg = (fssel << 5) | (reg & 0x1F);
    if (write_register(dev, UB0_REG_ACCEL_CONFIG0, reg) < 0) return -2;
    
    dev->accel_scale = (float)(1 << (4 - fssel)) / 32768.0f;
    dev->accel_fs = fssel;
    return 1;
}

int icm42605_set_gyro_fs(ICM42605_t *dev, ICM42605_GyroFS fssel) {
    set_bank(dev, 0);
    uint8_t reg = 0;
    if (read_registers(dev, UB0_REG_GYRO_CONFIG0, 1, &reg) < 0) return -1;
    
    reg = (fssel << 5) | (reg & 0x1F);
    if (write_register(dev, UB0_REG_GYRO_CONFIG0, reg) < 0) return -2;
    
    dev->gyro_scale = (2000.0f / (float)(1 << fssel)) / 32768.0f;
    dev->gyro_fs = fssel;
    return 1;
}

int icm42605_set_accel_odr(ICM42605_t *dev, ICM42605_ODR odr) {
    set_bank(dev, 0);
    uint8_t reg = 0;
    if (read_registers(dev, UB0_REG_ACCEL_CONFIG0, 1, &reg) < 0) return -1;
    reg = odr | (reg & 0xF0);
    return write_register(dev, UB0_REG_ACCEL_CONFIG0, reg);
}

int icm42605_set_gyro_odr(ICM42605_t *dev, ICM42605_ODR odr) {
    set_bank(dev, 0);
    uint8_t reg = 0;
    if (read_registers(dev, UB0_REG_GYRO_CONFIG0, 1, &reg) < 0) return -1;
    reg = odr | (reg & 0xF0);
    return write_register(dev, UB0_REG_GYRO_CONFIG0, reg);
}

int icm42605_init(ICM42605_t *dev, I2C_HandleTypeDef *hi2c, uint8_t i2c_addr) {
    dev->hi2c = hi2c;
    dev->dev_addr = i2c_addr << 1; // STM32 HAL requires 8-bit left-shifted address
    dev->bank = 255;
    dev->acc_s[0] = 1.0f; dev->acc_s[1] = 1.0f; dev->acc_s[2] = 1.0f;
    dev->acc_b[0] = 0.0f; dev->acc_b[1] = 0.0f; dev->acc_b[2] = 0.0f;
    dev->gyr_b[0] = 0.0f; dev->gyr_b[1] = 0.0f; dev->gyr_b[2] = 0.0f;

    reset_device(dev);

    HAL_Delay(50);

    // 3. Read WHO_AM_I and print the returned byte before returning -3
    uint8_t id = who_am_i(dev);
    if (id != ICM42605_WHO_AM_I_VAL) {
        usb_send("WHO_AM_I failed! Expected 0x42, got 0x%02X\r\n", id);
        return -3;
    }

    if (write_register(dev, UB0_REG_PWR_MGMT0, 0x0F) < 0) {
        return -4;
    }

    if (icm42605_set_accel_fs(dev, ACCEL_FS_GPM16) < 0) return -5;
    if (icm42605_set_gyro_fs(dev, GYRO_FS_DPS2000) < 0) return -6;

    return icm42605_calibrate_gyro(dev);
}

int icm42605_get_agt(ICM42605_t *dev) {
    uint8_t buffer[14];
    if (read_registers(dev, UB0_REG_TEMP_DATA1, 14, buffer) < 0) return -1;

    for (int i = 0; i < 7; i++) {
        dev->raw_meas[i] = (int16_t)((buffer[i * 2] << 8) | buffer[i * 2 + 1]);
    }

    dev->temp = ((float)dev->raw_meas[0] / TEMP_DATA_REG_SCALE) + TEMP_OFFSET;

    // Temporary variables for raw scaled values before orientation remapping
    float raw_acc[3], raw_gyr[3];

    raw_acc[0] = ((dev->raw_meas[1] * dev->accel_scale) - dev->acc_b[0]) * dev->acc_s[0];
    raw_acc[1] = ((dev->raw_meas[2] * dev->accel_scale) - dev->acc_b[1]) * dev->acc_s[1];
    raw_acc[2] = ((dev->raw_meas[3] * dev->accel_scale) - dev->acc_b[2]) * dev->acc_s[2];

    raw_gyr[0] = (dev->raw_meas[4] * dev->gyro_scale) - dev->gyr_b[0];
    raw_gyr[1] = (dev->raw_meas[5] * dev->gyro_scale) - dev->gyr_b[1];
    raw_gyr[2] = (dev->raw_meas[6] * dev->gyro_scale) - dev->gyr_b[2];

    // Remap axes to match your upright board mounting orientation
    // (Adjust these assignments/signs depending on how your board is physically rotated)
    dev->acc[0] = raw_acc[2];   
    dev->acc[1] = raw_acc[1];
    dev->acc[2] = -raw_acc[0]; 

    dev->gyr[0] = raw_gyr[2];
    dev->gyr[1] = raw_gyr[1];
    dev->gyr[2] = -raw_gyr[0];

    return 1;
}

// old code
// int icm42605_get_agt(ICM42605_t *dev) {
//     uint8_t buffer[14];
//     if (read_registers(dev, UB0_REG_TEMP_DATA1, 14, buffer) < 0) return -1;

//     for (int i = 0; i < 7; i++) {
//         dev->raw_meas[i] = (int16_t)((buffer[i * 2] << 8) | buffer[i * 2 + 1]);
//     }

//     dev->temp = ((float)dev->raw_meas[0] / TEMP_DATA_REG_SCALE) + TEMP_OFFSET;

//     dev->acc[0] = ((dev->raw_meas[1] * dev->accel_scale) - dev->acc_b[0]) * dev->acc_s[0];
//     dev->acc[1] = ((dev->raw_meas[2] * dev->accel_scale) - dev->acc_b[1]) * dev->acc_s[1];
//     dev->acc[2] = ((dev->raw_meas[3] * dev->accel_scale) - dev->acc_b[2]) * dev->acc_s[2];

//     dev->gyr[0] = (dev->raw_meas[4] * dev->gyro_scale) - dev->gyr_b[0];
//     dev->gyr[1] = (dev->raw_meas[5] * dev->gyro_scale) - dev->gyr_b[1];
//     dev->gyr[2] = (dev->raw_meas[6] * dev->gyro_scale) - dev->gyr_b[2];

//     return 1;
// }

int icm42605_calibrate_gyro(ICM42605_t *dev) {
    ICM42605_GyroFS current_fs = dev->gyro_fs;
    if (icm42605_set_gyro_fs(dev, GYRO_FS_DPS250) < 0) return -1;

    float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
    for (size_t i = 0; i < NUM_CALIB_SAMPLES; i++) {
        icm42605_get_agt(dev);
        gyro_sum[0] += (dev->gyr[0] + dev->gyr_b[0]) / NUM_CALIB_SAMPLES;
        gyro_sum[1] += (dev->gyr[1] + dev->gyr_b[1]) / NUM_CALIB_SAMPLES;
        gyro_sum[2] += (dev->gyr[2] + dev->gyr_b[2]) / NUM_CALIB_SAMPLES;
        HAL_Delay(1);
    }

    dev->gyr_b[0] = gyro_sum[0];
    dev->gyr_b[1] = gyro_sum[1];
    dev->gyr_b[2] = gyro_sum[2];

    return icm42605_set_gyro_fs(dev, current_fs);
}