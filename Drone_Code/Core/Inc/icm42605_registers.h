#ifndef ICM42605_REGISTERS_H_
#define ICM42605_REGISTERS_H_

#include <stdint.h>

// Accessible from all user banks
#define REG_BANK_SEL                   0x76

// User Bank 0
#define UB0_REG_DEVICE_CONFIG          0x11
#define UB0_REG_DRIVE_CONFIG           0x13
#define UB0_REG_INT_CONFIG             0x14
#define UB0_REG_FIFO_CONFIG            0x16
#define UB0_REG_TEMP_DATA1             0x1D
#define UB0_REG_TEMP_DATA0             0x1E
#define UB0_REG_ACCEL_DATA_X1          0x1F
#define UB0_REG_ACCEL_DATA_X0          0x20
#define UB0_REG_ACCEL_DATA_Y1          0x21
#define UB0_REG_ACCEL_DATA_Y0          0x22
#define UB0_REG_ACCEL_DATA_Z1          0x23
#define UB0_REG_ACCEL_DATA_Z0          0x24
#define UB0_REG_GYRO_DATA_X1           0x25
#define UB0_REG_GYRO_DATA_X0           0x26
#define UB0_REG_GYRO_DATA_Y1           0x27
#define UB0_REG_GYRO_DATA_Y0           0x28
#define UB0_REG_GYRO_DATA_Z1           0x29
#define UB0_REG_GYRO_DATA_Z0           0x2A
#define UB0_REG_INT_STATUS             0x2D
#define UB0_REG_SIGNAL_PATH_RESET      0x4B
#define UB0_REG_INTF_CONFIG0           0x4C
#define UB0_REG_INTF_CONFIG1           0x4D
#define UB0_REG_PWR_MGMT0              0x4E
#define UB0_REG_GYRO_CONFIG0           0x4F
#define UB0_REG_ACCEL_CONFIG0          0x50
#define UB0_REG_GYRO_CONFIG1           0x51
#define UB0_REG_GYRO_ACCEL_CONFIG0     0x52
#define UB0_REG_ACCEL_CONFIG1          0x53
#define UB0_REG_INT_CONFIG0            0x63
#define UB0_REG_INT_CONFIG1            0x64
#define UB0_REG_INT_SOURCE0            0x65
#define UB0_REG_WHO_AM_I               0x75

// User Bank 1
#define UB1_REG_SENSOR_CONFIG0         0x03
#define UB1_REG_GYRO_CONFIG_STATIC2    0x0B

// User Bank 2
#define UB2_REG_ACCEL_CONFIG_STATIC2   0x03

// Default WHO_AM_I value
#define ICM42605_WHO_AM_I_VAL          0x42

#endif // ICM42605_REGISTERS_H_