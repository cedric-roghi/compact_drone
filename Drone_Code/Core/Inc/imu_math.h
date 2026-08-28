#ifndef IMU_MATH_H_
#define IMU_MATH_H_

#include <math.h>

#define RAD_TO_DEG 57.295779513082320876798154814105f
#define DEG_TO_RAD 0.017453292519943295769236907684886f

typedef struct {
    float roll;   // degrees (X-axis rotation)
    float pitch;  // degrees (Y-axis rotation)
    float yaw;    // degrees (Z-axis rotation, Gyro integration only)
    float alpha;  // Complementary filter weight (e.g., 0.98f)
} IMU_Attitude_t;

void imu_math_init(IMU_Attitude_t *att, float filter_alpha);
void imu_math_update(IMU_Attitude_t *att, float acc_x, float acc_y, float acc_z, 
                     float gyro_x, float gyro_y, float gyro_z, float dt);

#endif // IMU_MATH_H_