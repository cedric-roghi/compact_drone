#include "imu_math.h"

void imu_math_init(IMU_Attitude_t *att, float filter_alpha) {
    att->roll = 0.0f;
    att->pitch = 0.0f;
    att->yaw = 0.0f;
    att->alpha = filter_alpha;
}

void imu_math_update(IMU_Attitude_t *att, float acc_x, float acc_y, float acc_z, 
                     float gyro_x, float gyro_y, float gyro_z, float dt) {
    // 1. Calculate Roll and Pitch angles from Accelerometer (G vectors)
    float roll_acc = atan2f(acc_y, acc_z) * RAD_TO_DEG;
    float pitch_acc = atan2f(-acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z)) * RAD_TO_DEG;

    // 2. Complementary Filter: Fuse Gyro integration with Accelerometer absolute tilt
    att->roll = att->alpha * (att->roll + gyro_x * dt) + (1.0f - att->alpha) * roll_acc;
    att->pitch = att->alpha * (att->pitch + gyro_y * dt) + (1.0f - att->alpha) * pitch_acc;

    // 3. Yaw angle calculated purely via Gyro integration (Requires Magnetometer / GPS for absolute heading)
    att->yaw += gyro_z * dt;
}