#include "imu_math.h"

void imu_math_init(IMU_Attitude_t *att, float filter_alpha) {
    att->roll = 0.0f;
    att->pitch = 0.0f;
    att->yaw = 0.0f;
    att->alpha = filter_alpha;
}

void imu_math_update(IMU_Attitude_t *att, float acc_x, float acc_y, float acc_z, 
                     float gyro_x, float gyro_y, float gyro_z, 
                     float mag_x, float mag_y, float mag_z, float dt) {
    // 1. Roll and Pitch from Accelerometer
    float roll_acc = atan2f(acc_y, acc_z) * RAD_TO_DEG;
    float pitch_acc = atan2f(-acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z)) * RAD_TO_DEG;

    att->roll = att->alpha * (att->roll + gyro_x * dt) + (1.0f - att->alpha) * roll_acc;
    att->pitch = att->alpha * (att->pitch + gyro_y * dt) + (1.0f - att->alpha) * pitch_acc;

    // 2. Convert filtered angles to radians for magnetometer tilt compensation
    float roll_rad = att->roll * DEG_TO_RAD;
    float pitch_rad = att->pitch * DEG_TO_RAD;

    // 3. Tilt-compensated magnetic heading (Requires axis alignment with IMU!)
    float mag_x_comp = mag_x * cosf(pitch_rad) + mag_z * sinf(pitch_rad);
    float mag_y_comp = mag_x * sinf(roll_rad) * sinf(pitch_rad) + mag_y * cosf(roll_rad) - mag_z * sinf(roll_rad) * cosf(pitch_rad);
    
    float mag_heading = atan2f(-mag_y_comp, mag_x_comp) * RAD_TO_DEG;

    // 4. Complementary filter for Yaw (fuse high-frequency gyro with low-frequency mag)
    att->yaw = att->alpha * (att->yaw + gyro_z * dt) + (1.0f - att->alpha) * mag_heading;
}