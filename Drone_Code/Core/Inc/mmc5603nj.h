#ifndef MMC5603NJ_H
#define MMC5603NJ_H

#include "stm32f4xx_hal.h"

// 7-bit address 0x30 left-shifted for STM32 HAL
#define MMC5603NJ_I2C_ADDR (0x30 << 1)

typedef struct {
    I2C_HandleTypeDef *hi2c;
    float mag_x; // Gauss
    float mag_y; // Gauss
    float mag_z; // Gauss
} MMC5603NJ_t;

int mmc5603nj_init(MMC5603NJ_t *dev, I2C_HandleTypeDef *hi2c);
int mmc5603nj_read(MMC5603NJ_t *dev);

#endif // MMC5603NJ_H