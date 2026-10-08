#include "mmc5603nj.h"

#define REG_XOUT0       0x00
#define REG_STATUS1     0x18
#define REG_ODR         0x1A
#define REG_CTRL0       0x1B
#define REG_CTRL1       0x1C
#define REG_CTRL2       0x1D

int mmc5603nj_init(MMC5603NJ_t *dev, I2C_HandleTypeDef *hi2c)
{
    dev->hi2c = hi2c;
    uint8_t data;

    // 1. Software Reset
    data = 0x80; // SW_RST
    if (HAL_I2C_Mem_Write(dev->hi2c, MMC5603NJ_I2C_ADDR, REG_CTRL1, I2C_MEMADD_SIZE_8BIT, &data, 1, 100) != HAL_OK)
    {
        return -1;
    }
    HAL_Delay(20); // Wait for reset to complete

    // 2. Perform a SET operation to magnetize the sensing film
    data = 0x08; // Set bit
    if (HAL_I2C_Mem_Write(dev->hi2c, MMC5603NJ_I2C_ADDR, REG_CTRL0, I2C_MEMADD_SIZE_8BIT, &data, 1, 100) != HAL_OK)
    {
        return -2;
    }
    HAL_Delay(10); // Allow SET pulse to finish

    // Set BW = 01 (3.5ms measurement) so 100 Hz ODR is legal with Auto_SR enabled
    data = 0x01; // BW1:BW0 = 01
    HAL_I2C_Mem_Write(dev->hi2c, MMC5603NJ_I2C_ADDR, REG_CTRL1, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    // 3. Set Output Data Rate (ODR) to 100 Hz
    data = 100; // 0x64
    if (HAL_I2C_Mem_Write(dev->hi2c, MMC5603NJ_I2C_ADDR, REG_ODR, I2C_MEMADD_SIZE_8BIT, &data, 1, 100) != HAL_OK)
    {
        return -3;
    }

    // 4. Enable Auto Set/Reset and Continuous Measurement Mode
    data = 0xA0; // Bit 7: CMM_en, Bit 5: Auto_SR_en
    if (HAL_I2C_Mem_Write(dev->hi2c, MMC5603NJ_I2C_ADDR, REG_CTRL0, I2C_MEMADD_SIZE_8BIT, &data, 1, 100) != HAL_OK)
    {
        return -4;
    }

    data = 0x10; // Cmm_en (bit 4 of Internal Control 2)
    HAL_I2C_Mem_Write(dev->hi2c, MMC5603NJ_I2C_ADDR, REG_CTRL2, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    return 1; // Success
}

int mmc5603nj_read(MMC5603NJ_t *dev) {
    uint8_t buf[9];
    
    // Read 9 bytes starting from XOUT0 (0x00) through ZOUT2/Status (0x08)
    if (HAL_I2C_Mem_Read(dev->hi2c, MMC5603NJ_I2C_ADDR, REG_XOUT0, I2C_MEMADD_SIZE_8BIT, buf, 9, 100) != HAL_OK) {
        return -1;
    }

    // Parse the 20-bit data format per the MMC5603NJ datasheet
    uint32_t raw_x = ((uint32_t)buf[0] << 12) | ((uint32_t)buf[1] << 4) | (buf[6] & 0x0F);
    uint32_t raw_y = ((uint32_t)buf[2] << 12) | ((uint32_t)buf[3] << 4) | (buf[7] & 0x0F);
    uint32_t raw_z = ((uint32_t)buf[4] << 12) | ((uint32_t)buf[5] << 4) | (buf[8] & 0x0F);

    // Normalize values
    // The device has a 20-bit span, so the zero-field null point is at 2^19 (524288).
    // The typical sensitivity scale factor is ~16384 LSB per Gauss.
    float raw_mag_x = (float)((int32_t)raw_x - 524288) / 16000.0f;
    float raw_mag_y = (float)((int32_t)raw_y - 524288) / 16000.0f;
    float raw_mag_z = (float)((int32_t)raw_z - 524288) / 16000.0f;

    // Remap axes to match the upright board mounting orientation
    dev->mag_x = raw_mag_z;
    dev->mag_y = raw_mag_y;
    dev->mag_z = -raw_mag_x;

    return 1;
}