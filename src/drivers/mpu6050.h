#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stdbool.h>
#include "lsm6dsm.h"   /* reuse IMU_Data_t */

/* MPU-6050 I2C address (AD0 pin low) */
#define MPU6050_ADDR            0x68u

/* Register addresses */
#define MPU6050_REG_SMPLRT_DIV  0x19
#define MPU6050_REG_CONFIG      0x1A
#define MPU6050_REG_GYRO_CFG    0x1B
#define MPU6050_REG_ACCEL_CFG   0x1C
#define MPU6050_REG_INT_STATUS  0x3A   /* bit0 = DATA_RDY_INT */
#define MPU6050_REG_ACCEL_XOUT  0x3B
#define MPU6050_REG_PWR_MGMT_1  0x6B
#define MPU6050_REG_WHO_AM_I    0x75

#define MPU6050_DRDY_BIT        0x01u

#define MPU6050_WHO_AM_I_VAL    0x68u

/* Sensitivity: ±16 g, ±2000 dps */
#define MPU6050_ACCEL_SENS      (1.0f / 2048.0f * 9.80665f) /* LSB → m/s² */
#define MPU6050_GYRO_SENS       (1.0f / 16.4f * 0.017453f)  /* LSB → rad/s */

typedef struct {
    void     (*i2c_write)(uint8_t addr, uint8_t reg,
                          const uint8_t *data, uint8_t len);
    void     (*i2c_read)(uint8_t addr, uint8_t reg,
                         uint8_t *data, uint8_t len);
    uint32_t (*tick_ms)(void);
    void     (*delay_ms)(uint32_t ms);
} MPU6050_HAL_t;

bool MPU6050_Init(const MPU6050_HAL_t *hal);
bool MPU6050_Read(const MPU6050_HAL_t *hal, IMU_Data_t *out);

#endif /* MPU6050_H */
