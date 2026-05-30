#include "mpu6050.h"

static void reg_write(const MPU6050_HAL_t *hal, uint8_t reg, uint8_t val)
{
    hal->i2c_write(MPU6050_ADDR, reg, &val, 1u);
}

static uint8_t reg_read(const MPU6050_HAL_t *hal, uint8_t reg)
{
    uint8_t val = 0;
    hal->i2c_read(MPU6050_ADDR, reg, &val, 1u);
    return val;
}

bool MPU6050_Init(const MPU6050_HAL_t *hal)
{
    /*
     * Wake from sleep, select PLL with X-axis gyroscope reference (CLKSEL=1).
     * The internal 8 MHz RC oscillator has ±10% accuracy over temperature,
     * which translates to a ±10% ODR error — i.e. the actual dt used in
     * integration could be off by 1 ms at 100 Hz. The gyro PLL is stable
     * to ±1% and is the datasheet-recommended source.
     */
    reg_write(hal, MPU6050_REG_PWR_MGMT_1, 0x01u);
    hal->delay_ms(10u);

    if (reg_read(hal, MPU6050_REG_WHO_AM_I) != MPU6050_WHO_AM_I_VAL)
        return false;

    /*
     * SMPLRT_DIV: ODR = 1000 / (1 + SMPLRT_DIV) = 100 Hz → SMPLRT_DIV = 9
     * CONFIG:     DLPF_CFG=2 → accel BW 94 Hz, gyro BW 98 Hz
     * Filters motor vibration before integration.
     */
    reg_write(hal, MPU6050_REG_SMPLRT_DIV, 0x09u);
    reg_write(hal, MPU6050_REG_CONFIG,     0x02u);

    /* Gyro full-scale: ±2000 dps (FS_SEL=3) */
    reg_write(hal, MPU6050_REG_GYRO_CFG,  0x18u);

    /* Accel full-scale: ±16 g (AFS_SEL=3) */
    reg_write(hal, MPU6050_REG_ACCEL_CFG, 0x18u);

    return true;
}

bool MPU6050_Read(const MPU6050_HAL_t *hal, IMU_Data_t *out)
{
    out->valid = false;

    /*
     * Check INT_STATUS DATA_RDY bit before reading.  Without this check the
     * function previously always returned true even when the I2C bus was dead
     * (raw[] would be all-zero or stale), making the watchdog useless.
     */
    if (!(reg_read(hal, MPU6050_REG_INT_STATUS) & MPU6050_DRDY_BIT))
        return false;

    /* 14 bytes: ACCEL_XOUT_H/L × 3, TEMP_H/L, GYRO_XOUT_H/L × 3 */
    uint8_t raw[14] = {0};
    hal->i2c_read(MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT, raw, 14u);

    int16_t ax = (int16_t)((uint16_t)raw[0]  << 8 | raw[1]);
    int16_t ay = (int16_t)((uint16_t)raw[2]  << 8 | raw[3]);
    int16_t az = (int16_t)((uint16_t)raw[4]  << 8 | raw[5]);
    /* raw[6], raw[7] = temperature, skip */
    int16_t gx = (int16_t)((uint16_t)raw[8]  << 8 | raw[9]);
    int16_t gy = (int16_t)((uint16_t)raw[10] << 8 | raw[11]);
    int16_t gz = (int16_t)((uint16_t)raw[12] << 8 | raw[13]);

    out->a[0] = (float)ax * MPU6050_ACCEL_SENS;
    out->a[1] = (float)ay * MPU6050_ACCEL_SENS;
    out->a[2] = (float)az * MPU6050_ACCEL_SENS;
    out->g[0] = (float)gx * MPU6050_GYRO_SENS;
    out->g[1] = (float)gy * MPU6050_GYRO_SENS;
    out->g[2] = (float)gz * MPU6050_GYRO_SENS;
    out->timestamp_ms = hal->tick_ms();
    out->valid = true;

    return true;
}
