#include "lsm6dsm.h"
#include <string.h>

static void reg_write(const LSM6DSM_HAL_t *hal, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7Fu), val };
    uint8_t rx[2];
    hal->cs_assert();
    hal->spi_exchange(tx, rx, 2);
    hal->cs_release();
}

static uint8_t reg_read(const LSM6DSM_HAL_t *hal, uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(reg | 0x80u), 0x00u };
    uint8_t rx[2] = {0, 0};
    hal->cs_assert();
    hal->spi_exchange(tx, rx, 2);
    hal->cs_release();
    return rx[1];
}

static void burst_read(const LSM6DSM_HAL_t *hal, uint8_t reg,
                       uint8_t *buf, uint8_t len)
{
    /* 1 command byte + len data bytes */
    uint8_t tx[13] = { (uint8_t)(reg | 0x80u) };
    uint8_t rx[13];
    if (len > 12u) len = 12u;
    hal->cs_assert();
    hal->spi_exchange(tx, rx, (uint16_t)(len + 1u));
    hal->cs_release();
    memcpy(buf, &rx[1], len);
}

bool LSM6DSM_Init(const LSM6DSM_HAL_t *hal)
{
    if (reg_read(hal, LSM6DSM_REG_WHO_AM_I) != LSM6DSM_WHO_AM_I_VAL)
        return false;

    /* Software reset */
    reg_write(hal, LSM6DSM_REG_CTRL3_C, 0x01u);
    hal->delay_ms(2u);

    /*
     * CTRL1_XL: ODR=208 Hz (bits[7:4]=0101), FS=±16g (bits[3:2]=11),
     *           anti-aliasing=400 Hz (bits[1:0]=00)
     */
    reg_write(hal, LSM6DSM_REG_CTRL1_XL, 0x5Cu);

    /*
     * CTRL2_G:  ODR=208 Hz (bits[7:4]=0101), FS=±2000 dps (bits[3:2]=11)
     */
    reg_write(hal, LSM6DSM_REG_CTRL2_G, 0x5Cu);

    /*
     * CTRL3_C:  BDU=1 (block data update), IF_INC=1 (address auto-increment)
     */
    reg_write(hal, LSM6DSM_REG_CTRL3_C, 0x44u);

    /*
     * CTRL8_XL: composite LPF2 enabled, cutoff ODR/4 ≈ 52 Hz
     * Filters motor vibration (typically 50-300 Hz) before velocity integration.
     */
    reg_write(hal, LSM6DSM_REG_CTRL8_XL, 0x09u);

    return true;
}

bool LSM6DSM_Read(const LSM6DSM_HAL_t *hal, IMU_Data_t *out)
{
    uint8_t status = reg_read(hal, LSM6DSM_REG_STATUS);
    /* Bit 0: accel data ready, bit 1: gyro data ready */
    if ((status & 0x03u) != 0x03u) {
        out->valid = false;
        return false;
    }

    /* Burst read: 6 gyro bytes + 6 accel bytes = 12 bytes starting at OUTX_L_G */
    uint8_t raw[12];
    burst_read(hal, LSM6DSM_REG_OUTX_L_G, raw, 12u);

    int16_t gx = (int16_t)((uint16_t)raw[1]  << 8 | raw[0]);
    int16_t gy = (int16_t)((uint16_t)raw[3]  << 8 | raw[2]);
    int16_t gz = (int16_t)((uint16_t)raw[5]  << 8 | raw[4]);
    int16_t ax = (int16_t)((uint16_t)raw[7]  << 8 | raw[6]);
    int16_t ay = (int16_t)((uint16_t)raw[9]  << 8 | raw[8]);
    int16_t az = (int16_t)((uint16_t)raw[11] << 8 | raw[10]);

    out->gx = (float)gx * LSM6DSM_GYRO_SENS;
    out->gy = (float)gy * LSM6DSM_GYRO_SENS;
    out->gz = (float)gz * LSM6DSM_GYRO_SENS;
    out->ax = (float)ax * LSM6DSM_ACCEL_SENS;
    out->ay = (float)ay * LSM6DSM_ACCEL_SENS;
    out->az = (float)az * LSM6DSM_ACCEL_SENS;
    out->timestamp_ms = hal->tick_ms();
    out->valid = true;

    return true;
}
