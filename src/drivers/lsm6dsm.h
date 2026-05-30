#ifndef LSM6DSM_H
#define LSM6DSM_H

#include <stdint.h>
#include <stdbool.h>

/* LSM6DSM register addresses */
#define LSM6DSM_REG_WHO_AM_I    0x0F
#define LSM6DSM_REG_CTRL1_XL    0x10
#define LSM6DSM_REG_CTRL2_G     0x11
#define LSM6DSM_REG_CTRL3_C     0x12
#define LSM6DSM_REG_CTRL8_XL    0x17
#define LSM6DSM_REG_STATUS      0x1E
#define LSM6DSM_REG_OUTX_L_G    0x22
#define LSM6DSM_REG_OUTX_L_XL   0x28

#define LSM6DSM_WHO_AM_I_VAL    0x6A

/* Sensitivity: ±16 g, ±2000 dps */
#define LSM6DSM_ACCEL_SENS      (0.000488f * 9.80665f)  /* LSB → m/s², ±16g range */
#define LSM6DSM_GYRO_SENS       (0.070f * 0.017453293f) /* LSB → rad/s, ±2000 dps */

typedef struct {
    float a[3];              /* m/s², body frame [x,y,z] */
    float g[3];              /* rad/s, body frame [x,y,z] */
    uint32_t timestamp_ms;
    bool valid;
} IMU_Data_t;

/* HAL abstraction — platform fills these function pointers */
typedef struct {
    void     (*cs_assert)(void);
    void     (*cs_release)(void);
    void     (*spi_exchange)(const uint8_t *tx, uint8_t *rx, uint16_t len);
    uint32_t (*tick_ms)(void);
    void     (*delay_ms)(uint32_t ms);
} LSM6DSM_HAL_t;

bool LSM6DSM_Init(const LSM6DSM_HAL_t *hal);
bool LSM6DSM_Read(const LSM6DSM_HAL_t *hal, IMU_Data_t *out);

#endif /* LSM6DSM_H */
