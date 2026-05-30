/*
 * Rocket Dead Reckoning & Apogee Detection — Top-Level Integration
 *
 * Target: STM32F401RCT6 @ 84 MHz, ARM Cortex-M4 FPU
 *
 * Execution model:
 *   - TIM2 fires at 208 Hz (LSM6DSM ODR) → timer_isr() sets imu_a_flag
 *   - TIM3 fires at 100 Hz (MPU-6050 ODR) → sets imu_b_flag
 *   - main() polls flags and runs the detection pipeline
 *
 * This file uses platform stubs (marked PLATFORM_*) that the integrator
 * must implement for the specific STM32 HAL / RTOS configuration.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "ekf_config.h"
#include "flight_fsm.h"
#include "drivers/lsm6dsm.h"
#include "drivers/mpu6050.h"
#include "nav/attitude.h"
#include "nav/dead_reckoning.h"
#include "nav/apogee_detector.h"

/* =========================================================================
 * PLATFORM STUBS — replace with real HAL calls
 * ========================================================================= */

/* SPI for LSM6DSM */
static void platform_lsm6dsm_cs_assert(void)  { /* HAL_GPIO_WritePin(CS_GPIO, CS_PIN, GPIO_PIN_RESET); */ }
static void platform_lsm6dsm_cs_release(void) { /* HAL_GPIO_WritePin(CS_GPIO, CS_PIN, GPIO_PIN_SET);   */ }
static void platform_lsm6dsm_spi_xchg(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    /* HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)tx, rx, len, 10); */
    (void)tx; (void)rx; (void)len;
}

/* I2C for MPU-6050 */
static void platform_i2c_write(uint8_t addr, uint8_t reg,
                                const uint8_t *data, uint8_t len)
{
    /* HAL_I2C_Mem_Write(&hi2c1, addr<<1, reg, 1, (uint8_t*)data, len, 10); */
    (void)addr; (void)reg; (void)data; (void)len;
}
static void platform_i2c_read(uint8_t addr, uint8_t reg,
                               uint8_t *data, uint8_t len)
{
    /* HAL_I2C_Mem_Read(&hi2c1, addr<<1, reg, 1, data, len, 10); */
    (void)addr; (void)reg; (void)data; (void)len;
}

static uint32_t platform_tick_ms(void)   { return 0; /* HAL_GetTick(); */ }
static void     platform_delay_ms(uint32_t ms) { (void)ms; /* HAL_Delay(ms); */ }

/* Pyro channel fire: drive GPIO high for 500 ms */
static void platform_fire_apogee_charge(void)
{
    /* HAL_GPIO_WritePin(PYRO1_GPIO, PYRO1_PIN, GPIO_PIN_SET);  */
    /* HAL_Delay(500);                                            */
    /* HAL_GPIO_WritePin(PYRO1_GPIO, PYRO1_PIN, GPIO_PIN_RESET); */
}

/* =========================================================================
 * Hardware abstraction wiring
 * ========================================================================= */
static const LSM6DSM_HAL_t lsm6dsm_hal = {
    .cs_assert    = platform_lsm6dsm_cs_assert,
    .cs_release   = platform_lsm6dsm_cs_release,
    .spi_exchange = platform_lsm6dsm_spi_xchg,
    .tick_ms      = platform_tick_ms,
    .delay_ms     = platform_delay_ms,
};

static const MPU6050_HAL_t mpu6050_hal = {
    .i2c_write = platform_i2c_write,
    .i2c_read  = platform_i2c_read,
    .tick_ms   = platform_tick_ms,
    .delay_ms  = platform_delay_ms,
};

/* =========================================================================
 * Module instances (static allocation — no heap)
 * ========================================================================= */
static FSM_State_t       fsm;
static Attitude_t        att_a, att_b;
static DR_State_t        dr_a,  dr_b;
static ApogeeDetector_t  apg_a, apg_b;

/* Calibration sample buffers */
static float calib_buf_a[CALIB_SAMPLE_COUNT];
static float calib_buf_b[CALIB_SAMPLE_COUNT];

/* IMU-ready flags set by timer ISR */
static volatile bool imu_a_flag = false;
static volatile bool imu_b_flag = false;

/* Watchdog timestamps */
static uint32_t last_imu_a_ms = 0;
static uint32_t last_imu_b_ms = 0;

/* Apogee fired flag (edge-triggered to call pyro only once) */
static bool apogee_charge_fired = false;

/* =========================================================================
 * Timer ISR callbacks — call from TIM2 and TIM3 interrupt handlers
 * ========================================================================= */
void RocketDR_TimerA_ISR(void)   /* 208 Hz */
{
    imu_a_flag = true;
}

void RocketDR_TimerB_ISR(void)   /* 100 Hz */
{
    imu_b_flag = true;
}

/* =========================================================================
 * Calibration helper
 * ========================================================================= */
static void run_calibration(void)
{
    IMU_Data_t d;
    uint16_t i;

    /* Collect CALIB_SAMPLE_COUNT samples at ~208 Hz (~1 second) */
    for (i = 0; i < CALIB_SAMPLE_COUNT; i++) {
        /* Wait for next IMU-A sample */
        while (!imu_a_flag) {}
        imu_a_flag = false;

        LSM6DSM_Read(&lsm6dsm_hal, &d);
        if (d.valid) {
            float *a = &d.ax;
            calib_buf_a[i] = a[ROCKET_ACCEL_AXIS] * ROCKET_ACCEL_SIGN;
        } else {
            calib_buf_a[i] = GRAVITY_MS2;  /* fallback if read fails */
        }
    }

    DR_CalibrateBias(&dr_a, calib_buf_a, CALIB_SAMPLE_COUNT);

    /* Calibrate IMU-B using MPU-6050 at its 100 Hz rate */
    for (i = 0; i < (CALIB_SAMPLE_COUNT / 2); i++) {
        while (!imu_b_flag) {}
        imu_b_flag = false;

        MPU6050_Read(&mpu6050_hal, &d);
        if (d.valid) {
            float *a = &d.ax;
            calib_buf_b[i] = a[ROCKET_ACCEL_AXIS] * ROCKET_ACCEL_SIGN;
        } else {
            calib_buf_b[i] = GRAVITY_MS2;
        }
    }

    DR_CalibrateBias(&dr_b, calib_buf_b, CALIB_SAMPLE_COUNT / 2);
}

/* =========================================================================
 * Process one IMU-A sample (208 Hz)
 * ========================================================================= */
static void process_imu_a(uint32_t now_ms)
{
    IMU_Data_t d;
    if (!LSM6DSM_Read(&lsm6dsm_hal, &d)) {
        /* Check watchdog */
        if ((now_ms - last_imu_a_ms) > IMU_WATCHDOG_MS)
            apg_a.imu_failed = true;
        return;
    }

    last_imu_a_ms    = now_ms;
    apg_a.imu_failed = false;

    float *a = &d.ax;
    float a_vert   = a[ROCKET_ACCEL_AXIS]           * ROCKET_ACCEL_SIGN;
    float *g       = &d.gx;
    float gyro_pitch = g[ROCKET_PITCH_GYRO_AXIS]    * ROCKET_PITCH_GYRO_SIGN;

    ATT_Update(&att_a, &d.ax, gyro_pitch, IMU_A_DT_S);
    float pitch = ATT_GetPitchRad(&att_a);

    DR_Predict(&dr_a, a_vert, pitch, IMU_A_DT_S);

    float vel  = DR_GetVelocity(&dr_a);
    float a_net = a_vert * cosf(pitch) - GRAVITY_MS2 - dr_a.x[2];

    if (fsm.phase == PHASE_COAST)
        APOGEE_Update(&apg_a, vel, a_net, fsm.phase, now_ms);
}

/* =========================================================================
 * Process one IMU-B sample (100 Hz)
 * ========================================================================= */
static void process_imu_b(uint32_t now_ms)
{
    IMU_Data_t d;
    if (!MPU6050_Read(&mpu6050_hal, &d)) {
        if ((now_ms - last_imu_b_ms) > IMU_WATCHDOG_MS)
            apg_b.imu_failed = true;
        return;
    }

    last_imu_b_ms    = now_ms;
    apg_b.imu_failed = false;

    float *a = &d.ax;
    float a_vert     = a[ROCKET_ACCEL_AXIS]         * ROCKET_ACCEL_SIGN;
    float *g         = &d.gx;
    float gyro_pitch = g[ROCKET_PITCH_GYRO_AXIS]    * ROCKET_PITCH_GYRO_SIGN;

    ATT_Update(&att_b, &d.ax, gyro_pitch, IMU_B_DT_S);
    float pitch = ATT_GetPitchRad(&att_b);

    DR_Predict(&dr_b, a_vert, pitch, IMU_B_DT_S);

    float vel   = DR_GetVelocity(&dr_b);
    float a_net = a_vert * cosf(pitch) - GRAVITY_MS2 - dr_b.x[2];

    if (fsm.phase == PHASE_COAST)
        APOGEE_Update(&apg_b, vel, a_net, fsm.phase, now_ms);
}

/* =========================================================================
 * Application entry point
 * ========================================================================= */
void RocketDR_Main(void)
{
    /* --- Initialise modules -------------------------------------------- */
    FSM_Init(&fsm);
    ATT_Init(&att_a);
    ATT_Init(&att_b);
    DR_Init(&dr_a, IMU_A_DT_S);
    DR_Init(&dr_b, IMU_B_DT_S);
    APOGEE_Init(&apg_a);
    APOGEE_Init(&apg_b);

    /* --- Sensor init --------------------------------------------------- */
    LSM6DSM_Init(&lsm6dsm_hal);
    MPU6050_Init(&mpu6050_hal);

    /* --- Calibration on pad (rocket must be static) -------------------- */
    run_calibration();

    /* --- Main loop ----------------------------------------------------- */
    while (1)
    {
        uint32_t now_ms = platform_tick_ms();

        /* IMU-A task (208 Hz) */
        if (imu_a_flag) {
            imu_a_flag = false;
            process_imu_a(now_ms);
        }

        /* IMU-B task (100 Hz) */
        if (imu_b_flag) {
            imu_b_flag = false;
            process_imu_b(now_ms);
        }

        /* --- Flight FSM update (use IMU-A as primary) ------------------ */
        float a_net_a = 0.0f;
        {
            float a_vert = 0.0f; /* would come from last IMU-A read */
            float pitch  = ATT_GetPitchRad(&att_a);
            a_net_a = a_vert * cosf(pitch) - GRAVITY_MS2 - dr_a.x[2];
        }

        bool apogee_vote = APOGEE_Vote(&apg_a, &apg_b);

        FlightPhase_t prev_phase = fsm.phase;
        FlightPhase_t phase = FSM_Update(&fsm,
                                         a_net_a,
                                         DR_GetVelocity(&dr_a),
                                         apogee_vote,
                                         now_ms);

        /* Notify apogee detectors when COAST is entered */
        if (prev_phase != PHASE_COAST && phase == PHASE_COAST) {
            APOGEE_OnCoastEntry(&apg_a, now_ms);
            APOGEE_OnCoastEntry(&apg_b, now_ms);
        }

        /* --- Fire apogee charge (single shot) -------------------------- */
        if (phase == PHASE_APOGEE && !apogee_charge_fired) {
            apogee_charge_fired = true;
            platform_fire_apogee_charge();
        }
    }
}
