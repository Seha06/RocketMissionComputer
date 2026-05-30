/*
 * Rocket Dead Reckoning & Apogee Detection — Top-Level Integration
 *
 * Target: STM32F401RCT6 @ 84 MHz, ARM Cortex-M4 FPU
 *
 * Execution model:
 *   - TIM2 fires at 208 Hz (LSM6DSM ODR) → RocketDR_TimerA_ISR() sets imu_a_flag
 *   - TIM3 fires at 100 Hz (MPU-6050 ODR) → sets imu_b_flag
 *   - main() polls flags and runs the detection pipeline
 *
 * This file uses platform stubs (marked PLATFORM_*) that the integrator
 * must replace with real STM32 HAL calls.
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

static uint32_t platform_tick_ms(void)        { return 0; /* HAL_GetTick(); */ }
static void     platform_delay_ms(uint32_t ms) { (void)ms; /* HAL_Delay(ms); */ }

/*
 * Non-blocking pyro pin control.
 * The integrator maps these to GPIO set/clear — no HAL_Delay inside.
 * The 500 ms pulse width is managed by the main loop timer below.
 */
static void platform_pyro_pin_set(bool active)
{
    if (active) {
        /* HAL_GPIO_WritePin(PYRO1_GPIO, PYRO1_PIN, GPIO_PIN_SET);   */
    } else {
        /* HAL_GPIO_WritePin(PYRO1_GPIO, PYRO1_PIN, GPIO_PIN_RESET); */
    }
    (void)active;
}

/* Feed the Independent Watchdog. Call at least every IWDG_TIMEOUT_MS. */
static void platform_iwdg_refresh(void)
{
    /* HAL_IWDG_Refresh(&hiwdg); */
}

/*
 * Portable critical-section helpers for Cortex-M4.
 * On M4, reading/writing a single aligned 32-bit word is atomic at the bus
 * level, but C does not guarantee atomicity for volatile float.  Wrapping
 * the shared float accesses in a critical section is cheap (2 µs at 84 MHz)
 * and makes the contract explicit for any future port or static analyser.
 */
static inline uint32_t critical_enter(void)
{
    /* uint32_t primask;
     * __asm volatile ("MRS %0, PRIMASK" : "=r"(primask));
     * __disable_irq();
     * return primask; */
    return 0u;
}
static inline void critical_exit(uint32_t primask)
{
    /* if (!primask) __enable_irq(); */
    (void)primask;
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

/*
 * Last vertical acceleration from each IMU — written by process_imu_x(),
 * read by the FSM block in the main loop.  Declared volatile because both
 * the ISR-triggered processing function and the main-loop FSM block access
 * them in separate passes through the loop.
 */
static volatile float last_a_vert_a = 0.0f;
static volatile float last_a_vert_b = 0.0f;

/*
 * IMU-ready flags set by timer ISR.
 * Declared volatile uint8_t (not bool) so that the clear in the main loop
 * compiles to a single STR instruction on Cortex-M4, making it atomic
 * with respect to the ISR's single-instruction set.
 */
static volatile uint8_t imu_a_flag = 0u;
static volatile uint8_t imu_b_flag = 0u;

/* Watchdog timestamps */
static uint32_t last_imu_a_ms = 0u;
static uint32_t last_imu_b_ms = 0u;

/* Pyro pulse: non-blocking 500 ms drive */
#define PYRO_PULSE_MS       500u
static bool     apogee_charge_fired = false;
static bool     pyro_active         = false;
static uint32_t pyro_start_ms       = 0u;

/* =========================================================================
 * IMU reading plausibility limits
 * Maximum credible values for ±16 g / ±2000 dps sensors.
 * A sample outside these ranges indicates a sensor fault or SEU.
 * ========================================================================= */
#define IMU_ACCEL_MAX_MS2   (18.0f * GRAVITY_MS2)   /* 18 g */
#define IMU_GYRO_MAX_RPS    (2100.0f * 0.017453f)    /* 2100 dps in rad/s */

static bool imu_sample_sane(const IMU_Data_t *d)
{
    for (int i = 0; i < 3; i++) {
        if (d->a[i] >  IMU_ACCEL_MAX_MS2 || d->a[i] < -IMU_ACCEL_MAX_MS2) return false;
        if (d->g[i] >  IMU_GYRO_MAX_RPS  || d->g[i] < -IMU_GYRO_MAX_RPS)  return false;
    }
    return true;
}

/* =========================================================================
 * Timer ISR callbacks — call from TIM2 and TIM3 interrupt handlers
 * ========================================================================= */
void RocketDR_TimerA_ISR(void)   /* 208 Hz */
{
    imu_a_flag = 1u;
    /* __DSB() ensures the store is visible to main loop before ISR returns.
     * Without a barrier, the compiler may reorder this write past other
     * volatile accesses at -O2 (AAPCS §B.8, MISRA-C:2012 Dir 4.1). */
    /* __DSB(); */
}

void RocketDR_TimerB_ISR(void)   /* 100 Hz */
{
    imu_b_flag = 1u;
    /* __DSB(); */
}

/* =========================================================================
 * Calibration helper
 * ========================================================================= */
static void run_calibration(void)
{
    IMU_Data_t d;
    uint16_t i;
    uint16_t valid_a = 0u, valid_b = 0u;

    /* Collect CALIB_SAMPLE_COUNT samples at 208 Hz (~5 s) */
    for (i = 0; i < CALIB_SAMPLE_COUNT; i++) {
        while (!imu_a_flag) { platform_iwdg_refresh(); }
        imu_a_flag = 0u;

        if (LSM6DSM_Read(&lsm6dsm_hal, &d) && imu_sample_sane(&d)) {
            calib_buf_a[i] = d.a[ROCKET_ACCEL_AXIS] * ROCKET_ACCEL_SIGN;
            valid_a++;
        } else {
            calib_buf_a[i] = GRAVITY_MS2;
        }
    }

    /*
     * Require at least 90% valid samples.  Fewer implies sensor init failure
     * or intermittent SPI — do not proceed.  We call platform_fault_handler()
     * rather than spinning: the caller must log/LED/safe-state the system.
     * A rocket with a dead primary IMU must not be permitted to launch.
     *
     * (The previous implementation used while(valid_a < threshold) with no
     *  body that could change valid_a — guaranteed infinite loop on fault.)
     */
    if (valid_a < (CALIB_SAMPLE_COUNT * 9u / 10u)) {
        /* platform_fault_handler(FAULT_IMU_A_CALIB); */
        while (1) { platform_iwdg_refresh(); }  /* halt — operator must power-cycle */
    }

    DR_CalibrateBias(&dr_a, calib_buf_a, CALIB_SAMPLE_COUNT);

    /* Calibrate IMU-B at 100 Hz */
    for (i = 0; i < (CALIB_SAMPLE_COUNT / 2u); i++) {
        while (!imu_b_flag) { platform_iwdg_refresh(); }
        imu_b_flag = 0u;

        if (MPU6050_Read(&mpu6050_hal, &d) && imu_sample_sane(&d)) {
            calib_buf_b[i] = d.a[ROCKET_ACCEL_AXIS] * ROCKET_ACCEL_SIGN;
            valid_b++;
        } else {
            calib_buf_b[i] = GRAVITY_MS2;
        }
    }

    /* IMU-B is backup; 75% threshold acceptable */
    if (valid_b < ((CALIB_SAMPLE_COUNT / 2u) * 3u / 4u)) {
        /* platform_fault_handler(FAULT_IMU_B_CALIB); */
        /* IMU-B failure is non-fatal: continue with A-only, mark B failed */
        apg_b.imu_failed = true;
    }

    DR_CalibrateBias(&dr_b, calib_buf_b, CALIB_SAMPLE_COUNT / 2u);
}

/* =========================================================================
 * Process one IMU-A sample (208 Hz)
 * ========================================================================= */
static void process_imu_a(uint32_t now_ms)
{
    IMU_Data_t d;
    if (!LSM6DSM_Read(&lsm6dsm_hal, &d) || !imu_sample_sane(&d)) {
        if ((now_ms - last_imu_a_ms) > IMU_WATCHDOG_MS)
            apg_a.imu_failed = true;
        return;
    }

    last_imu_a_ms    = now_ms;
    apg_a.imu_failed = false;

    float a_vert     = d.a[ROCKET_ACCEL_AXIS]        * ROCKET_ACCEL_SIGN;
    float gyro_pitch = d.g[ROCKET_PITCH_GYRO_AXIS]   * ROCKET_PITCH_GYRO_SIGN;

    /* Store for FSM.  Use critical section: C standard does not guarantee
     * that a float write is atomic, even on Cortex-M4 with FPU.  The
     * section is entered/exited in both writer (here) and reader (main loop)
     * so any future port to Cortex-M0 or a multi-core target stays safe. */
    {
        uint32_t ps = critical_enter();
        last_a_vert_a = a_vert;
        critical_exit(ps);
    }

    ATT_Update(&att_a, d.a, gyro_pitch, IMU_A_DT_S);
    float pitch = ATT_GetPitchRad(&att_a);

    DR_Predict(&dr_a, a_vert, pitch, IMU_A_DT_S);

    if (fsm.phase == PHASE_COAST)
        APOGEE_Update(&apg_a, DR_GetVelocityRaw(&dr_a), fsm.phase, now_ms);
}

/* =========================================================================
 * Process one IMU-B sample (100 Hz)
 * ========================================================================= */
static void process_imu_b(uint32_t now_ms)
{
    IMU_Data_t d;
    if (!MPU6050_Read(&mpu6050_hal, &d) || !imu_sample_sane(&d)) {
        if ((now_ms - last_imu_b_ms) > IMU_WATCHDOG_MS)
            apg_b.imu_failed = true;
        return;
    }

    last_imu_b_ms    = now_ms;
    apg_b.imu_failed = false;

    float a_vert     = d.a[ROCKET_ACCEL_AXIS]        * ROCKET_ACCEL_SIGN;
    float gyro_pitch = d.g[ROCKET_PITCH_GYRO_AXIS]   * ROCKET_PITCH_GYRO_SIGN;

    {
        uint32_t ps = critical_enter();
        last_a_vert_b = a_vert;
        critical_exit(ps);
    }

    ATT_Update(&att_b, d.a, gyro_pitch, IMU_B_DT_S);
    float pitch = ATT_GetPitchRad(&att_b);

    DR_Predict(&dr_b, a_vert, pitch, IMU_B_DT_S);

    if (fsm.phase == PHASE_COAST)
        APOGEE_Update(&apg_b, DR_GetVelocityRaw(&dr_b), fsm.phase, now_ms);
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

        /* Feed hardware watchdog — must happen every loop iteration.
         * If this stops (e.g. stuck in an ISR or infinite loop), the IWDG
         * will reset the MCU.  After reset the rocket re-calibrates, which
         * is acceptable pre-launch; in flight the system restarts cleanly. */
        platform_iwdg_refresh();

        /* IMU-A task (208 Hz) */
        if (imu_a_flag) {
            imu_a_flag = 0u;   /* clear before read — single STR, atomic on M4 */
            process_imu_a(now_ms);
        }

        /* IMU-B task (100 Hz) */
        if (imu_b_flag) {
            imu_b_flag = 0u;
            process_imu_b(now_ms);
        }

        /* --- Flight FSM update (use IMU-A as primary) ------------------
         *
         * last_a_vert_a is written by process_imu_a() in this same loop
         * iteration (or the previous one).  We read it here — after the
         * IMU tasks — so it always reflects the most recent sample.
         *
         * Previously this block used a_vert = 0.0f, which meant the FSM
         * permanently saw a_net ≈ −g and never left PHASE_PAD_STATIC.
         * The rocket would never detect launch and the pyro would never
         * fire.
         */
        {
            uint32_t ps  = critical_enter();
        float a_vert = last_a_vert_a;
        critical_exit(ps);
            float pitch  = ATT_GetPitchRad(&att_a);
            float a_net_a = a_vert * cosf(pitch) - GRAVITY_MS2 - dr_a.x[2];

            bool apogee_vote = APOGEE_Vote(&apg_a, &apg_b);

            FlightPhase_t prev_phase = fsm.phase;
            FlightPhase_t phase = FSM_Update(&fsm,
                                             a_net_a,
                                             DR_GetVelocityRaw(&dr_a),
                                             apogee_vote,
                                             now_ms);

            /* Notify apogee detectors on BOOST → COAST transition */
            if (prev_phase != PHASE_COAST && phase == PHASE_COAST) {
                APOGEE_OnCoastEntry(&apg_a, now_ms);
                APOGEE_OnCoastEntry(&apg_b, now_ms);
            }
        }

        /* --- Non-blocking pyro pulse management ------------------------
         *
         * Firing is split into "start" and "stop" edges so that the main
         * loop is never stalled.  The original implementation called
         * HAL_Delay(500) inside platform_fire_apogee_charge(), blocking
         * all IMU processing, watchdog feeds, and FSM updates for 500 ms
         * immediately after apogee — the most time-critical moment.
         */
        if (fsm.phase == PHASE_APOGEE && !apogee_charge_fired) {
            apogee_charge_fired = true;
            pyro_active         = true;
            pyro_start_ms       = now_ms;
            platform_pyro_pin_set(true);
        }

        if (pyro_active && (now_ms - pyro_start_ms) >= PYRO_PULSE_MS) {
            pyro_active = false;
            platform_pyro_pin_set(false);
        }
    }
}
