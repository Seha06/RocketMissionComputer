#ifndef EKF_CONFIG_H
#define EKF_CONFIG_H

/* IMU sample rates */
#define IMU_A_ODR_HZ            208.0f
#define IMU_A_DT_S              (1.0f / IMU_A_ODR_HZ)
#define IMU_B_ODR_HZ            100.0f
#define IMU_B_DT_S              (1.0f / IMU_B_ODR_HZ)

/*
 * Body-frame axis convention:
 *   ROCKET_ACCEL_AXIS=2 means sensor Z-axis is the rocket's longitudinal axis.
 *   At rest on pad (nose up): accelerometer Z reads +9.81 m/s².
 *   Adjust sign/index if PCB is mounted differently.
 */
#define ROCKET_ACCEL_AXIS       2
#define ROCKET_ACCEL_SIGN       (+1.0f)
#define ROCKET_PITCH_GYRO_AXIS  1
#define ROCKET_PITCH_GYRO_SIGN  (+1.0f)

/* Physical constants */
#define GRAVITY_MS2             9.80665f

/* Pad calibration */
#define CALIB_SAMPLE_COUNT      200

/* Dead reckoning noise */
#define DR_SIGMA_ACCEL          0.05f
#define DR_SIGMA_BIAS_DRIFT     0.0001f

/* Apogee detection
 * Threshold = 0: fire when velocity first turns negative.
 * N_CONSEC=2 @ 208 Hz → max detection delay = 2 × 4.81 ms = 9.6 ms ≤ 10 ms.
 * Velocity noise σ ≈ 0.006 m/s after 3 s coast; threshold is safe against
 * false triggers (coast deceleration = 9.81 m/s² >> noise). */
#define APOGEE_VEL_THRESHOLD    -0.05f
#define APOGEE_N_CONSEC         1
#define VELOCITY_LPF_ALPHA      0.3f   /* for telemetry output only */
#define APOGEE_COAST_MAX_MS     10000U
#define APOGEE_COAST_MIN_MS     500U
#define APOGEE_FREEFALL_THRESH  1.5f
#define APOGEE_FREEFALL_MS      50U

/* Flight phase thresholds */
#define LAUNCH_ACCEL_THRESH     14.7f   /* 1.5g net (requires >2.5g specific force) */
#define LAUNCH_DEBOUNCE_MS      30U
#define BURNOUT_ACCEL_THRESH    2.0f    /* net accel < 0.2g: free fall */
#define BURNOUT_DEBOUNCE_MS     50U

/* IMU watchdog */
#define IMU_WATCHDOG_MS         50U

#endif /* EKF_CONFIG_H */
