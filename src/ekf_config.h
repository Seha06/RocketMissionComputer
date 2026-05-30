#ifndef EKF_CONFIG_H
#define EKF_CONFIG_H

/* IMU sample rates */
#define IMU_A_ODR_HZ            208.0f
#define IMU_A_DT_S              (1.0f / IMU_A_ODR_HZ)
#define IMU_B_ODR_HZ            100.0f
#define IMU_B_DT_S              (1.0f / IMU_B_ODR_HZ)

/*
 * Body-frame axis convention:
 *   ROCKET_ACCEL_AXIS=2  →  sensor Z aligned with rocket longitudinal axis.
 *   At rest on pad (nose up): accelerometer Z reads +9.81 m/s².
 */
#define ROCKET_ACCEL_AXIS       2
#define ROCKET_ACCEL_SIGN       (+1.0f)
#define ROCKET_PITCH_GYRO_AXIS  1
#define ROCKET_PITCH_GYRO_SIGN  (+1.0f)

/* Physical constants */
#define GRAVITY_MS2             9.80665f
#define DEG_TO_RAD              0.017453293f   /* π/180 — shared, 7 significant figures */

/* -----------------------------------------------------------------------
 * Target mission profile: M2020 motor on 18 kg rocket, ~3000 m apogee
 *   Motor (Cesaroni/AeroTech M2020):
 *     Avg thrust : 2020 N
 *     Burn time  : 4.2 s
 *     Prop mass  : ~1.8 kg
 *   Net boost acceleration: (2020/17.1 - 9.81) ≈ 108 m/s²  (~11g net)
 *   Burnout velocity (with drag, DRAG_K=0.0073): ~362 m/s
 *   Coast time to 3000 m: ~18 s
 *   Total time to apogee: ~22 s
 * ----------------------------------------------------------------------- */

/* Pad calibration
 * 1000 samples @ 208 Hz = 4.8 s of static data.
 * σ_calib = σ_a / √1000 = 0.05/31.6 = 0.00158 m/s²
 * 6σ drift over 20 s flight = 20 × 6 × 0.00158 = 0.19 m/s (within threshold margin) */
#define CALIB_SAMPLE_COUNT      1000

/* Dead reckoning noise */
#define DR_SIGMA_ACCEL          0.05f
#define DR_SIGMA_BIAS_DRIFT     0.0001f

/* -----------------------------------------------------------------------
 * Apogee detection — 100 ms window, 100% success requirement
 *
 * Three independent criteria (any one sufficient):
 *
 *   C1 (PRIMARY, fast):
 *     DR velocity < APOGEE_VEL_THRESHOLD for N_CONSEC consecutive samples.
 *     Threshold -0.25 m/s absorbs worst-case 6σ drift (+0.19 m/s).
 *     Delay: 0.25/9.81 + N_CONSEC×4.81 ms = 25 + 24 = 49 ms ≤ 100 ms ✓
 *
 *   C2 (SAFETY TIMER — absolute hard stop):
 *     Fires if coast phase exceeds APOGEE_COAST_MAX_MS.
 *     Set conservatively above the longest expected coast time.
 *
 *   C3 (SUSTAINED DROP — noise-proof backup):
 *     DR velocity < APOGEE_VEL_SECONDARY for APOGEE_SUSTAINED_MS continuously.
 *     More negative threshold requires sustained drop, immune to noise spikes.
 *     Delay: 0.4/9.81 + SUSTAINED_MS = 41 + 50 = 91 ms ≤ 100 ms ✓
 *
 * NOTE: C3 uses sustained velocity, NOT specific-force (free-fall) detection.
 *       During the ENTIRE coast phase the rocket is in free-fall (a_specific ≈ 0),
 *       so a free-fall criterion cannot distinguish apogee from general coast.
 * ----------------------------------------------------------------------- */
#define APOGEE_VEL_THRESHOLD    -0.25f  /* C1 primary threshold (m/s) */
#define APOGEE_N_CONSEC         ((int8_t)5) /* IMU-A C1: 5 × 4.81 ms = 24 ms confirmation */
#define APOGEE_N_CONSEC_B       ((int8_t)3) /* IMU-B C1: 3 × 10 ms  = 30 ms confirmation
                                             * Lower count compensates for B's 100 Hz rate so
                                             * both paths meet the 100 ms detection budget. */
#define APOGEE_VEL_SECONDARY    -0.40f  /* C3 sustained threshold (m/s) */
#define APOGEE_SUSTAINED_MS     50U     /* C3: must hold 50 ms continuously */
#define VELOCITY_LPF_ALPHA      0.3f    /* LPF coefficient — telemetry only */
#define APOGEE_COAST_MAX_MS     35000U  /* C2: 35 s absolute limit */
#define APOGEE_COAST_MIN_MS     5000U   /* Min coast before detection allowed (after 4.2 s burn) */

/* -----------------------------------------------------------------------
 * Flight phase detection thresholds (M2020 / 18 kg profile)
 *
 *   Boost net acceleration ≈ 108 m/s² → threshold 30 m/s² = 3g net safe margin
 *   Burnout: specific force drops from ~11g to ~0 → net drops to -g ≈ -9.81
 *   Burnout threshold 5 m/s² (0.5g net) captures transition cleanly
 * ----------------------------------------------------------------------- */
#define LAUNCH_ACCEL_THRESH     30.0f   /* 3g net: motor ignition confirmed */
#define LAUNCH_DEBOUNCE_MS      20U
#define BURNOUT_ACCEL_THRESH    5.0f    /* 0.5g net: motor out */
#define BURNOUT_DEBOUNCE_MS     80U     /* longer debounce to ignore motor chuff */
#define BOOST_MAX_MS            7000U   /* M2020 burn is 4.2 s; 7 s is absolute max.
                                         * If PHASE_BOOST lasts longer than this, the
                                         * sensor reading is stale or the motor has
                                         * certainly burned out — force coast entry so
                                         * the C2 coast timer can start. */

/* IMU watchdog: consider failed if silent for this long */
#define IMU_WATCHDOG_MS         50U

/* FSM: time to wait in PHASE_APOGEE before transitioning to PHASE_DESCEND.
 * Distinct from PYRO_PULSE_MS in main.c — one controls ejection charge
 * duration, the other controls FSM state dwell time. */
#define APOGEE_DESCEND_DELAY_MS 500U

#endif /* EKF_CONFIG_H */
