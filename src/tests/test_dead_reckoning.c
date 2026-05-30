/*
 * Unit test: M2020 / 18 kg rocket — dead reckoning apogee detection
 *
 * Mission requirements:
 *   - Apogee altitude  : ~3000 m AGL
 *   - Detection window : ≤ 100 ms after true apogee
 *   - Success rate     : 100 % (1000/1000 Monte Carlo trials)
 *
 * Motor model (Cesaroni / AeroTech M2020 class):
 *   Avg thrust  : 2020 N
 *   Burn time   : 4.2 s
 *   Prop mass   : 1.8 kg
 *
 * Aerodynamics:
 *   Quadratic drag  F_d = DRAG_K * v * |v|  (lumped coefficient)
 *   DRAG_K tuned so the no-noise nominal trajectory reaches ~3000 m.
 *
 * Accelerometer convention (specific force):
 *   At rest on pad : a_meas ≈ +g  (reaction force from pad)
 *   During boost   : a_meas = thrust/m + drag/m  (specific force, up positive)
 *   Coast/freefall : a_meas ≈ 0   (weightless — NOT equal to -g!)
 *   Navigation-frame accel = a_meas - g  (subtract gravity to get inertial accel)
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

#include "../ekf_config.h"
#include "../nav/dead_reckoning.c"
#include "../nav/apogee_detector.c"
#include "../flight_fsm.c"
#include "../nav/attitude.c"

/* Motor parameters */
#define MOTOR_THRUST_N      2020.0f
#define MOTOR_BURN_S        4.2f
#define MOTOR_PROP_KG       1.8f
#define ROCKET_TOTAL_KG     18.0f
#define ROCKET_DRY_KG       (ROCKET_TOTAL_KG - MOTOR_PROP_KG)

/*
 * Lumped drag coefficient: F_drag = DRAG_K * v * |v|
 * Tuned so that the zero-noise simulation reaches ≈ 3000 m apogee (22 s flight).
 * Binary-search calibrated for M2020/18 kg profile; burnout velocity ≈ 362 m/s.
 */
#define DRAG_K              0.0073f

/* Gaussian noise (Box-Muller) */
static float randn(float sigma)
{
    float u1 = ((float)rand() + 1.0f) / ((float)RAND_MAX + 1.0f);
    float u2 = ((float)rand() + 1.0f) / ((float)RAND_MAX + 1.0f);
    return sigma * sqrtf(-2.0f * logf(u1)) * cosf(6.2831853f * u2);
}

/*
 * Simulate one flight and run the detection algorithm.
 * Returns detection delay in ms relative to true apogee, or INT32_MIN if
 * apogee was never detected.
 */
static int run_trial(unsigned seed, float bias)
{
    srand(seed);

    const float dt   = IMU_A_DT_S;
    const float NOISE = 0.05f;   /* LSM6DSM noise density at 208 Hz */

    DR_State_t       dr;
    ApogeeDetector_t apg;
    FSM_State_t      fsm;
    Attitude_t       att;

    DR_Init(&dr, dt);
    APOGEE_Init(&apg);
    FSM_Init(&fsm);
    ATT_Init(&att);

    /* Calibration: 1000 static samples (CALIB_SAMPLE_COUNT) */
    float calib[CALIB_SAMPLE_COUNT];
    for (int i = 0; i < CALIB_SAMPLE_COUNT; i++)
        calib[i] = GRAVITY_MS2 + bias + randn(NOISE);
    DR_CalibrateBias(&dr, calib, CALIB_SAMPLE_COUNT);

    /* Flight simulation */
    float true_v   = 0.0f;   /* true inertial vertical velocity (m/s) */
    float true_alt = 0.0f;   /* true altitude AGL (m) */
    uint32_t t_ms  = 0;
    uint32_t apogee_true_ms = 0;
    uint32_t apogee_det_ms  = 0;
    bool     apogee_det     = false;
    float    peak_alt       = 0.0f;

    /* 40 seconds covers any realistic M2020 flight to apogee + margin */
    const int MAX_STEPS = (int)(40.0f / dt);

    for (int step = 0; step < MAX_STEPS; step++) {
        float t_s = (float)step * dt;

        /* Current rocket mass (propellant burns linearly) */
        float m = (t_s < MOTOR_BURN_S)
                  ? ROCKET_TOTAL_KG - (MOTOR_PROP_KG / MOTOR_BURN_S) * t_s
                  : ROCKET_DRY_KG;

        /* Aerodynamic drag deceleration (always opposes velocity) */
        float a_drag = -(DRAG_K * true_v * fabsf(true_v)) / m;

        /* Thrust specific force (zero after burnout) */
        float a_thrust_specific = (t_s < MOTOR_BURN_S) ? (MOTOR_THRUST_N / m) : 0.0f;

        /*
         * Specific force = what the accelerometer measures (no gravity):
         *   a_specific = a_thrust + a_drag
         * Inertial accel = specific force − gravity (up positive):
         *   a_inertial = a_specific − g
         */
        float a_specific = a_thrust_specific + a_drag;
        float a_inertial = a_specific - GRAVITY_MS2;

        /* Integrate true trajectory */
        true_v   += a_inertial * dt;
        true_alt += true_v * dt;
        if (true_alt > peak_alt) peak_alt = true_alt;

        /* True apogee: first sample where velocity ≤ 0 */
        if (true_v <= 0.0f && apogee_true_ms == 0)
            apogee_true_ms = t_ms;

        /* Simulated accelerometer reading: specific force + bias + noise */
        float a_meas = a_specific + bias + randn(NOISE);

        /* Attitude (pitch≈0 for near-vertical flight) */
        float a_arr[3] = {0.0f, 0.0f, a_meas};
        ATT_Update(&att, a_arr, 0.0f, dt);
        float pitch = ATT_GetPitchRad(&att);

        /* Dead reckoning */
        DR_Predict(&dr, a_meas, pitch, dt);

        /* Net acceleration estimate for FSM phase detection */
        float a_net_est = a_meas * cosf(pitch) - GRAVITY_MS2 - dr.x[2];

        /* Flight FSM */
        FlightPhase_t prev  = fsm.phase;
        FlightPhase_t phase = FSM_Update(&fsm, a_net_est,
                                          DR_GetVelocityRaw(&dr),
                                          false, t_ms);

        if (prev != PHASE_COAST && phase == PHASE_COAST)
            APOGEE_OnCoastEntry(&apg, t_ms);

        /* Apogee detection (raw velocity, no LPF) */
        if (phase == PHASE_COAST && !apogee_det) {
            if (APOGEE_Update(&apg, DR_GetVelocityRaw(&dr), phase, t_ms)) {
                apogee_det    = true;
                apogee_det_ms = t_ms;
            }
        }

        /* Stop simulation once well past apogee (descent confirmed) */
        if (apogee_det && (t_ms - apogee_det_ms) > 2000u)
            break;

        t_ms += (uint32_t)(dt * 1000.0f);
    }

    if (!apogee_det || apogee_true_ms == 0)
        return (int)0x80000000;  /* INT32_MIN: not detected */

    return (int)((int32_t)apogee_det_ms - (int32_t)apogee_true_ms);
}

int main(void)
{
    /* ------------------------------------------------------------------
     * Nominal trajectory check (no noise, no bias)
     * ------------------------------------------------------------------ */
    printf("=== M2020 / 18 kg Apogee Detection Test ===\n\n");
    {
        srand(0);
        float calib_nom[CALIB_SAMPLE_COUNT];
        for (int i = 0; i < CALIB_SAMPLE_COUNT; i++)
            calib_nom[i] = GRAVITY_MS2; /* perfect, no noise */
        DR_State_t dr_nom; DR_Init(&dr_nom, IMU_A_DT_S);
        DR_CalibrateBias(&dr_nom, calib_nom, CALIB_SAMPLE_COUNT);

        float v=0, h=0, peak=0;
        float t_s=0;
        for (int s=0; s<(int)(30.0f/IMU_A_DT_S); s++, t_s+=IMU_A_DT_S) {
            float m = (t_s<MOTOR_BURN_S)?ROCKET_TOTAL_KG-(MOTOR_PROP_KG/MOTOR_BURN_S)*t_s:ROCKET_DRY_KG;
            float ad = -(DRAG_K*v*fabsf(v))/m;
            float asp = (t_s<MOTOR_BURN_S)?(MOTOR_THRUST_N/m):0.0f;
            v += (asp + ad - GRAVITY_MS2)*IMU_A_DT_S;
            h += v*IMU_A_DT_S;
            if(h>peak) peak=h;
        }
        printf("Nominal trajectory: peak altitude = %.0f m\n", peak);
        if (peak < 2000.0f || peak > 5000.0f)
            printf("WARNING: apogee %.0f m is outside [2000, 5000] m — check DRAG_K\n", peak);
    }

    /* ------------------------------------------------------------------
     * Single verbose trial
     * ------------------------------------------------------------------ */
    int d_single = run_trial(42u, 0.12f);
    printf("Single trial (seed=42, bias=0.12): detection delay = %d ms\n\n",
           d_single);
    if (d_single == (int)0x80000000) {
        printf("FAIL: apogee not detected\n");
        return 1;
    }
    if (d_single < -100 || d_single > 100) {
        printf("FAIL: delay %d ms outside [-100, +100] ms window\n", d_single);
        return 1;
    }

    /* ------------------------------------------------------------------
     * Monte Carlo: 1000 trials
     * Bias range: ±0.2 m/s² around 0.1 m/s² (realistic LSM6DSM offset)
     * ------------------------------------------------------------------ */
    printf("Running 1000 Monte Carlo trials...\n");

    int pass = 0, fail = 0;
    int min_d = 9999, max_d = -9999;
    int n_early = 0, n_nodet = 0;

    for (int i = 0; i < 1000; i++) {
        float bias = 0.1f + 0.2f * ((float)(i % 11) / 11.0f - 0.5f);
        int d = run_trial((unsigned)i + 5000u, bias);

        if (d == (int)0x80000000) {
            n_nodet++;
            fail++;
            continue;
        }

        if (d < min_d) min_d = d;
        if (d > max_d) max_d = d;
        if (d < 0) n_early++;

        /* Pass: detected within 100 ms of true apogee (positive or slightly negative) */
        if (d >= -20 && d <= 100)
            pass++;
        else
            fail++;
    }

    printf("Results:\n");
    printf("  PASS      : %d / 1000  (%.1f%%)\n", pass,  (float)pass  * 0.1f);
    printf("  FAIL      : %d / 1000\n", fail);
    printf("  Not det.  : %d\n", n_nodet);
    printf("  Early(<0) : %d  (up to 20 ms early = acceptable, rocket still ascending)\n", n_early);
    printf("  Delay range: [%d ms, %d ms]\n", min_d, max_d);

    if (pass < 1000) {
        printf("\nFAIL: %d trial(s) outside [-20, +100] ms window\n", fail);
        return 1;
    }

    printf("\nPASS: 100%% detection within 100 ms window\n");
    return 0;
}
