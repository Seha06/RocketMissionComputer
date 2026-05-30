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

        /* True apogee: first sample where velocity ≤ 0.
         * step > 0 guard: at step=0 true_v is still 0.0f before motor
         * fires (it is set after the integration below) — without this guard
         * apogee_true_ms would be set to 0 at t=0, making every detection
         * delay look like it happened exactly at t=0ms. */
        if (step > 0 && true_v <= 0.0f && apogee_true_ms == 0u)
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
        FlightPhase_t phase = FSM_Update(&fsm, a_net_est, false, t_ms);

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

        /* Accumulate t_ms without truncation error.
         * (uint32_t)(dt*1000) = 4 ms but true step is 4.807 ms.
         * Over 22 s (4576 steps) this under-counts by 3.7 s, making
         * C2 timer validation in the test meaningless.
         * Derive from step count to stay exact. */
        t_ms = (uint32_t)((float)(step + 1) * 1000.0f / IMU_A_ODR_HZ + 0.5f);
    }

    if (!apogee_det || apogee_true_ms == 0)
        return (int)0x80000000;  /* INT32_MIN: not detected */

    return (int)((int32_t)apogee_det_ms - (int32_t)apogee_true_ms);
}

/* ------------------------------------------------------------------
 * run_trial_with_pitch: same as run_trial but with a constant pitch
 * offset.  The IMU body-axis acceleration is projected by cos(pitch),
 * and the lateral component ax is set to a_specific * sin(pitch) so
 * that the attitude filter sees a realistic tilt vector.
 * ------------------------------------------------------------------ */
static int run_trial_with_pitch(unsigned seed, float bias, float pitch_deg)
{
    srand(seed);

    const float dt    = IMU_A_DT_S;
    const float NOISE = 0.05f;
    const float pitch_rad = pitch_deg * 0.017453293f;
    const float cos_p = cosf(pitch_rad);
    const float sin_p = sinf(pitch_rad);

    DR_State_t       dr;
    ApogeeDetector_t apg;
    FSM_State_t      fsm;
    Attitude_t       att;

    DR_Init(&dr, dt);
    APOGEE_Init(&apg);
    FSM_Init(&fsm);
    ATT_Init(&att);

    /* Calibration with tilt: specific force on body-z = g * cos(pitch) */
    float calib[CALIB_SAMPLE_COUNT];
    for (int i = 0; i < CALIB_SAMPLE_COUNT; i++)
        calib[i] = GRAVITY_MS2 * cos_p + bias + randn(NOISE);
    DR_CalibrateBias(&dr, calib, CALIB_SAMPLE_COUNT);

    float true_v   = 0.0f;
    float true_alt = 0.0f;
    uint32_t t_ms  = 0;
    uint32_t apogee_true_ms = 0;
    uint32_t apogee_det_ms  = 0;
    bool     apogee_det     = false;

    const int MAX_STEPS = (int)(40.0f / dt);

    for (int step = 0; step < MAX_STEPS; step++) {
        float t_s = (float)step * dt;
        float m   = (t_s < MOTOR_BURN_S)
                    ? ROCKET_TOTAL_KG - (MOTOR_PROP_KG / MOTOR_BURN_S) * t_s
                    : ROCKET_DRY_KG;

        float a_drag     = -(DRAG_K * true_v * fabsf(true_v)) / m;
        float a_thrust_s = (t_s < MOTOR_BURN_S) ? (MOTOR_THRUST_N / m) : 0.0f;
        float a_specific = a_thrust_s + a_drag;
        float a_inertial = a_specific - GRAVITY_MS2;

        true_v   += a_inertial * dt;
        true_alt += true_v * dt;

        if (step > 0 && true_v <= 0.0f && apogee_true_ms == 0u)
            apogee_true_ms = t_ms;

        /* Body-frame acceleration: a_body_z = a_specific * cos_p,
         * a_body_x = a_specific * sin_p (lateral from tilt) */
        float a_body_z = a_specific * cos_p + bias + randn(NOISE);
        float a_body_x = a_specific * sin_p + randn(0.01f);
        float a_arr[3] = {a_body_x, 0.0f, a_body_z};

        ATT_Update(&att, a_arr, 0.0f, dt);
        float pitch = ATT_GetPitchRad(&att);

        DR_Predict(&dr, a_body_z, pitch, dt);

        float a_net_est = a_body_z * cosf(pitch) - GRAVITY_MS2 - dr.x[2];

        FlightPhase_t prev  = fsm.phase;
        FlightPhase_t phase = FSM_Update(&fsm, a_net_est, false, t_ms);

        if (prev != PHASE_COAST && phase == PHASE_COAST)
            APOGEE_OnCoastEntry(&apg, t_ms);

        if (phase == PHASE_COAST && !apogee_det) {
            if (APOGEE_Update(&apg, DR_GetVelocityRaw(&dr), phase, t_ms)) {
                apogee_det    = true;
                apogee_det_ms = t_ms;
            }
        }

        if (apogee_det && (t_ms - apogee_det_ms) > 2000u) break;

        t_ms = (uint32_t)((float)(step + 1) * 1000.0f / IMU_A_ODR_HZ + 0.5f);
    }

    if (!apogee_det || apogee_true_ms == 0)
        return (int)0x80000000;

    return (int)((int32_t)apogee_det_ms - (int32_t)apogee_true_ms);
}

/* ------------------------------------------------------------------
 * C2 safety timer test: verify that APOGEE_Update fires C2 after
 * APOGEE_COAST_MAX_MS even when velocity stays strongly positive
 * (simulating both IMUs reporting bad data that keeps v > 0).
 * ------------------------------------------------------------------ */
static int test_c2_timer(void)
{
    int pass = 0, fail = 0;

    ApogeeDetector_t det;
    APOGEE_Init(&det);

    const uint32_t coast_start = 5000u;
    APOGEE_OnCoastEntry(&det, coast_start);

    /* Tick with strongly positive velocity — C1 and C3 must not fire */
    const float far_positive_v = 100.0f;
    const uint32_t step_ms = 10u;
    uint32_t t = coast_start + APOGEE_COAST_MIN_MS;
    bool fired = false;

    while (t <= coast_start + APOGEE_COAST_MAX_MS + 1000u) {
        if (!fired)
            fired = APOGEE_Update(&det, far_positive_v, PHASE_COAST, t);
        t += step_ms;
    }

    if (fired && det.apogee_fired) {
        pass++;
    } else {
        printf("  FAIL [C2 timer]: did not fire after %u ms coast\n",
               APOGEE_COAST_MAX_MS);
        fail++;
    }

    /* Verify C2 fired at the right time (not before COAST_MAX) */
    {
        ApogeeDetector_t det2;
        APOGEE_Init(&det2);
        APOGEE_OnCoastEntry(&det2, coast_start);

        uint32_t pre_c2 = coast_start + APOGEE_COAST_MAX_MS - step_ms;
        bool pre_fired = APOGEE_Update(&det2, far_positive_v, PHASE_COAST, pre_c2);
        if (!pre_fired) {
            pass++;
        } else {
            printf("  FAIL [C2 timer]: fired %u ms too early\n", step_ms);
            fail++;
        }
    }

    return fail;
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
     *
     * Bias range: ±0.39 m/s² (LSM6DSM datasheet typical worst-case ±40 mg).
     * Previous range [0, +0.2] m/s² tested only positive bias and covered
     * less than half of the sensor's specified range.  Negative bias causes
     * DR velocity to read *more negative* than true, potentially triggering
     * apogee early; this must be validated.
     * ------------------------------------------------------------------ */
    printf("Running 1000 Monte Carlo trials...\n");
    printf("  Bias range: [%.3f, +%.3f] m/s² (LSM6DSM ±40 mg spec)\n",
           -0.39f, 0.39f);

    int pass = 0, fail = 0;
    int min_d = 9999, max_d = -9999;
    int n_early = 0, n_nodet = 0;

    for (int i = 0; i < 1000; i++) {
        /* Sweep full ±0.39 m/s² bias range uniformly */
        float bias = -0.39f + 0.78f * ((float)i / 999.0f);
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

    /* ------------------------------------------------------------------
     * Dual-IMU Voting Unit Tests
     * APOGEE_Vote() had 0% coverage in the Monte Carlo above.
     * Test all four branches: both-healthy, A-only, B-only, both-failed.
     * ------------------------------------------------------------------ */
    printf("\n=== Dual-IMU Voting Tests ===\n");
    int vote_pass = 0, vote_fail = 0;

    /* Helper macro */
    #define VOTE_CHECK(label, da, db, expected) do {                        \
        bool got = APOGEE_Vote(&(da), &(db));                               \
        if (got == (expected)) {                                            \
            vote_pass++;                                                    \
        } else {                                                            \
            printf("  FAIL [%s]: expected %d got %d\n", label,             \
                   (int)(expected), (int)got);                              \
            vote_fail++;                                                    \
        }                                                                   \
    } while (0)

    /* 1. Both healthy, both fired → deploy */
    {
        ApogeeDetector_t a = {0}, b = {0};
        a.apogee_fired = true; a.imu_failed = false;
        b.apogee_fired = true; b.imu_failed = false;
        VOTE_CHECK("both-healthy both-fired", a, b, true);
    }
    /* 2. Both healthy, only A fired → no deploy (AND logic) */
    {
        ApogeeDetector_t a = {0}, b = {0};
        a.apogee_fired = true;  a.imu_failed = false;
        b.apogee_fired = false; b.imu_failed = false;
        VOTE_CHECK("both-healthy A-only", a, b, false);
    }
    /* 3. A healthy+fired, B failed → A decides, deploy */
    {
        ApogeeDetector_t a = {0}, b = {0};
        a.apogee_fired = true; a.imu_failed = false;
        b.apogee_fired = false; b.imu_failed = true;
        VOTE_CHECK("A-healthy-fired B-failed", a, b, true);
    }
    /* 4. A failed, B healthy+fired → B decides, deploy */
    {
        ApogeeDetector_t a = {0}, b = {0};
        a.apogee_fired = false; a.imu_failed = true;
        b.apogee_fired = true;  b.imu_failed = false;
        VOTE_CHECK("A-failed B-healthy-fired", a, b, true);
    }
    /* 5. Both failed, neither C2 fired → no deploy */
    {
        ApogeeDetector_t a = {0}, b = {0};
        a.imu_failed = true; b.imu_failed = true;
        VOTE_CHECK("both-failed none-fired", a, b, false);
    }
    /* 6. Both failed, C2 fired in A → deploy (safety backstop) */
    {
        ApogeeDetector_t a = {0}, b = {0};
        a.apogee_fired = true; a.imu_failed = true;
        b.imu_failed = true;
        VOTE_CHECK("both-failed C2-in-A", a, b, true);
    }
    /* 7. A healthy not-fired, B healthy fired → no deploy (AND) */
    {
        ApogeeDetector_t a = {0}, b = {0};
        a.apogee_fired = false; a.imu_failed = false;
        b.apogee_fired = true;  b.imu_failed = false;
        VOTE_CHECK("both-healthy B-only", a, b, false);
    }

    printf("  Voting tests: %d PASS / %d FAIL\n", vote_pass, vote_fail);
    if (vote_fail > 0) {
        printf("FAIL: dual-IMU voting logic error\n");
        return 1;
    }
    printf("PASS: dual-IMU voting all correct\n");

    /* ------------------------------------------------------------------
     * C2 Safety Timer Tests
     * Verifies that C2 fires after APOGEE_COAST_MAX_MS even when velocity
     * stays strongly positive (C1 and C3 never trigger).
     * ------------------------------------------------------------------ */
    printf("\n=== C2 Safety Timer Tests ===\n");
    {
        int c2_fail = test_c2_timer();
        if (c2_fail > 0) {
            printf("FAIL: C2 timer test failed (%d error(s))\n", c2_fail);
            return 1;
        }
        printf("PASS: C2 safety timer fires correctly\n");
    }

    /* ------------------------------------------------------------------
     * Pitch Deviation Tests (0°, 5°, 10°, 15°)
     * Verifies that the complementary filter + cosine projection keep
     * detection delay within 100 ms even when the rocket tilts during boost.
     * ------------------------------------------------------------------ */
    printf("\n=== Pitch Deviation Tests ===\n");
    {
        const float pitch_angles[] = {0.0f, 5.0f, 10.0f, 15.0f};
        const int   N_ANGLES = 4;
        const int   N_TRIALS = 50;
        int pitch_overall_fail = 0;

        for (int ai = 0; ai < N_ANGLES; ai++) {
            float pitch_deg = pitch_angles[ai];
            int p_pass = 0, p_fail = 0;
            int p_min = 9999, p_max = -9999;

            for (int ti = 0; ti < N_TRIALS; ti++) {
                /* Mix seed to avoid correlated LCG states across angles */
                unsigned seed = (unsigned)(ti * 0x9e3779b9u) ^ (unsigned)(ai * 0x517CC1B7u);
                float bias = -0.20f + 0.40f * ((float)ti / (float)(N_TRIALS - 1));

                int d = run_trial_with_pitch(seed, bias, pitch_deg);

                if (d == (int)0x80000000) {
                    p_fail++;
                    continue;
                }

                if (d < p_min) p_min = d;
                if (d > p_max) p_max = d;

                if (d >= -20 && d <= 100) p_pass++;
                else                      p_fail++;
            }

            printf("  pitch=%5.1f°  pass=%d/%d  delay=[%d, %d] ms  %s\n",
                   pitch_deg, p_pass, N_TRIALS, p_min, p_max,
                   (p_fail == 0) ? "PASS" : "FAIL ***");

            if (p_fail > 0) pitch_overall_fail += p_fail;
        }

        if (pitch_overall_fail > 0) {
            printf("FAIL: %d pitch trial(s) outside window\n", pitch_overall_fail);
            return 1;
        }
        printf("PASS: all pitch deviation trials within 100 ms window\n");
    }

    return 0;
}
