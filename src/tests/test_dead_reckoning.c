/*
 * Unit test: DR + apogee detector with synthetic flight profile
 *
 * Rocket body-frame convention: Z-axis = longitudinal (up).
 * Accelerometer measures specific force (not inertial acceleration).
 *
 *   Pad (static):  a_meas ≈ +g (sensor reads reaction force from ground)
 *   Boost:         a_meas = thrust/mass (specific force, large positive)
 *   Coast/freefall: a_meas ≈ 0  (weightless — sensor reads ~0)
 *
 * Navigation-frame net acceleration:
 *   a_net = a_meas - g       (specific force minus gravity)
 *   Pad:   a_net = g - g = 0       (at rest ✓)
 *   Boost: a_net = (n*g) - g = (n-1)*g   (e.g. 3g thrust → 2g net up)
 *   Coast: a_net = 0 - g = -g            (free fall ✓)
 *
 * Profile (3g thrust, 2 s burn):
 *   0–2 s:  boost,  v builds to 2*(3-1)*g = 39.2 m/s
 *   2–6 s:  coast,  v decreases at g → reaches 0 at t≈6 s (apogee)
 *
 * Expected: apogee fires within ≤10 ms of v=0.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <assert.h>

#include "../ekf_config.h"

/* Include implementation files directly for testing */
#include "../nav/dead_reckoning.c"
#include "../nav/apogee_detector.c"
#include "../flight_fsm.c"
#include "../nav/attitude.c"

/* Simple Gaussian noise generator (Box-Muller) */
static float randn(float sigma)
{
    float u1 = ((float)rand() + 1.0f) / ((float)RAND_MAX + 1.0f);
    float u2 = ((float)rand() + 1.0f) / ((float)RAND_MAX + 1.0f);
    return sigma * sqrtf(-2.0f * logf(u1)) * cosf(6.2831853f * u2);
}

/* Run one Monte Carlo trial. Returns detection delay in ms, or -1 on failure. */
static int run_trial(unsigned seed, float bias)
{
    srand(seed);
    const float NOISE   = 0.05f;  /* m/s² per sample */
    const float THRUST  = 3.0f * GRAVITY_MS2;  /* specific force during boost */
    const float BURN_S  = 2.0f;
    const float dt      = IMU_A_DT_S;

    DR_State_t       dr;
    ApogeeDetector_t apg;
    FSM_State_t      fsm;
    Attitude_t       att;

    DR_Init(&dr, dt);
    APOGEE_Init(&apg);
    FSM_Init(&fsm);
    ATT_Init(&att);

    /* Calibration: 200 static samples (a_meas ≈ g + bias) */
    float calib[CALIB_SAMPLE_COUNT];
    for (int i = 0; i < CALIB_SAMPLE_COUNT; i++)
        calib[i] = GRAVITY_MS2 + bias + randn(NOISE);
    DR_CalibrateBias(&dr, calib, CALIB_SAMPLE_COUNT);

    /* Simulate flight */
    float true_velocity = 0.0f;
    uint32_t t_ms = 0;
    uint32_t apogee_true_ms = 0;
    uint32_t apogee_det_ms  = 0;
    bool apogee_det = false;

    for (int step = 0; step < (int)(10.0f / dt); step++) {
        float t_s = step * dt;

        /*
         * Specific force (what the accelerometer measures):
         *   Boost:  THRUST  (thrust force / mass)
         *   Coast:  0       (free fall, weightless)
         */
        float a_specific = (t_s < BURN_S) ? THRUST : 0.0f;

        /* True inertial (navigation-frame) acceleration = specific - gravity */
        float a_inertial = a_specific - GRAVITY_MS2;
        true_velocity += a_inertial * dt;

        /* Mark first sample where true velocity ≤ 0 as true apogee */
        if (true_velocity <= 0.0f && apogee_true_ms == 0)
            apogee_true_ms = t_ms;

        /* Simulated sensor reading with noise and bias */
        float a_meas = a_specific + bias + randn(NOISE);

        /* Attitude (pitch = 0 for ideal vertical flight) */
        float a_arr[3] = {0.0f, 0.0f, a_meas};
        ATT_Update(&att, a_arr, 0.0f, dt);
        float pitch = ATT_GetPitchRad(&att);

        /* Dead reckoning */
        DR_Predict(&dr, a_meas, pitch, dt);

        /* Flight FSM: first argument = net accel estimate from DR */
        float a_net_est = a_meas * cosf(pitch) - GRAVITY_MS2 - dr.x[2];
        FlightPhase_t prev = fsm.phase;
        FlightPhase_t phase = FSM_Update(&fsm,
                                          a_net_est,
                                          DR_GetVelocityRaw(&dr),
                                          false, t_ms);

        if (prev != PHASE_COAST && phase == PHASE_COAST)
            APOGEE_OnCoastEntry(&apg, t_ms);

        /* Apogee detection: use RAW velocity (no LPF lag) */
        if (phase == PHASE_COAST && !apogee_det) {
            if (APOGEE_Update(&apg,
                              DR_GetVelocityRaw(&dr),  /* raw, no LPF lag */
                              a_net_est, phase, t_ms)) {
                apogee_det = true;
                apogee_det_ms = t_ms;
            }
        }

        t_ms += (uint32_t)(dt * 1000.0f);
    }

    if (!apogee_det || apogee_true_ms == 0)
        return -1;

    return (int)((int32_t)apogee_det_ms - (int32_t)apogee_true_ms);
}

int main(void)
{
    /* --- Single verbose trial --- */
    srand(42);
    const float BIAS = 0.15f;
    int delay = run_trial(42, BIAS);
    printf("Single trial: apogee detection delay = %d ms\n", delay);
    if (delay < 0) { printf("FAIL: apogee not detected\n"); return 1; }
    if (delay > 10) { printf("FAIL: delay %d ms > 10 ms limit\n", delay); return 1; }

    /* --- Monte Carlo: 1000 trials with varying noise seeds and bias --- */
    int pass = 0, fail = 0;
    for (int i = 0; i < 1000; i++) {
        /* Vary bias ±0.2 m/s² to test calibration robustness */
        float bias = 0.1f + 0.2f * ((float)(i % 7) / 7.0f - 0.5f);
        int d = run_trial((unsigned)i + 1000u, bias);
        if (d >= -10 && d <= 10)  /* ±10 ms of true apogee */
            pass++;
        else
            fail++;
    }

    printf("Monte Carlo (1000 trials): PASS=%d FAIL=%d (%.1f%%)\n",
           pass, fail, 100.0f * pass / 1000.0f);

    if (pass < 990) {
        printf("FAIL: success rate %.1f%% < 99%%\n",
               100.0f * pass / 1000.0f);
        return 1;
    }

    printf("PASS: ≥99%% success, detection delay ≤10 ms\n");
    return 0;
}
