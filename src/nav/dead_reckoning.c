#include "dead_reckoning.h"
#include "ekf_config.h"
#include <math.h>
#include <string.h>

/* -------------------------------------------------------------------------
 * Helper: 3×3 matrix multiply C = A * B
 * ------------------------------------------------------------------------- */
static inline void mat3_mul(const float A[3][3], const float B[3][3], float C[3][3])
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++)
                s += A[i][k] * B[k][j];
            C[i][j] = s;
        }
    }
}

/* C = A * B^T */
static inline void mat3_mul_bt(const float A[3][3], const float B[3][3],
                               float C[3][3])
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++)
                s += A[i][k] * B[j][k];
            C[i][j] = s;
        }
    }
}

/* C = A + B */
static inline void mat3_add(const float A[3][3], const float B[3][3], float C[3][3])
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            C[i][j] = A[i][j] + B[i][j];
}

/* Enforce P symmetry to counteract floating-point rounding */
static inline void mat3_symmetrise(float P[3][3])
{
    for (int i = 0; i < 3; i++)
        for (int j = i + 1; j < 3; j++)
            P[i][j] = P[j][i] = 0.5f * (P[i][j] + P[j][i]);
}

/* -------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */
void DR_Init(DR_State_t *dr, float dt_s)
{
    memset(dr, 0, sizeof(*dr));
    dr->dt = dt_s;

    /* Initial covariance: large uncertainty before calibration */
    dr->P[0][0] = 100.0f;   /* altitude: ±10 m */
    dr->P[1][1] = 1.0f;     /* velocity: ±1 m/s */
    dr->P[2][2] = 1.0f;     /* bias:     ±1 m/s² */

    /* Process noise (diagonal, scaled at init — multiplied by dt in Predict) */
    float sa2 = DR_SIGMA_ACCEL * DR_SIGMA_ACCEL;
    float sb2 = DR_SIGMA_BIAS_DRIFT * DR_SIGMA_BIAS_DRIFT;
    dr->Q[1][1] = sa2;
    dr->Q[2][2] = sb2;
    /* Q[0][0] = 0: altitude noise is driven by velocity, not independently */
}

void DR_CalibrateBias(DR_State_t *dr,
                      const float *a_vertical, uint32_t n)
{
    if (n == 0u) return;

    double sum = 0.0;
    for (uint32_t i = 0u; i < n; i++)
        sum += (double)a_vertical[i];

    float mean = (float)(sum / (double)n);

    /*
     * At rest: a_vertical = g + bias (sensor reads gravity + bias).
     * Bias = mean - g (using sign convention: +g when pointing up).
     */
    dr->x[2] = mean - GRAVITY_MS2;

    /* Tighten bias covariance after calibration */
    dr->P[2][2] = 0.01f;

    dr->x[0] = 0.0f;  /* altitude = 0 at launch pad */
    dr->x[1] = 0.0f;  /* velocity = 0 at launch pad */
    dr->velocity_lpf = 0.0f;
    dr->calibrated = true;
}

void DR_Predict(DR_State_t *dr,
                float a_body_vertical, float pitch_rad, float dt)
{
    /*
     * Project body-frame vertical acceleration to navigation-frame vertical.
     * For small pitch angles (< 15°), cos(pitch) ≈ 0.97, error < 3%.
     * For larger deviations, the cosine projection remains accurate.
     */
    float cos_pitch = cosf(pitch_rad);
    float a_nav = a_body_vertical * cos_pitch;

    /* Subtract gravity and bias to get net vertical acceleration */
    float a_net = a_nav - GRAVITY_MS2 - dr->x[2];

    float h  = dr->x[0];
    float v  = dr->x[1];
    float ba = dr->x[2];

    /* State propagation */
    dr->x[0] = h  + v * dt + 0.5f * a_net * dt * dt;
    dr->x[1] = v  + a_net * dt;
    dr->x[2] = ba;  /* bias held constant (random walk modelled in Q) */

    /*
     * Covariance propagation: P = F * P * F^T + Q
     *
     * State transition matrix:
     *   F = | 1  dt  -0.5*dt² |
     *       | 0   1  -dt      |
     *       | 0   0   1       |
     */
    float dt2 = dt * dt;
    float F[3][3] = {
        { 1.0f, dt,   -0.5f * dt2 },
        { 0.0f, 1.0f, -dt         },
        { 0.0f, 0.0f,  1.0f       }
    };

    float FP[3][3], FPFt[3][3];
    mat3_mul(F, dr->P, FP);
    mat3_mul_bt(FP, F, FPFt);

    /*
     * Per-step process noise:
     *   Velocity: noise source is per-sample acceleration σ_a (m/s²).
     *             Each step contributes σ_a×dt to velocity; variance = σ_a²×dt².
     *   Bias:     random-walk specified per step (m/s²); variance = σ_b² directly.
     *
     * Q[1][1] stores σ_a², Q[2][2] stores σ_b² (set in DR_Init).
     * dt2 was computed above for the F matrix.
     */
    float Qdt[3][3] = {
        { 0.0f,                  0.0f,             0.0f           },
        { 0.0f, dr->Q[1][1]*dt2, 0.0f             },
        { 0.0f, 0.0f,            dr->Q[2][2]      }
    };

    mat3_add(FPFt, Qdt, dr->P);
    mat3_symmetrise(dr->P);

    /* Enforce minimum variance floor to prevent negative diagonal entries */
    if (dr->P[0][0] < 1e-6f) dr->P[0][0] = 1e-6f;
    if (dr->P[1][1] < 1e-6f) dr->P[1][1] = 1e-6f;
    if (dr->P[2][2] < 1e-9f) dr->P[2][2] = 1e-9f;

    /* Low-pass filter on velocity for smooth zero-crossing detection */
    dr->velocity_lpf = VELOCITY_LPF_ALPHA * dr->x[1] +
                       (1.0f - VELOCITY_LPF_ALPHA) * dr->velocity_lpf;
}

float DR_GetVelocity(const DR_State_t *dr)
{
    return dr->velocity_lpf;
}

float DR_GetVelocityRaw(const DR_State_t *dr)
{
    return dr->x[1];
}

float DR_GetAltitude(const DR_State_t *dr)
{
    return dr->x[0];
}

float DR_GetVelocityStd(const DR_State_t *dr)
{
    return sqrtf(dr->P[1][1]);
}
