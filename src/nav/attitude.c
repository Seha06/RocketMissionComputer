#include "attitude.h"
#include "ekf_config.h"
#include <math.h>
#include <string.h>

/* Complementary filter weight: 0.02 gives ~8 Hz accel correction BW */
#define COMP_ALPHA_GYRO     0.98f
#define COMP_ALPHA_ACCEL    (1.0f - COMP_ALPHA_GYRO)

/* Suppress accelerometer correction when rocket exceeds this acceleration.
 * Motor burn creates 3-30g; at those levels accel tilt is meaningless. */
#define HIGH_G_SUPPRESS_THRESH  (2.0f * GRAVITY_MS2)

void ATT_Init(Attitude_t *att)
{
    att->pitch_rad = 0.0f;
    att->gyro_bias_rad_s = 0.0f;
    att->initialised = true;
}

void ATT_CalibrateGyroBias(Attitude_t *att, float bias_rad_s)
{
    att->gyro_bias_rad_s = bias_rad_s;
}

void ATT_Update(Attitude_t *att,
                const float a[3], float gyro_pitch, float dt)
{
    float accel_mag = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);

    /* Gyro integration (always active) */
    float pitch_gyro = att->pitch_rad + (gyro_pitch - att->gyro_bias_rad_s) * dt;

    /*
     * Require a[ROCKET_ACCEL_AXIS] > 0: during coast the longitudinal axis
     * reads negative specific force (aerodynamic drag opposing upward motion).
     * atan2(ax, negative_az) = ~π which would flip pitch 180° — wrong.
     * The accel correction is only valid when the main axis sees +g (upright,
     * low-G flight near pad or slow manoeuvre).
     */
    if (accel_mag < HIGH_G_SUPPRESS_THRESH && accel_mag > 0.5f * GRAVITY_MS2
        && a[ROCKET_ACCEL_AXIS] > 0.0f) {
        /*
         * 3-D tilt from gravity: cone angle between the rocket axis and
         * the gravity vector, computed from the axial and lateral components.
         * Using the total lateral magnitude (not a single perpendicular axis)
         * captures tilt in both X-Z and Y-Z planes, so a Y-axis lean does
         * not go undetected.  ROCKET_ACCEL_AXIS selects the axial channel
         * without hardcoding an index.
         */
        float a_axial   = a[ROCKET_ACCEL_AXIS];
        float a_sq_sum  = a[0]*a[0] + a[1]*a[1] + a[2]*a[2];
        float a_lateral = sqrtf(fmaxf(0.0f, a_sq_sum - a_axial*a_axial));
        float pitch_accel = atan2f(a_lateral, a_axial);
        att->pitch_rad = COMP_ALPHA_GYRO * pitch_gyro +
                         COMP_ALPHA_ACCEL * pitch_accel;
    } else {
        att->pitch_rad = pitch_gyro;
    }
}

float ATT_GetPitchRad(const Attitude_t *att)
{
    return att->pitch_rad;
}
