#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <stdbool.h>

/*
 * Pitch angle tracker for a near-vertical rocket.
 * Uses a complementary filter: gyro integration (high-freq) corrected by
 * accelerometer tilt estimate (low-freq, disabled during high-g boost).
 */
typedef struct {
    float pitch_rad;        /* Current pitch from vertical (rad), 0 = straight up */
    float gyro_bias_rad_s;  /* Estimated gyro bias on pitch axis */
    bool  initialised;
} Attitude_t;

void ATT_Init(Attitude_t *att);

/*
 * Calibrate gyro pitch-axis bias from static samples collected on the pad.
 * Must be called while the rocket is motionless, before launch.
 * Removes systematic offset that would otherwise integrate over the 4.2 s
 * boost phase and produce a persistent pitch error (~4°) at burnout.
 */
void ATT_CalibrateGyroBias(Attitude_t *att, float bias_rad_s);

/*
 * att  : attitude state
 * a    : [ax, ay, az] m/s² body frame
 * gyro : pitch-axis gyro rate (rad/s)
 * dt   : time step (s)
 *
 * During high-g (|a| > 2g), accelerometer correction is disabled —
 * only gyro integration runs, preventing boost-phase corruption.
 */
void ATT_Update(Attitude_t *att,
                const float a[3], float gyro_pitch, float dt);

float ATT_GetPitchRad(const Attitude_t *att);

#endif /* ATTITUDE_H */
