#ifndef DEAD_RECKONING_H
#define DEAD_RECKONING_H

#include <stdint.h>
#include <stdbool.h>

/*
 * 1-D dead reckoning along the rocket's vertical axis.
 *
 * State vector: x = [altitude (m), velocity (m/s), accel_bias (m/s²)]
 *
 * The 3×3 covariance P is propagated alongside the state so that
 * uncertainty grows naturally with integration time. No measurement
 * update step is used (pure dead reckoning); the covariance is provided
 * as a health indicator and for potential future sensor fusion extension.
 *
 * Bias is estimated from static IMU samples before launch and held
 * constant during flight.
 */
typedef struct {
    float x[3];         /* [altitude, velocity, accel_bias] */
    float P[3][3];      /* Error covariance */
    float Q[3][3];      /* Process noise (constant, diagonal) */

    float velocity_lpf; /* Low-pass filtered velocity for apogee detection */
    float dt;           /* Nominal time step (s) */
    bool  calibrated;
} DR_State_t;

/*
 * Initialise state. Call once at power-on.
 * dt_s: expected time step between DR_Predict calls.
 */
void DR_Init(DR_State_t *dr, float dt_s);

/*
 * Calibrate accelerometer bias from N static IMU samples.
 * a_vertical[]: array of N vertical-axis accelerometer readings (m/s²)
 * n           : number of samples (use CALIB_SAMPLE_COUNT)
 *
 * Must be called while the rocket is stationary on the launch pad.
 */
void DR_CalibrateBias(DR_State_t *dr,
                      const float *a_vertical, uint16_t n);

/*
 * Predict step — call at every IMU sample.
 * a_body_vertical : IMU vertical-axis reading (m/s²), body frame
 * pitch_rad       : rocket tilt from vertical (from attitude tracker)
 * dt              : actual time step (s), use measured dt for accuracy
 */
void DR_Predict(DR_State_t *dr,
                float a_body_vertical, float pitch_rad, float dt);

/* Getters */
float DR_GetVelocity(const DR_State_t *dr);         /* Filtered velocity  */
float DR_GetVelocityRaw(const DR_State_t *dr);      /* Unfiltered velocity */
float DR_GetAltitude(const DR_State_t *dr);
float DR_GetVelocityStd(const DR_State_t *dr);      /* 1-sigma uncertainty */

#endif /* DEAD_RECKONING_H */
