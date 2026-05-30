#include "apogee_detector.h"
#include "ekf_config.h"
#include <math.h>
#include <string.h>

void APOGEE_Init(ApogeeDetector_t *det)
{
    memset(det, 0, sizeof(*det));
}

void APOGEE_OnCoastEntry(ApogeeDetector_t *det, uint32_t now_ms)
{
    det->coast_start_ms = now_ms;
    det->consec_count   = 0;
    det->freefall_active = false;
}

bool APOGEE_Update(ApogeeDetector_t *det,
                   float velocity, float a_net,
                   FlightPhase_t phase, uint32_t now_ms)
{
    if (det->apogee_fired)
        return true;   /* already latched */

    if (phase != PHASE_COAST)
    {
        det->consec_count = 0;
        det->freefall_active = false;
        return false;
    }

    uint32_t coast_elapsed_ms = now_ms - det->coast_start_ms;

    /* --- C1: velocity threshold --------------------------------------- */
    if (velocity < APOGEE_VEL_THRESHOLD) {
        if (det->consec_count < 127)
            det->consec_count++;
    } else {
        det->consec_count = 0;
    }

    /* Require minimum coast time before C1 can fire */
    bool c1 = (det->consec_count >= APOGEE_N_CONSEC) &&
              (coast_elapsed_ms  >= APOGEE_COAST_MIN_MS);

    /* --- C2: safety timer --------------------------------------------- */
    bool c2 = (coast_elapsed_ms >= APOGEE_COAST_MAX_MS);

    /* --- C3: free-fall detection -------------------------------------- */
    bool near_freefall = (fabsf(a_net) < APOGEE_FREEFALL_THRESH);

    if (near_freefall && !det->freefall_active &&
        coast_elapsed_ms >= APOGEE_COAST_MIN_MS)
    {
        det->freefall_entry_ms = now_ms;
        det->freefall_active   = true;
    }

    if (!near_freefall)
        det->freefall_active = false;

    bool c3 = det->freefall_active &&
              ((now_ms - det->freefall_entry_ms) >= APOGEE_FREEFALL_MS);

    /* Any criterion sufficient */
    if (c1 || c2 || c3) {
        det->apogee_fired = true;
        return true;
    }

    return false;
}

bool APOGEE_Vote(const ApogeeDetector_t *det_a,
                 const ApogeeDetector_t *det_b)
{
    bool a = det_a->apogee_fired;
    bool b = det_b->apogee_fired;
    bool a_failed = det_a->imu_failed;
    bool b_failed = det_b->imu_failed;

    /* Both healthy and agree */
    if (!a_failed && !b_failed)
        return (a && b);

    /* One failed: single IMU decides */
    if (a_failed)  return b;
    if (b_failed)  return a;

    return false;
}
