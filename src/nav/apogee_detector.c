#include "apogee_detector.h"
#include "ekf_config.h"
#include <string.h>

void APOGEE_Init(ApogeeDetector_t *det)
{
    memset(det, 0, sizeof(*det));
}

void APOGEE_OnCoastEntry(ApogeeDetector_t *det, uint32_t now_ms)
{
    det->coast_start_ms   = now_ms;
    det->consec_count     = 0;
    det->sustained_active = false;
}

bool APOGEE_Update(ApogeeDetector_t *det,
                   float velocity,
                   FlightPhase_t phase,
                   uint32_t now_ms)
{
    if (det->apogee_fired)
        return true;

    if (phase != PHASE_COAST) {
        det->consec_count     = 0;
        det->sustained_active = false;
        return false;
    }

    uint32_t coast_elapsed = now_ms - det->coast_start_ms;

    /* Require minimum coast time before any criterion can fire.
     * Prevents spurious trigger right after burnout. */
    if (coast_elapsed < APOGEE_COAST_MIN_MS)
        return false;

    /* ----- C1: consecutive samples below primary threshold --------------- */
    if (velocity < APOGEE_VEL_THRESHOLD) {
        if (det->consec_count < 127) det->consec_count++;
    } else {
        det->consec_count = 0;
    }
    bool c1 = (det->consec_count >= APOGEE_N_CONSEC);

    /* ----- C2: absolute safety timer ------------------------------------- */
    bool c2 = (coast_elapsed >= APOGEE_COAST_MAX_MS);

    /* ----- C3: sustained velocity drop -----------------------------------
     * Opens a timing window the first time velocity drops below
     * APOGEE_VEL_SECONDARY.  If velocity stays below that level for
     * APOGEE_SUSTAINED_MS without interruption, C3 fires.
     * A single sample above the threshold resets the window — this
     * requires a genuinely sustained negative velocity, not a noise spike. */
    if (velocity < APOGEE_VEL_SECONDARY) {
        if (!det->sustained_active) {
            det->sustained_entry_ms = now_ms;
            det->sustained_active   = true;
        }
    } else {
        det->sustained_active = false;
    }

    bool c3 = det->sustained_active &&
              ((now_ms - det->sustained_entry_ms) >= APOGEE_SUSTAINED_MS);

    if (c1 || c2 || c3) {
        det->apogee_fired = true;
        return true;
    }

    return false;
}

bool APOGEE_Vote(const ApogeeDetector_t *det_a,
                 const ApogeeDetector_t *det_b)
{
    bool a_ok = !det_a->imu_failed;
    bool b_ok = !det_b->imu_failed;

    if (a_ok && b_ok) return (det_a->apogee_fired && det_b->apogee_fired);
    if (a_ok)         return  det_a->apogee_fired;
    if (b_ok)         return  det_b->apogee_fired;

    /*
     * Both IMUs failed.  C1 and C3 are meaningless without sensor data, but
     * C2 (the absolute coast timer) fires entirely from the system clock and
     * does not depend on IMU health.  If either detector's C2 already fired
     * apogee_fired, honour it — this is the last-resort safety backstop.
     * Without this the rocket would never deploy even though C2 expired.
     */
    return (det_a->apogee_fired || det_b->apogee_fired);
}
