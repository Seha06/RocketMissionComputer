#ifndef APOGEE_DETECTOR_H
#define APOGEE_DETECTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "../flight_fsm.h"

/*
 * Per-IMU apogee detector.
 * Three independent criteria; any one is sufficient to declare apogee.
 *
 * C1 (PRIMARY):   raw velocity < threshold for N_CONSEC samples
 * C2 (BACKUP):    coast timer exceeded T_COAST_MAX  (sensor failure safe)
 * C3 (SECONDARY): near-free-fall detected for > FREEFALL_MS after coast
 *
 * Dual-IMU voting (APOGEE_Vote) requires both detectors to agree, unless
 * one is marked as failed.
 */
typedef struct {
    int8_t   consec_count;       /* consecutive samples below threshold */
    bool     apogee_fired;       /* latched when apogee detected */
    bool     imu_failed;         /* set if IMU watchdog expires */
    uint32_t coast_start_ms;     /* timestamp of COAST entry */
    uint32_t freefall_entry_ms;  /* timestamp when freefall criterion started */
    bool     freefall_active;    /* C3 tracking flag */
} ApogeeDetector_t;

void   APOGEE_Init(ApogeeDetector_t *det);

/* Notify detector that flight phase entered COAST */
void   APOGEE_OnCoastEntry(ApogeeDetector_t *det, uint32_t now_ms);

/*
 * Call every IMU sample during COAST phase.
 * velocity     : raw (unfiltered) vertical velocity from dead reckoning (m/s).
 *                Raw velocity is used here — LPF would add 10-20 ms lag,
 *                violating the ≤10 ms detection requirement.
 * a_net        : net vertical acceleration (m/s²)
 * phase        : current flight phase
 * now_ms       : current tick (ms)
 *
 * Returns true when apogee is detected (latches — stays true afterwards).
 */
bool   APOGEE_Update(ApogeeDetector_t *det,
                     float velocity, float a_net,
                     FlightPhase_t phase, uint32_t now_ms);

/*
 * Dual-IMU voting: fire apogee if:
 *   - Both detectors agree, OR
 *   - One detector fired and the other is marked as failed, OR
 *   - Safety timer on either detector expired
 */
bool   APOGEE_Vote(const ApogeeDetector_t *det_a,
                   const ApogeeDetector_t *det_b);

#endif /* APOGEE_DETECTOR_H */
