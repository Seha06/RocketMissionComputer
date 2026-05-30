#ifndef APOGEE_DETECTOR_H
#define APOGEE_DETECTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "../flight_fsm.h"

/*
 * Per-IMU apogee detector — three independent criteria (any one fires ejection).
 *
 * C1 PRIMARY  : raw DR velocity < APOGEE_VEL_THRESHOLD for N_CONSEC samples.
 *               Fast response (~49 ms worst-case for M2020/18 kg profile).
 *
 * C2 TIMER    : coast elapsed time > APOGEE_COAST_MAX_MS.
 *               Absolute safety backstop; fires even if both IMUs are degraded.
 *
 * C3 SUSTAINED: raw DR velocity < APOGEE_VEL_SECONDARY continuously for
 *               APOGEE_SUSTAINED_MS.  Immune to noise spikes; fires if C1
 *               oscillates around threshold without reaching N_CONSEC.
 *
 * *** Design note on free-fall: ***
 * Throughout the entire coast phase the rocket is in free-fall, so the
 * accelerometer specific force ≈ 0 everywhere in coast — not only at apogee.
 * A specific-force criterion cannot distinguish apogee from general coast and
 * is therefore not used.  C1 and C3 both operate on integrated DR velocity.
 *
 * Dual-IMU voting via APOGEE_Vote():
 *   Both healthy  → require both to agree.
 *   One failed    → single healthy IMU decides.
 *   Both failed   → C2 timer provides the last resort.
 */
typedef struct {
    int8_t   consec_count;        /* C1: consecutive sub-threshold sample count */
    bool     apogee_fired;        /* latched true once apogee is declared */
    bool     imu_failed;          /* watchdog: set when IMU silent > IMU_WATCHDOG_MS */
    uint32_t coast_start_ms;      /* timestamp when COAST phase entered */
    uint32_t sustained_entry_ms;  /* C3: timestamp when sustained-drop window opened */
    bool     sustained_active;    /* C3: velocity has been below secondary threshold */
} ApogeeDetector_t;

void APOGEE_Init(ApogeeDetector_t *det);

/* Call when the flight FSM transitions into PHASE_COAST. */
void APOGEE_OnCoastEntry(ApogeeDetector_t *det, uint32_t now_ms);

/*
 * Call every IMU sample while in PHASE_COAST.
 *
 *   velocity : raw (unfiltered) DR vertical velocity (m/s).
 *              Raw is used to avoid LPF lag violating timing budget.
 *   phase    : current flight phase.
 *   now_ms   : system tick (ms).
 *
 * Returns true when apogee declared (latched — stays true after first fire).
 */
bool APOGEE_Update(ApogeeDetector_t *det,
                   float velocity,
                   FlightPhase_t phase,
                   uint32_t now_ms);

/*
 * Dual-IMU vote.  Result drives the flight FSM.
 *   Both healthy  → (det_a->apogee_fired && det_b->apogee_fired)
 *   One failed    → surviving IMU decides
 *   Both failed   → false (C2 timer inside each detector still fires independently)
 */
bool APOGEE_Vote(const ApogeeDetector_t *det_a,
                 const ApogeeDetector_t *det_b);

#endif /* APOGEE_DETECTOR_H */
