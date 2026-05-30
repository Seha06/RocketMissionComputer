#include "flight_fsm.h"
#include "ekf_config.h"
#include <string.h>

void FSM_Init(FSM_State_t *fsm)
{
    memset(fsm, 0, sizeof(*fsm));
    fsm->phase = PHASE_PAD_STATIC;
}

FlightPhase_t FSM_Update(FSM_State_t *fsm,
                         float a_net_ms2,
                         bool  apogee_vote,
                         uint32_t now_ms)
{
    switch (fsm->phase) {

    /* ------------------------------------------------------------------ */
    case PHASE_PAD_STATIC:
        if (a_net_ms2 >= LAUNCH_ACCEL_THRESH) {
            if (!fsm->launch_debounce_active) {
                fsm->launch_debounce_ms    = now_ms;
                fsm->launch_debounce_active = true;
            }
            if ((now_ms - fsm->launch_debounce_ms) >= LAUNCH_DEBOUNCE_MS) {
                fsm->phase           = PHASE_BOOST;
                fsm->phase_entry_ms  = now_ms;
                fsm->launch_debounce_active = false;
            }
        } else {
            fsm->launch_debounce_active = false;
        }
        break;

    /* ------------------------------------------------------------------ */
    case PHASE_BOOST:
        /*
         * Burnout detected when acceleration drops below threshold.
         * Debounce prevents motor chuff (brief thrust interruptions) from
         * triggering a false coast entry.
         */
        if (a_net_ms2 < BURNOUT_ACCEL_THRESH) {
            if (!fsm->burnout_debounce_active) {
                fsm->burnout_debounce_ms    = now_ms;
                fsm->burnout_debounce_active = true;
            }
            if ((now_ms - fsm->burnout_debounce_ms) >= BURNOUT_DEBOUNCE_MS) {
                fsm->phase           = PHASE_COAST;
                fsm->phase_entry_ms  = now_ms;
                fsm->burnout_debounce_active = false;
            }
        } else {
            fsm->burnout_debounce_active = false;
        }
        /*
         * Safety timeout: if PHASE_BOOST has lasted longer than BOOST_MAX_MS
         * the sensor data may be stale (IMU failed mid-burn with a high
         * specific-force reading latched) or the motor has certainly burned out.
         * Force coast entry so that the C2 absolute timer can start and
         * apogee detection is not permanently blocked.
         */
        if ((now_ms - fsm->phase_entry_ms) >= BOOST_MAX_MS) {
            fsm->phase                   = PHASE_COAST;
            fsm->phase_entry_ms          = now_ms;
            fsm->burnout_debounce_active  = false;
        }
        break;

    /* ------------------------------------------------------------------ */
    case PHASE_COAST:
        /*
         * Chuff recovery: if thrust-level acceleration reappears, the motor
         * is still burning (false burnout from a pressure oscillation that
         * lasted longer than BURNOUT_DEBOUNCE_MS).  Revert to BOOST so that
         * the apogee detector stays gated off and burnout re-detection can
         * proceed cleanly.
         */
        if (a_net_ms2 >= LAUNCH_ACCEL_THRESH) {
            fsm->phase          = PHASE_BOOST;
            fsm->phase_entry_ms = now_ms;
            fsm->burnout_debounce_active = false;
            break;
        }
        if (apogee_vote) {
            fsm->phase          = PHASE_APOGEE;
            fsm->phase_entry_ms = now_ms;
        }
        break;

    /* ------------------------------------------------------------------ */
    case PHASE_APOGEE:
        /* Transition to DESCEND after ejection dwell period */
        if ((now_ms - fsm->phase_entry_ms) >= APOGEE_DESCEND_DELAY_MS) {
            fsm->phase          = PHASE_DESCEND;
            fsm->phase_entry_ms = now_ms;
        }
        break;

    /* ------------------------------------------------------------------ */
    case PHASE_DESCEND:
        /* Terminal state — no further transitions in this module */
        break;

    default:
        break;
    }

    return fsm->phase;
}
