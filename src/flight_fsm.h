#ifndef FLIGHT_FSM_H
#define FLIGHT_FSM_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    PHASE_PAD_STATIC = 0,   /* Pre-launch: rocket on pad, calibrating */
    PHASE_BOOST,            /* Motor burning: high acceleration */
    PHASE_COAST,            /* Motor out: coasting to apogee */
    PHASE_APOGEE,           /* Apogee detected: ejection charge fired */
    PHASE_DESCEND,          /* Falling */
    PHASE_COUNT
} FlightPhase_t;

typedef struct {
    FlightPhase_t phase;
    uint32_t      phase_entry_ms;
    uint32_t      launch_debounce_ms;
    uint32_t      burnout_debounce_ms;
    bool          launch_debounce_active;
    bool          burnout_debounce_active;
} FSM_State_t;

void          FSM_Init(FSM_State_t *fsm);
FlightPhase_t FSM_Update(FSM_State_t *fsm,
                         float a_net_ms2,   /* net vertical accel (m/s²) */
                         float velocity,    /* vertical velocity (m/s) */
                         bool  apogee_vote, /* from dual-IMU apogee detector */
                         uint32_t now_ms);

#endif /* FLIGHT_FSM_H */
