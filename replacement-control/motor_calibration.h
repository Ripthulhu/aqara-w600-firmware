#ifndef MOTOR_CALIBRATION_H
#define MOTOR_CALIBRATION_H
#include "motor_homing.h"
#include "motor_slowdown.h"

enum motor_calibration_state {
    MOTOR_CAL_IDLE, MOTOR_CAL_ACTIVE, MOTOR_CAL_WAIT_STOP,
    MOTOR_CAL_CANDIDATE, MOTOR_CAL_FAILED, MOTOR_CAL_RETRACTED
};
enum motor_calibration_phase {
    MOTOR_CAL_NO_PHASE, MOTOR_CAL_PROBE_CLOSE, MOTOR_CAL_PROBE_OPEN,
    MOTOR_CAL_HOME, MOTOR_CAL_MEASURE, MOTOR_CAL_CONFIRM_BACKOFF, MOTOR_CAL_CONFIRM_RETURN
};
enum motor_calibration_direction { MOTOR_CAL_NONE, MOTOR_CAL_OPEN, MOTOR_CAL_CLOSE };
enum motor_calibration_error {
    MOTOR_CAL_OK, MOTOR_CAL_BAD_POLICY, MOTOR_CAL_BAD_STATE, MOTOR_CAL_CLOCK,
    MOTOR_CAL_ATTEMPT_TIMEOUT, MOTOR_CAL_HOMING, MOTOR_CAL_COUNTER,
    MOTOR_CAL_ENERGY_LIMIT, MOTOR_CAL_PROBE_SIGNAL, MOTOR_CAL_NO_MOTION,
    MOTOR_CAL_STOP_TIMEOUT, MOTOR_CAL_REFERENCE_UNQUALIFIED,
    MOTOR_CAL_TRAVEL_RANGE, MOTOR_CAL_REPEATABILITY, MOTOR_CAL_TOKEN_EXHAUSTED,
    MOTOR_CAL_ABORTED
};
struct motor_calibration_policy {
    struct motor_homing_policy homing[2]; /* OPEN, CLOSE: direction-1, no normalization assumed. */
    uint32_t whole_attempt_ticks, probe_on_ticks, probe_slack_ticks;
    uint32_t min_travel_ticks, max_travel_ticks, repeat_tolerance_ticks;
    uint32_t stop_confirm_ticks, min_stop_ticks;
};
struct motor_calibration_action {
    uint32_t token, on_budget_ticks;
    uint8_t direction;
};
struct motor_calibration_candidate {
    uint32_t opening_ticks, closing_ticks;
    uint16_t final_endpoint_permille;
};
/* Optional calibration-only confirmation. Durations use the sequencer's tick
 * unit (100us in firmware). Set only while IDLE; zero preserves legacy homing. */
struct motor_calibration_confirmation {
    uint32_t max_sample_gap_ticks, backoff_ticks, backoff_limit_ticks;
    uint32_t return_ticks, return_limit_ticks, rest_ticks;
};
struct motor_calibration {
    struct motor_homing homing;
    struct motor_calibration_policy policy;
    struct motor_calibration_action action;
    struct motor_calibration_candidate candidate;
    uint32_t samples[2][2], final_on_samples[2][2], sample_tokens[2][2];
    uint32_t last_token, attempt_started_at, phase_started_at, last_now;
    uint32_t stop_requested_at, stop_observed_at, stopped_on_ticks;
    uint32_t on_ticks, last_on_at, last_signal, last_sample_on, first_low_on;
    enum motor_calibration_state state;
    enum motor_calibration_phase phase;
    enum motor_calibration_error error;
    uint8_t direction, measured_legs, stop_reason;
    bool intentional_abort, have_sample_on, have_first_low, have_stop_proof;
    struct motor_calibration_confirmation confirmation;
    uint32_t intermediate_started_at, pattern_last_at;
    uint32_t original_token, original_first_low, original_final_on;
    enum motor_calibration_phase original_phase;
    uint8_t original_direction;
    uint16_t intermediate_count;
    bool pattern_high, pattern_have_sample;
    uint32_t confirmed_candidates, confirmed_return_on, confirmed_return_mv;
    uint16_t original_intermediate_count, confirmed_intermediate_count;
    struct motor_slowdown slowdown;
    bool retract_only;
    uint32_t closing_load_at;
    bool closing_load_seen;
    uint8_t measured_close_kind;
    struct motor_seat seat;
    bool original_seat;
    bool original_load;
    uint16_t original_load_mv;
};

/* Pure sequencer: no start callback, MMIO, persistence or position reference.
 * Serialize calls; independent tick must keep running during FG/ADC stalls.
 * All timestamps AND cumulative ENERGIZED counts use one tick unit. Off/settle
 * time is excluded from on_ticks. current cumulative_on_ticks is captured at
 * now; sample_on_ticks is captured WITH the sample at measured_at, not receipt.
 * These are truthful driver counts, not OEM elapsed-time registers. Feedback
 * must come from a physically
 * qualified sensing/power phase; ADC conversion success alone is not validity.
 * Each action token may be consumed once by a future bounded driver. A failed
 * or busy start never renews the attempt. Never energize without an ACTIVE
 * action and its armed homing guard. A CANDIDATE is NOT calibrated/position-valid.
 * Means use the energized count at the FIRST low sample of the qualified streak,
 * not later confirmation/stop energy (recorded separately). No guessed timing
 * subtraction or compensation. A motion-qualified probe continues as HOME with
 * the same token, counters and original move deadline; it does not reverse or
 * restart. An early qualified endpoint also completes HOME, never a learned leg.
 * Probe target crossings may overshoot only by
 * the explicit probe_slack_ticks; actual counts must never be clamped. Caller
 * validates repeated travel and qualified endpoints before referencing position. Candidate
 * fields are usable only in state MOTOR_CAL_CANDIDATE. */
bool motor_calibration_init(struct motor_calibration *, void (*stop)(void *), void *context);
bool motor_calibration_start(struct motor_calibration *, uint32_t now,
                             const struct motor_calibration_policy *, bool proven_stopped);
/* One inward HOME plus the existing backoff/return confirmation; no learned travel. */
bool motor_calibration_retract_start(struct motor_calibration *, uint32_t now,
                             const struct motor_calibration_policy *, bool proven_stopped);
void motor_calibration_tick(struct motor_calibration *, uint32_t now);
bool motor_calibration_foreground_progress(struct motor_calibration *, uint32_t token,
                                           uint32_t now, uint32_t cumulative_on_ticks);
bool motor_calibration_feedback(struct motor_calibration *, uint32_t token, uint32_t now,
                                uint32_t measured_at, uint32_t value, bool valid,
                                uint32_t sample_on_ticks,
                                uint32_t cumulative_on_ticks);
/* First proven stop records observed-off time/final count and returns false;
 * it never advances. A second fresh proof after min_stop_ticks advances only
 * with IDENTICAL energized count. Early second/unproven confirmations reject
 * unchanged unless a deadline expires. Stop-request deadline never renews.
 * Without confirmation, HOME/MEASURE needs the driver's independently qualified
 * multi-low endpoint. With confirmation enabled, the flag instead attests a
 * healthy driver with qualified motion and the pre-stop low for this action;
 * the sequencer additionally requires its pre-stop band transition, one bounded
 * backoff and matching return transition before crediting the ORIGINAL leg.
 * No post-stop zeros or confirmation travel enter the four measured legs.
 * This remains electrical evidence, not an encoder or mounted-force measurement.
 * Stale phase tokens reject unchanged; no automatic retry exists. */
bool motor_calibration_confirm_stop(struct motor_calibration *, uint32_t token,
                                    uint32_t now, uint32_t final_on_ticks,
                                    bool proven_stopped, bool qualified_endpoint);
/* Required proved-off dwell before consuming/rearming the current action. */
uint32_t motor_calibration_stop_dwell(const struct motor_calibration *);
void motor_calibration_abort(struct motor_calibration *);
/* Explicit reset only in IDLE/terminal + proven stop; preserves tokens. */
bool motor_calibration_reset(struct motor_calibration *, bool proven_stopped);
#endif
