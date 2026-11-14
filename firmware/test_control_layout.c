#include <stddef.h>
#include "valve_calibration.h"
#include "settings_app.h"
#include "radio.h"
#include "stop_mode.h"
/* Test-fixture offsets emitted by the ARM compiler, never linked into firmware. */
const uint32_t w600_test_control_layout[]={
    offsetof(struct w600_valve_calibration,position),
    offsetof(struct w600_valve_calibration,calibration_applied),
    offsetof(struct w600_endpoint,control_error)
};
#define SEQ(member) (offsetof(struct w600_valve_calibration,sequence)+offsetof(struct motor_calibration,member))
/* Separate optional array: existing three-word control-layout ABI stays intact.
 * All offsets below are relative to the valve symbol, not guessed SRAM addresses. */
const uint32_t w600_test_confirmation_layout[]={
    1,sizeof(struct w600_valve_calibration),sizeof(struct motor_calibration),
    SEQ(state),SEQ(phase),SEQ(error),SEQ(action),SEQ(attempt_started_at),SEQ(last_token),
    SEQ(first_low_on),SEQ(on_ticks),SEQ(confirmed_candidates),SEQ(original_phase),
    SEQ(original_direction),SEQ(original_token),SEQ(original_first_low),SEQ(original_final_on),
    SEQ(confirmed_return_on),SEQ(confirmed_return_mv),SEQ(original_intermediate_count),
    SEQ(confirmed_intermediate_count),offsetof(struct w600_valve_calibration,position),
    offsetof(struct w600_valve_calibration,calibration_applied)
};
const uint32_t w600_test_settings_layout[]={
    sizeof(struct w600_settings_service),
    offsetof(struct w600_settings_service,attempts),offsetof(struct w600_settings_service,saves),
    offsetof(struct w600_settings_service,status),sizeof(enum w600_settings_status),
    offsetof(struct w600_settings_service,initialized),offsetof(struct w600_settings_service,dirty),
    offsetof(struct w600_settings_service,blocked),
    offsetof(struct w600_endpoint,known),offsetof(struct w600_endpoint,mode),
    offsetof(struct w600_endpoint,calibrated),offsetof(struct w600_endpoint,opening_known),
    offsetof(struct w600_endpoint,moving),offsetof(struct w600_endpoint,external_temp_raw),
    offsetof(struct w600_endpoint,local_temp_raw)
};
const uint32_t w600_test_radio_diagnostics_layout[]={
    1,sizeof(struct w600_radio),sizeof(struct w600_stop_report),
    offsetof(struct w600_radio,target_change_count),offsetof(struct w600_radio,last_target_change_at),
    offsetof(struct w600_stop_report,radio_wakes),offsetof(struct w600_stop_report,button_wakes),
    offsetof(struct w600_stop_report,timer_wakes),offsetof(struct w600_stop_report,last_radio_at),
    offsetof(struct w600_radio,last_rx_at)
};
