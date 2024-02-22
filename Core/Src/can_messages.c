#include "can_messages.h"
#include <string.h>

extern int bms_lv_primary_can_id;
extern can_mgr_msg_t can_messages_states[N_MONITORED_MESSAGES];
extern uint8_t can_messages_is_new[N_MONITORED_MESSAGES];

int primary_car_status_handler(can_mgr_msg_t *msg);
int primary_set_inverter_connection_status_handler(can_mgr_msg_t *msg);
int primary_hv_feedback_sd_voltage_handler(can_mgr_msg_t *msg);

int (*primary_message_handlers[primary_MESSAGE_COUNT])(can_mgr_msg_t *) =
    CAN_MESSAGES_HANDLERS;

void can_init_errors_handler(int can_mgr_error_code) {
  // TODO: manage errors
}

int can_mgr_from_id_to_index(int can_id, int msg_id) {
  if (can_id != bms_lv_primary_can_id)
    return -1;
  switch (msg_id) {
  case PRIMARY_CAR_STATUS_FRAME_ID:
    return 0;
  case PRIMARY_SET_INVERTER_CONNECTION_STATUS_FRAME_ID:
    return 1;
  case PRIMARY_HV_FEEDBACK_SD_VOLTAGE_FRAME_ID:
    return 2;
  default:
    return -1;
  }
  return -1;
}

int can_start(void) {
  if (can_mgr_start(bms_lv_primary_can_id) < 0) {
    // Handle Error
  }
  return 0;
}

int can_routine(void) {
  for (size_t msg_idx = 0; msg_idx < N_MONITORED_MESSAGES; ++msg_idx) {
    if (can_messages_is_new[msg_idx] &&
        (primary_message_handlers[msg_idx] != NULL)) {
      (*primary_message_handlers[msg_idx])(&can_messages_states[msg_idx]);
    }
  }
  return 0;
}

int primary_car_status_handler(can_mgr_msg_t *msg) { return 0; }

int primary_set_inverter_connection_status_handler(can_mgr_msg_t *msg) {
  return 0;
}

int primary_hv_feedback_sd_voltage_handler(can_mgr_msg_t *msg) { return 0; }

int primary_lv_feedbacks_send(void) { return 0; }
