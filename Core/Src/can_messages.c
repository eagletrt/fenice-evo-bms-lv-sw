#include "can_messages.h"
#include <string.h>

extern int primary_can_id;

void can_routine(void) {
  consume_rx_queue(primary_can_id);
  // flush_tx_queue(primary_can_id);
  // TODO: check the timers to send can messages
}

/**
 * TODO: Feature request canlib, the maximum length to store all the messages of
 * the canlib without doing this very ugly multiplication
 * TODO: incorporate all of this stuff in the can manager
 */
uint8_t primary_messages_last_state[primary_MESSAGE_COUNT]
                                   [primary_MAX_STRUCT_SIZE_CONVERSION];

int (*primary_message_handlers[primary_MESSAGE_COUNT])(can_manager_message_t *);

void can_init_errors_handler(int can_mgr_error_code) {}

// TODO add support for return values (and error handling) in can manager
void can_primary_ntw_handler(can_manager_message_t *msg) {
  int index = primary_index_from_id(msg->id);
  if (index != -1 && index < primary_MESSAGE_COUNT &&
      primary_message_handlers[index] != NULL) {
    (*primary_message_handlers[index])(msg);
  }
}

int primary_car_status_handler(can_manager_message_t *msg) {
  CANLIB_UNPACK_AND_UPDATE_MSG(primary, PRIMARY, car_status, CAR_STATUS);
  return 0;
}

int primary_set_inverter_connection_status_handler(can_manager_message_t *msg) {
  CANLIB_UNPACK_AND_UPDATE_MSG(primary, PRIMARY, set_inverter_connection_status,
                               SET_INVERTER_CONNECTION_STATUS);
  return 0;
}

int primary_hv_feedback_sd_voltage_handler(can_manager_message_t *msg) {
  CANLIB_UNPACK_AND_UPDATE_MSG(primary, PRIMARY, hv_feedback_sd_voltage,
                               HV_FEEDBACK_SD_VOLTAGE);
  // hv_feedback_sd_voltage_new_state
  return 0;
}

void can_messages_callbacks_init() {
  for (int idx = 0; idx < primary_MESSAGE_COUNT; idx++) {
    primary_message_handlers[idx] = NULL;
  }
  primary_message_handlers[PRIMARY_CAR_STATUS_INDEX] =
      primary_car_status_handler;
  primary_message_handlers[PRIMARY_SET_INVERTER_CONNECTION_STATUS_INDEX] =
      primary_set_inverter_connection_status_handler;
  primary_message_handlers[PRIMARY_HV_FEEDBACK_SD_VOLTAGE_INDEX] =
      primary_hv_feedback_sd_voltage_handler;
}

void primary_lv_feedbacks_send() {
  primary_lv_feedbacks_converted_t converted = {

  };
  CANLIB_PACK_MSG(primary, PRIMARY, lv_feedbacks, LV_FEEDBACKS);
  can_send(primary_can_id, &msg);
}
