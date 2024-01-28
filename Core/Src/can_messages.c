#include "can_messages.h"
#include <string.h>

extern int primary_can_id;

void can_routine(void) {
  consume_rx_queue(primary_can_id);
  flush_tx_queue(primary_can_id);
}

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

void can_messages_callbacks_init() {
  // memset(primary_message_handlers, 0, primary_MESSAGE_COUNT * sizeof(void
  // *)); // TODO check this is correct
  primary_message_handlers[PRIMARY_CAR_STATUS_INDEX] =
      primary_car_status_handler;
  primary_message_handlers[PRIMARY_SET_INVERTER_CONNECTION_STATUS_INDEX] =
      primary_set_inverter_connection_status_handler;
}

void primary_lv_feedbacks_send() {
  primary_lv_feedbacks_converted_t converted = {0};
  CANLIB_PACK_MSG(primary, PRIMARY, lv_feedbacks, LV_FEEDBACKS);
  can_send(primary_can_id, &msg);
}
