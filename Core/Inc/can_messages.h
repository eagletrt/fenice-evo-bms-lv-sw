#ifndef CAN_MESSAGES_H
#define CAN_MESSAGES_H

#include "bms_lv_config.h"
#include "can_types.h"
#include "primary_network.h"
#include "primary_watchdog.h"

#define CANLIB_UNPACK_AND_UPDATE_MSG(ntw, NTW, msg_name, MSG_NAME)             \
  ntw##_##msg_name##_t raw;                                                    \
  ntw##_##msg_name##_converted_t converted;                                    \
  ntw##_##msg_name##_unpack(&raw, msg->data, NTW##_##MSG_NAME##_BYTE_SIZE);    \
  ntw##_##msg_name##_raw_to_conversion_struct(&converted, &raw);               \
  ntw##_##msg_name##_converted_t *last_state =                                 \
      (ntw##_##msg_name##_converted_t                                          \
           *)&ntw##_messages_last_state[ntw##_index_from_id(msg->id)][0];      \
  memcpy(last_state, &converted, sizeof(ntw##_##msg_name##_converted_t));

#define CANLIB_PACK_MSG(ntw, NTW, msg_name, MSG_NAME)                          \
  can_manager_message_t msg = {0};                                             \
  msg.id = NTW##_##MSG_NAME##_FRAME_ID;                                        \
  msg.size = NTW##_##MSG_NAME##_BYTE_SIZE;                                     \
  ntw##_##msg_name##_t raw = {0};                                              \
  ntw##_##msg_name##_conversion_to_raw_struct(&raw, &converted);               \
  ntw##_##msg_name##_pack(msg.data, &raw, PRIMARY_##MSG_NAME##_BYTE_SIZE);

void can_routine(void);
void can_primary_ntw_handler(can_manager_message_t *msg);
void can_messages_callbacks_init();
void can_init_errors_handler(int can_mgr_error_code);

#endif // CAN_MESSAGES_H
