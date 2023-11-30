/**
 * @author Enrico Dalla Croce (Kalsifer-742)
 * @date 30/11/2023
*/

#include "can_manager.h"
#include "primary_network.h"
#include "inverters_network.h"

#ifndef can_messages_h
#define can_messages_h

#define BMS_LV_MSG_HANDLER(ntw, NTW, msg_name, MSG_NAME)                       \
  ntw##_##msg_name##_t raw;                                                    \
  ntw##_##msg_name##_converted_t converted;                                    \
  ntw##_##msg_name##_unpack(&raw, msg->data, NTW##_##MSG_NAME##_BYTE_SIZE);    \
  ntw##_##msg_name##_raw_to_conversion_struct(&converted, &raw);               \
  can##_##msg_name##_##handler(converted);                                     \  

void can_init_errors_handler(int can_manager_error_code);
void can_primary_ntw_handler(can_manager_message_t* msg);
void can_secondary_ntw_handler(can_manager_message_t* msg);

#endif