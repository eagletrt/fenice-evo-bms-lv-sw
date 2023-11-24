#include "can_manager.h"
#include "primary_network.h"
#include "inverters_network.h"

#ifndef can_messages_handling_h
#define can_messages_handling_h

#define BMS_LV_CAN_UNPACK_AND_HANDLER(ntw, NTW, msg_name, MSG_NAME)           \
  ntw##_##msg_name##_t raw;                                                    \
  ntw##_##msg_name##_converted_t converted;                                    \
  ntw##_##msg_name##_unpack(&raw, msg->data, NTW##_##MSG_NAME##_BYTE_SIZE);    \
  ntw##_##msg_name##_raw_to_conversion_struct(&converted, &raw);               \
  can##_##msg_name##_##handler(converted);                                     \       

void can_init_errors_handler(int can_manager_error_code);

void can_primary_ntw_handler(can_manager_message_t* msg);
void can_secondary_ntw_handler(can_manager_message_t* msg);

void can_set_radiator_speed_handler(primary_set_radiator_speed_converted_t converted);
void can_set_pumps_speed_handler(primary_set_pumps_speed_converted_t converted);
void can_set_inverter_connection_status_handler(primary_set_inverter_connection_status_converted_t converted);
void can_lv_flash_req_handler();
void can_bms_lv_jmp_to_blt_handler();
void can_ts_status_handler(primary_ts_status_converted_t converted);
void can_inv_l_rcv_handler(inverters_inv_l_rcv_converted_t converted);
void can_inv_r_rcv_handler(inverters_inv_r_rcv_converted_t converted);



#endif