/**
 * @author Enrico Dalla Croce (Kalsifer-742)
 * @date 30/11/2023
 */

#include "can_messages.h"

#define BMS_LV_MSG_HANDLER(ntw, NTW, msg_name, MSG_NAME)                       \
  ntw##_##msg_name##_t raw;                                                    \
  ntw##_##msg_name##_converted_t converted;                                    \
  ntw##_##msg_name##_unpack(&raw, msg->data, NTW##_##MSG_NAME##_BYTE_SIZE);    \
  ntw##_##msg_name##_raw_to_conversion_struct(&converted, &raw);               \
  can##_##msg_name##_##handler(converted);

void can_set_radiator_speed_handler(
    primary_set_radiator_speed_converted_t converted);
void can_set_pumps_speed_handler(primary_set_pumps_speed_converted_t converted);
void can_set_inverter_connection_status_handler(
    primary_set_inverter_connection_status_converted_t converted);
void can_lv_flash_req_handler();
void can_bms_lv_jmp_to_blt_handler();
void can_ts_status_handler(primary_ts_status_converted_t converted);
void can_inv_l_rcv_handler(inverters_inv_l_rcv_converted_t converted);
void can_inv_r_rcv_handler(inverters_inv_r_rcv_converted_t converted);

extern int primary_can_id;
extern int secondary_can_id;

void can_routine(void) {
  consume_rx_queue(primary_can_id);
  consume_rx_queue(secondary_can_id);
  flush_tx_queue(primary_can_id);
  flush_tx_queue(secondary_can_id);
}

void can_init_errors_handler(int can_manager_error_code) {
  if (can_manager_error_code == CAN_MGR_FILTER_ERROR_CODE) {
    // TO-DO
  } else if (can_manager_error_code == CAN_MGR_CAN_INIT_IT_ERROR_CODE) {
    // TO-DO
  } else if (can_manager_error_code == CAN_MGR_CAN_START_ERROR_CODE) {
    // TO-DO
  }
}

void can_primary_ntw_handler(can_manager_message_t *msg) {
  switch (msg->id) {
  case PRIMARY_SET_RADIATOR_SPEED_FRAME_ID: {
    BMS_LV_MSG_HANDLER(primary, PRIMARY, set_radiator_speed,
                       SET_RADIATOR_SPEED);
    break;
  }
  case PRIMARY_SET_PUMPS_SPEED_FRAME_ID: {
    BMS_LV_MSG_HANDLER(primary, PRIMARY, set_pumps_speed, SET_PUMPS_SPEED);
    break;
  }
  case PRIMARY_SET_INVERTER_CONNECTION_STATUS_FRAME_ID: {
    BMS_LV_MSG_HANDLER(primary, PRIMARY, set_inverter_connection_status,
                       SET_INVERTER_CONNECTION_STATUS);
    break;
  }
  case PRIMARY_LV_CAN_FLASH_REQ_FRAME_ID: {
    can_lv_flash_req_handler();
    break;
  }
  case PRIMARY_BMS_LV_JMP_TO_BLT_FRAME_ID: {
    can_bms_lv_jmp_to_blt_handler();
    break;
  }
  case PRIMARY_TS_STATUS_FRAME_ID: {
    BMS_LV_MSG_HANDLER(primary, PRIMARY, ts_status, TS_STATUS);
    break;
  }
  case INVERTERS_INV_L_RCV_FRAME_ID: {
    BMS_LV_MSG_HANDLER(inverters, INVERTERS, inv_l_rcv, INV_L_RCV);
    break;
  }
  case INVERTERS_INV_R_RCV_FRAME_ID: {
    BMS_LV_MSG_HANDLER(inverters, INVERTERS, inv_r_rcv, INV_R_RCV);
    break;
  }

  default:
    // TO-DO
    break;
  }
}

void can_secondary_ntw_handler(can_manager_message_t *msg) {
  // The old code doesn't handle any messages from the secondary network
}

void can_set_radiator_speed_handler(
    primary_set_radiator_speed_converted_t converted) {
  // TO-DO
}

void can_set_pumps_speed_handler(
    primary_set_pumps_speed_converted_t converted) {
  // TO-DO
}

void can_set_inverter_connection_status_handler(
    primary_set_inverter_connection_status_converted_t converted) {
  // TO-DO
}

void can_lv_flash_req_handler() {
  // TO-DO
}

void can_bms_lv_jmp_to_blt_handler() {
  // TO-DO
}

void can_ts_status_handler(primary_ts_status_converted_t converted) {
  // TO-DO
}

void can_inv_l_rcv_handler(inverters_inv_l_rcv_converted_t converted) {
  // TO-DO
}

void can_inv_r_rcv_handler(inverters_inv_r_rcv_converted_t converted) {
  // TO-DO
}