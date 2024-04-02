#ifndef CAN_MANAGER_CONFIG_H
#define CAN_MANAGER_CONFIG_H

#define CAN_MGR_STM32_APPLICATION
#define CAN_MGR_N_CAN 2
#define CAN_MGR_TOTAL_CAN_RX_FIFOS 2
#define CAN_MGR_CAN_WAIT_ENABLED 0

#define N_MONITORED_MESSAGES 7
#define CAN_MESSAGES_MAPPING                                                   \
  {                                                                            \
    PRIMARY_SET_INVERTER_CONNECTION_STATUS_FRAME_ID,                           \
        PRIMARY_SET_RADIATOR_SPEED_FRAME_ID, PRIMARY_SET_PUMPS_SPEED_FRAME_ID, \
        PRIMARY_TS_STATUS_FRAME_ID, INVERTERS_INV_L_RCV_FRAME_ID,              \
        INVERTERS_INV_R_RCV_FRAME_ID,                                          \
        PRIMARY_LV_CAN_FLASH_REQ_STEERING_WHEEL_FRAME_ID                       \
  }
#define CAN_MESSAGES_HANDLERS                                                  \
  {                                                                            \
    primary_set_inverter_connection_status_handler,                            \
        primary_set_radiator_speed_handler, primary_set_pumps_speed_handler,   \
        primary_ts_status_handler, inverters_l_handler, inverters_r_handler,   \
        primary_flash_request_handler                                          \
  }

#endif // CAN_MANAGER_CONFIG_H
