#ifndef CAN_MANAGER_CONFIG_H
#define CAN_MANAGER_CONFIG_H

#define CAN_MGR_STM32_APPLICATION
#define CAN_MGR_N_CAN 2
#define CAN_MGR_TOTAL_CAN_RX_FIFOS 2
#define CAN_MGR_CAN_WAIT_ENABLED 0

#define N_MONITORED_MESSAGES 3
#define CAN_MESSAGES_MAPPING                                                   \
  {                                                                            \
    PRIMARY_CAR_STATUS_FRAME_ID,                                               \
        PRIMARY_SET_INVERTER_CONNECTION_STATUS_FRAME_ID,                       \
        PRIMARY_HV_FEEDBACK_SD_VOLTAGE_FRAME_ID                                \
  }
#define CAN_MESSAGES_HANDLERS                                                  \
  {                                                                            \
    primary_car_status_handler,                                                \
        primary_set_inverter_connection_status_handler,                        \
        primary_hv_feedback_sd_voltage_handler                                 \
  }

#endif // CAN_MANAGER_CONFIG_H
