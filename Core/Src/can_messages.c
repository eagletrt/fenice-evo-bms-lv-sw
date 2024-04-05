#include "can_messages.h"

#include "bms_fsm.h"
#include "dac_pump.h"
#include "inverter_conversions.h"
#include "lv_errors.h"
#include "mcp23017.h"
#include "radiator.h"
#include "spi.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include <string.h>

extern int bms_lv_primary_can_id;
extern can_mgr_msg_t can_messages_states[N_MONITORED_MESSAGES];
extern uint8_t can_messages_is_new[N_MONITORED_MESSAGES];
static float inv_temps[2] = {0};
primary_hv_status_converted_t ts_status_converted;
primary_ecu_status_converted_t ecu_status;
// primary_set_inverter_connection_status_converted_t set_inv_connection_status
// = {
//     0};

int primary_set_inverter_connection_status_handler(can_mgr_msg_t *msg);
int primary_set_radiator_speed_handler(can_mgr_msg_t *msg);
int primary_set_pumps_speed_handler(can_mgr_msg_t *msg);
int primary_ts_status_handler(can_mgr_msg_t *msg);
int inverters_l_handler(can_mgr_msg_t *msg);
int inverters_r_handler(can_mgr_msg_t *msg);
int primary_flash_request_handler(can_mgr_msg_t *msg);
int primary_ecu_status_handler(can_mgr_msg_t *msg);

void primary_lv_cells_voltage_send(void);
void primary_lv_cells_temp_send(void);
void primary_lv_total_voltage_send(void);
void primary_lv_status_send(void);
void primary_lv_errors_send(void);
void primary_lv_current_charger_send(void);
void primary_lv_current_battery_send(void);
void primary_lv_feedback_sd_send(void);
void primary_lv_feedback_enclosure_send(void);
void primary_lv_feedback_ts_send(void);
void primary_lv_feedback_gpio_send(void);
void primary_inverter_connection_status_send(void);
void primary_lv_version_send(void);
void primary_lv_pump_speed_send(void);
void primary_lv_radiator_speed_send(void);

int (*primary_message_handlers[N_MONITORED_MESSAGES])(can_mgr_msg_t *) =
    CAN_MESSAGES_HANDLERS;

void can_init_errors_handler(int can_mgr_error_code) {
  // TODO: manage errors
}

int can_mgr_from_id_to_index(int can_id, int msg_id) {
  if (can_id != bms_lv_primary_can_id)
    return -1;
  switch (msg_id) {
  case PRIMARY_LV_SET_INVERTER_CONNECTION_STATUS_FRAME_ID:
    return 0;
  case PRIMARY_LV_SET_RADIATOR_SPEED_FRAME_ID:
    return 1;
  case PRIMARY_LV_SET_PUMPS_SPEED_FRAME_ID:
    return 2;
  case PRIMARY_HV_STATUS_FRAME_ID:
    return 3;
  case INVERTERS_INV_L_RCV_FRAME_ID:
    return 4;
  case INVERTERS_INV_R_RCV_FRAME_ID:
    return 5;
  case PRIMARY_LV_CAN_FLASH_REQ_STEERING_WHEEL_FRAME_ID:
  case PRIMARY_LV_CAN_FLASH_REQ_TLM_FRAME_ID:
    return 6;
  case PRIMARY_ECU_STATUS_FRAME_ID:
    return 7;
  default:
    return -1;
  }
  return -1;
}

int can_start(void) {
  if (can_mgr_start(bms_lv_primary_can_id) < 0) {
    // TODO: Handle Error
    error_set(CAN, 0, HAL_GetTick());
  }
  return 0;
}

void can_send_messages() {
  static uint32_t last_msg[4] = {0};
  uint32_t current_time = HAL_GetTick();

  if (current_time - last_msg[0] > PRIMARY_LV_STATUS_CYCLE_TIME_MS) {
    primary_lv_status_send();
    primary_lv_errors_send();
    primary_inverter_connection_status_send();

    last_msg[0] = current_time;
  }

  if (current_time - last_msg[1] > PRIMARY_LV_CELLS_VOLTAGE_CYCLE_TIME_MS) {
    primary_lv_cells_voltage_send();
    primary_lv_cells_temp_send();
    primary_lv_total_voltage_send();
    primary_lv_current_battery_send();
    primary_lv_current_charger_send();
    primary_lv_feedback_ts_send();
    primary_lv_feedback_sd_send();
    primary_lv_feedback_enclosure_send();
    primary_lv_feedback_gpio_send();

    last_msg[1] = current_time;
  }

  if (current_time - last_msg[2] > PRIMARY_LV_PUMPS_SPEED_CYCLE_TIME_MS) {
    primary_lv_pump_speed_send();
    primary_lv_radiator_speed_send();

    last_msg[2] = current_time;
  }

  if (current_time - last_msg[3] > PRIMARY_LV_VERSION_CYCLE_TIME_MS) {
    primary_lv_version_send();

    last_msg[3] = current_time;
  }
}

int can_routine(void) {
  for (size_t msg_idx = 0; msg_idx < N_MONITORED_MESSAGES; ++msg_idx) {
    if (can_messages_is_new[msg_idx] &&
        (primary_message_handlers[msg_idx] != NULL)) {
      can_messages_is_new[msg_idx] = 0;
      (*primary_message_handlers[msg_idx])(&can_messages_states[msg_idx]);
    }
  }

  can_send_messages();
  return 0;
}

int primary_set_inverter_connection_status_handler(can_mgr_msg_t *msg) {
  // primary_set_inverter_connection_status_t set_inv_connection_raw;
  // primary_set_inverter_connection_status_unpack(
  //     &set_inv_connection_raw, msg->data,
  //     PRIMARY_SET_INVERTER_CONNECTION_STATUS_BYTE_SIZE);
  // primary_set_inverter_connection_status_raw_to_conversion_struct(
  //     &set_inv_connection_status, &set_inv_connection_raw);
  return 0;
}

int primary_set_radiator_speed_handler(can_mgr_msg_t *msg) {
  primary_lv_set_radiator_speed_t radiator_raw;
  primary_lv_set_radiator_speed_converted_t radiator_converted;
  primary_lv_set_radiator_speed_unpack(&radiator_raw, msg->data,
                                       PRIMARY_LV_SET_RADIATOR_SPEED_BYTE_SIZE);
  primary_lv_set_radiator_speed_raw_to_conversion_struct(&radiator_converted,
                                                         &radiator_raw);
  radiator_converted.radiator_speed =
      round(radiator_converted.radiator_speed * 10) / 10;

  if (radiator_converted.status == primary_lv_set_radiator_speed_status_auto) {
    radiator_set_status(primary_lv_radiator_speed_status_auto);
  } else if (radiator_converted.status ==
             primary_lv_set_radiator_speed_status_manual) {
    radiator_set_status(primary_lv_radiator_speed_status_manual);
    radiator_set_duty_cycle(radiator_converted.radiator_speed);
  }

  return 0;
}

int primary_set_pumps_speed_handler(can_mgr_msg_t *msg) {
  primary_lv_set_pumps_speed_t pumps_raw;
  primary_lv_set_pumps_speed_converted_t pumps_converted;
  primary_lv_set_pumps_speed_unpack(&pumps_raw, msg->data,
                                    PRIMARY_LV_SET_PUMPS_SPEED_BYTE_SIZE);
  primary_lv_set_pumps_speed_raw_to_conversion_struct(&pumps_converted,
                                                      &pumps_raw);
  pumps_converted.pumps_speed = round(pumps_converted.pumps_speed * 10) / 10;

  if (pumps_converted.status == primary_lv_set_pumps_speed_status_auto) {
    dac_pump_set_status(primary_lv_pumps_speed_status_auto);
  } else if (pumps_converted.status ==
             primary_lv_set_pumps_speed_status_manual) {
    dac_pump_set_status(primary_lv_pumps_speed_status_manual);
    dac_pump_set_duty_cycle(pumps_converted.pumps_speed);
  }

  return 0;
}

int primary_ts_status_handler(can_mgr_msg_t *msg) {
  primary_hv_status_t ts_status_raw;
  primary_hv_status_unpack(&ts_status_raw, msg->data,
                           PRIMARY_HV_STATUS_BYTE_SIZE);
  primary_hv_status_raw_to_conversion_struct(&ts_status_converted,
                                             &ts_status_raw);
  return 0;
}

int inverters_l_handler(can_mgr_msg_t *msg) {
  inverters_inv_l_rcv_t inv_raw;
  inverters_inv_l_rcv_converted_t inv_converted;
  inverters_inv_l_rcv_unpack(&inv_raw, msg->data,
                             INVERTERS_INV_L_RCV_BYTE_SIZE);
  inverters_inv_l_rcv_raw_to_conversion_struct(&inv_converted, &inv_raw);

  if (inv_converted.rcv_mux == inverters_inv_l_rcv_rcv_mux_ID_4A_T_Igbt) {
    inv_temps[0] = convert_t_igbt(inv_converted.t_igbt);
    float max = max(inv_temps[0], inv_temps[1]);

    if (radiator_is_auto()) {
      radiator_auto_mode(max);
    }

    if (dac_pump_is_auto()) {
      dac_pump_auto_mode(max);
    }
  }

  return 0;
}

int inverters_r_handler(can_mgr_msg_t *msg) {
  inverters_inv_r_rcv_t inv_raw;
  inverters_inv_r_rcv_converted_t inv_converted;
  inverters_inv_r_rcv_unpack(&inv_raw, msg->data,
                             INVERTERS_INV_R_RCV_BYTE_SIZE);
  inverters_inv_r_rcv_raw_to_conversion_struct(&inv_converted, &inv_raw);

  if (inv_converted.rcv_mux == inverters_inv_r_rcv_rcv_mux_ID_4A_T_Igbt) {
    inv_temps[1] = convert_t_igbt(inv_converted.t_igbt);
    float max = max(inv_temps[0], inv_temps[1]);

    if (radiator_is_auto()) {
      radiator_auto_mode(max);
    }

    if (dac_pump_is_auto()) {
      dac_pump_auto_mode(max);
    }
  }

  return 0;
}

int primary_flash_request_handler(can_mgr_msg_t *msg) {
  extern bool flash_requested;
  flash_requested = true;

  return 0;
}

int primary_ecu_status_handler(can_mgr_msg_t *msg) {
  primary_ecu_status_t ecu_status_raw;

  primary_ecu_status_unpack(&ecu_status_raw, msg->data,
                            PRIMARY_ECU_STATUS_BYTE_SIZE);
  primary_ecu_status_raw_to_conversion_struct(&ecu_status, &ecu_status_raw);

  return 0;
}

void primary_lv_cells_voltage_send(void) {
  primary_lv_cells_voltage_converted_t converted;
  float voltages[CELL_COUNT] = {0};

  monitor_get_voltages(voltages);
  for (size_t i = 0; i < 2; i++) {
    converted.start_index = i * 3;
    converted.voltage_0 = voltages[0 + i * 3];
    converted.voltage_1 = voltages[1 + i * 3];
    converted.voltage_2 = voltages[2 + i * 3];
    CANLIB_PACK_MSG(primary, PRIMARY, lv_cells_voltage, LV_CELLS_VOLTAGE);
    ERROR_TOGGLE_IF(can_mgr_send(bms_lv_primary_can_id, &msg) != 0, CAN, 0,
                    HAL_GetTick());
  }
}

void primary_lv_cells_temp_send(void) {
  primary_lv_cells_temp_converted_t converted;
  float temperatures[TEMP_SENSOR_COUNT] = {0};

  monitor_get_temperatures(temperatures);
  for (size_t i = 0; i < 4; i++) {
    converted.start_index = i * 3;
    converted.temp_0 = temperatures[0 + i * 3];
    converted.temp_1 = temperatures[1 + i * 3];
    converted.temp_2 = temperatures[2 + i * 3];
    CANLIB_PACK_MSG(primary, PRIMARY, lv_cells_temp, LV_CELLS_TEMP);
    ERROR_TOGGLE_IF(can_mgr_send(bms_lv_primary_can_id, &msg) != 0, CAN, 0,
                    HAL_GetTick());
  }
}

void primary_lv_total_voltage_send(void) {
  primary_lv_total_voltage_converted_t converted;
  float voltages[CELL_COUNT];
  monitor_get_voltages(voltages);

  converted.total = voltages[0] + voltages[1] + voltages[2] + voltages[3] +
                    voltages[4] + voltages[5];
  CANLIB_PACK_MSG(primary, PRIMARY, lv_total_voltage, LV_TOTAL_VOLTAGE);

  ERROR_TOGGLE_IF(can_mgr_send(bms_lv_primary_can_id, &msg) != 0, CAN, 0,
                  HAL_GetTick());
}

void primary_lv_status_send(void) {

  primary_lv_status_converted_t converted;
  extern state_t cur_state;

  switch (cur_state) {
  case STATE_INIT:
    converted.status = primary_lv_status_status_init;
    break;
  case STATE_IDLE:
    converted.status = primary_lv_status_status_idle;
    break;
  case STATE_TSON:
    converted.status = primary_lv_status_status_tson;
    break;
  case STATE_RUN:
    converted.status = primary_lv_status_status_run;
    break;
  case STATE_FLASHING:
    converted.status = primary_lv_status_status_flashing;
    break;
  case STATE_ERROR:
    converted.status = primary_lv_status_status_error;
    break;
  default:
    break;
  }

  CANLIB_PACK_MSG(primary, PRIMARY, lv_status, LV_STATUS);

  ERROR_TOGGLE_IF(can_mgr_send(bms_lv_primary_can_id, &msg) != 0, CAN, 0,
                  HAL_GetTick());
}

void primary_lv_errors_send(void) {
  primary_lv_errors_converted_t converted = {0};
  size_t expired_count = error_get_expired();

  if (expired_count > 0) {
    Error expired_instance[expired_count];
    error_dump_expired(expired_instance);
    for (size_t i = 0; i < expired_count; i++) {
      switch (expired_instance[i].group) {
      case CELL_UNDERVOLTAGE:
        converted.errors_cell_undervoltage = 1;
        break;
      case CELL_OVERVOLTAGE:
        converted.errors_cell_overvoltage = 1;
        break;
      case OPEN_WIRE:
        converted.errors_battery_open_wire = 1;
        break;
      case CAN:
        converted.errors_can = 1;
        break;
      case SPI:
        converted.errors_spi = 1;
        break;
      case OVER_CURRENT:
        converted.errors_over_current = 1;
        break;
      case CELL_UNDER_TEMPERATURE:
        converted.errors_cell_under_temperature = 1;
        break;
      case CELL_OVER_TEMPERATURE:
        converted.errors_cell_over_temperature = 1;
        break;
      case MCP23017:
        converted.errors_mcp23017 = 1;
        break;
      case HEALTH:
        converted.errors_mux = 1;
        break;
      default:
        break;
      }
    }
  }

  extern uint8_t health_status;
  converted.health_signals_sign_battery_current = (health_status >> 5) % 2;
  converted.health_signals_battery_current = (health_status >> 4) % 2;
  converted.health_signals_charger_current = (health_status >> 3) % 2;
  converted.health_signals_battery_voltage_out = (health_status >> 2) % 2;
  converted.health_signals_relay_out = (health_status >> 1) % 2;
  converted.health_signals_lvms_out = health_status % 2;

  CANLIB_PACK_MSG(primary, PRIMARY, lv_errors, LV_ERRORS);

  ERROR_TOGGLE_IF(can_mgr_send(bms_lv_primary_can_id, &msg) != 0, CAN, 0,
                  HAL_GetTick());
}

void primary_lv_current_charger_send(void) {
  primary_lv_current_charger_converted_t converted;
  extern float mux_sensors_mA[mux_sensors_n_values];

  converted.charger_current = mux_sensors_mA[mux_sensors_s_hall2_idx] / 1000.0;
  CANLIB_PACK_MSG(primary, PRIMARY, lv_current_charger, LV_CURRENT_CHARGER);

  ERROR_TOGGLE_IF(can_mgr_send(bms_lv_primary_can_id, &msg) != 0, CAN, 0,
                  HAL_GetTick());
}

void primary_lv_current_battery_send(void) {
  primary_lv_current_battery_converted_t converted;
  extern float mux_sensors_mA[mux_sensors_n_values];

  converted.lv_current = mux_sensors_mA[mux_sensors_s_hall1_idx] / 1000.0;
  CANLIB_PACK_MSG(primary, PRIMARY, lv_current_battery, LV_CURRENT_BATTERY);

  ERROR_TOGGLE_IF(can_mgr_send(bms_lv_primary_can_id, &msg) != 0, CAN, 0,
                  HAL_GetTick());
}

void primary_lv_feedback_sd_send(void) {
  primary_lv_feedback_sd_voltage_converted_t converted;
  extern float mux_fb_mV[mux_fb_n_values];

  FEEDBACK_SET_STATE(interlock);
  FEEDBACK_SET_STATE(lvms);
  converted.sd_end = mux_fb_mV[mux_fb_sd_end_idx] / 1000.0;
  converted.sd_start = mux_fb_mV[mux_fb_sd_start_idx] / 1000.0;

  CANLIB_PACK_MSG(primary, PRIMARY, lv_feedback_sd_voltage,
                  LV_FEEDBACK_SD_VOLTAGE);

  ERROR_TOGGLE_IF(can_mgr_send(bms_lv_primary_can_id, &msg) != 0, CAN, 0,
                  HAL_GetTick());
}

void primary_lv_feedback_enclosure_send(void) {
  primary_lv_feedback_enclosure_voltage_converted_t converted;
  extern float mux_fb_mV[mux_fb_n_values];

  FEEDBACK_SET_STATE(backplate);
  FEEDBACK_SET_STATE(hv_encl_2);
  FEEDBACK_SET_STATE(invc_lid);
  FEEDBACK_SET_STATE(lv_encl);

  CANLIB_PACK_MSG(primary, PRIMARY, lv_feedback_enclosure_voltage,
                  LV_FEEDBACK_ENCLOSURE_VOLTAGE);

  ERROR_TOGGLE_IF(can_mgr_send(bms_lv_primary_can_id, &msg) != 0, CAN, 0,
                  HAL_GetTick());
}

void primary_lv_feedback_ts_send(void) {
  primary_lv_feedback_ts_voltage_converted_t converted;

  extern float mux_fb_mV[mux_fb_n_values];
  FEEDBACK_SET_STATE(ams);
  FEEDBACK_SET_STATE(bspd);
  FEEDBACK_SET_STATE(hvd);
  FEEDBACK_SET_STATE(invc_interlock);

  CANLIB_PACK_MSG(primary, PRIMARY, lv_feedback_ts_voltage,
                  LV_FEEDBACK_TS_VOLTAGE);

  ERROR_TOGGLE_IF(can_mgr_send(bms_lv_primary_can_id, &msg) != 0, CAN, 0,
                  HAL_GetTick());
}

void primary_lv_feedback_gpio_send(void) {
  primary_lv_feedback_gpio_extender_converted_t converted;

  extern uint8_t mcp23017_feedbacks_state[8];

  converted.feedback_autonomous_system_actuation =
      mcp23017_feedbacks_state[mcp_feedbacks_bank_a__unused1];
  converted.feedback_hv_fans =
      mcp23017_feedbacks_state[mcp_feedbacks_bank_a_fan_fb];
  converted.feedback_inverters =
      mcp23017_feedbacks_state[mcp_feedbacks_bank_a_inverters_fb];
  converted.feedback_pcbs =
      mcp23017_feedbacks_state[mcp_feedbacks_bank_a_pcbs_fb];
  converted.feedback_pumps =
      mcp23017_feedbacks_state[mcp_feedbacks_bank_a_pumps_fb];
  converted.feedback_radiators =
      mcp23017_feedbacks_state[mcp_feedbacks_bank_a_radiators_fb];
  converted.feedback_shutdown =
      mcp23017_feedbacks_state[mcp_feedbacks_bank_a_shutdown_fb];

  CANLIB_PACK_MSG(primary, PRIMARY, lv_feedback_gpio_extender,
                  LV_FEEDBACK_GPIO_EXTENDER);

  ERROR_TOGGLE_IF(can_mgr_send(bms_lv_primary_can_id, &msg) != 0, CAN, 0,
                  HAL_GetTick());
}

void primary_inverter_connection_status_send(void) {
  primary_lv_inverter_connection_status_converted_t converted;
  extern uint8_t gpiob_register_value;
  converted.status =
      mcp23017_get_register_bit(gpiob_register_value, mcp_controls_bank_b_rfe);

  if (mcp23017_get_register_bit(gpiob_register_value,
                                mcp_controls_bank_b_rfe) !=
      mcp23017_get_register_bit(gpiob_register_value,
                                mcp_controls_bank_b_frg)) {
    converted.status = 0;
  }

  CANLIB_PACK_MSG(primary, PRIMARY, lv_inverter_connection_status,
                  LV_INVERTER_CONNECTION_STATUS);

  ERROR_TOGGLE_IF(can_mgr_send(bms_lv_primary_can_id, &msg) != 0, CAN, 0,
                  HAL_GetTick());
}

void primary_lv_version_send(void) {
  primary_lv_version_converted_t converted;
  converted.component_build_time = 0x1;
  converted.canlib_build_time = CANLIB_BUILD_TIME;
  CANLIB_PACK_MSG(primary, PRIMARY, lv_version, LV_VERSION);

  ERROR_TOGGLE_IF(can_mgr_send(bms_lv_primary_can_id, &msg) != 0, CAN, 0,
                  HAL_GetTick());
}

void primary_lv_pump_speed_send(void) {
  primary_lv_pumps_speed_converted_t converted;
  converted.status = dac_pump_get_status();
  converted.pumps_speed = dac_pump_get_duty_cycle();
  CANLIB_PACK_MSG(primary, PRIMARY, lv_pumps_speed, LV_PUMPS_SPEED);

  ERROR_TOGGLE_IF(can_mgr_send(bms_lv_primary_can_id, &msg) != 0, CAN, 0,
                  HAL_GetTick());
}

void primary_lv_radiator_speed_send(void) {
  primary_lv_radiator_speed_converted_t converted;
  converted.status = radiator_get_status();
  converted.radiator_speed = radiator_get_duty_cycle();
  CANLIB_PACK_MSG(primary, PRIMARY, lv_radiator_speed, LV_RADIATOR_SPEED);

  ERROR_TOGGLE_IF(can_mgr_send(bms_lv_primary_can_id, &msg) != 0, CAN, 0,
                  HAL_GetTick());
}