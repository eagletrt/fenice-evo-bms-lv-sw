#include "can_messages.h"

#include "dac_pump.h"
#include "lv_errors.h"
#include "radiator.h"
#include "spi.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include <string.h>

extern int bms_lv_primary_can_id;
extern can_mgr_msg_t can_messages_states[N_MONITORED_MESSAGES];
extern uint8_t can_messages_is_new[N_MONITORED_MESSAGES];
// static float inv_temps[2];
primary_ts_status_converted_t ts_status_converted = {0};
primary_set_inverter_connection_status_converted_t set_inv_connection_status = {
    0};

int primary_set_inverter_connection_status_handler(can_mgr_msg_t *msg);
int primary_set_radiator_speed_handler(can_mgr_msg_t *msg);
int primary_set_pumps_speed_handler(can_mgr_msg_t *msg);
int primary_ts_status_handler(can_mgr_msg_t *msg);
int inverters_l_handler(can_mgr_msg_t *msg);
int inverters_r_handler(can_mgr_msg_t *msg);

void primary_lv_cells_voltage_send(void);
void primary_lv_cells_temp_send(void);
void primary_lv_total_voltage_send(void);
void primary_lv_status_send(void);
void primary_lv_errors_send(void);
void primary_lv_health_signals_send(void);
void primary_lv_currents_send(void);
void primary_lv_feedbacks_send(void);
void primary_inverter_connection_status_send(void);
void primary_lv_version_send(void);
void primary_cooling_status_send(void);

int (*primary_message_handlers[N_MONITORED_MESSAGES])(can_mgr_msg_t *) =
    CAN_MESSAGES_HANDLERS;

void can_init_errors_handler(int can_mgr_error_code) {
  // TODO: manage errors
}

int can_mgr_from_id_to_index(int can_id, int msg_id) {
  if (can_id != bms_lv_primary_can_id)
    return -1;
  switch (msg_id) {
  case PRIMARY_SET_INVERTER_CONNECTION_STATUS_FRAME_ID:
    return 0;
  case PRIMARY_SET_RADIATOR_SPEED_FRAME_ID:
    return 1;
  case PRIMARY_SET_PUMPS_SPEED_FRAME_ID:
    return 2;
  case PRIMARY_TS_STATUS_FRAME_ID:
    return 3;
  case INVERTERS_INV_L_RCV_FRAME_ID:
    return 4;
  case INVERTERS_INV_R_RCV_FRAME_ID:
    return 5;
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
  primary_lv_cells_voltage_send();
  primary_lv_cells_temp_send();
  primary_lv_total_voltage_send();
  primary_lv_status_send();
  primary_lv_errors_send();
  primary_lv_health_signals_send();
  primary_lv_currents_send();
  primary_lv_feedbacks_send();
  primary_inverter_connection_status_send();
  primary_lv_version_send();
  primary_cooling_status_send();
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
  primary_set_inverter_connection_status_t set_inv_connection_raw;
  primary_set_inverter_connection_status_unpack(
      &set_inv_connection_raw, msg->data,
      PRIMARY_SET_INVERTER_CONNECTION_STATUS_BYTE_SIZE);
  primary_set_inverter_connection_status_raw_to_conversion_struct(
      &set_inv_connection_status, &set_inv_connection_raw);
  return 0;
}

int primary_set_radiator_speed_handler(can_mgr_msg_t *msg) {
  primary_set_radiator_speed_t radiator_raw;
  primary_set_radiator_speed_converted_t radiator_converted;
  primary_set_radiator_speed_unpack(&radiator_raw, msg->data,
                                    PRIMARY_SET_RADIATOR_SPEED_BYTE_SIZE);
  primary_set_radiator_speed_raw_to_conversion_struct(&radiator_converted,
                                                      &radiator_raw);
  radiator_converted.radiators_speed =
      round(radiator_converted.radiators_speed * 10) / 10;
  if (radiator_converted.radiators_speed >= 0) {
    radiator_set_auto_mode(false);
    radiator_set_duty_cycle(radiator_converted.radiators_speed);
  } else {
    // radiator_set_auto_mode(true);
  }

  return 0;
}

int primary_set_pumps_speed_handler(can_mgr_msg_t *msg) {
  primary_set_pumps_speed_t pumps_raw;
  primary_set_pumps_speed_converted_t pumps_converted;
  primary_set_pumps_speed_unpack(&pumps_raw, msg->data,
                                 PRIMARY_SET_PUMPS_SPEED_BYTE_SIZE);
  primary_set_pumps_speed_raw_to_conversion_struct(&pumps_converted,
                                                   &pumps_raw);
  pumps_converted.pumps_speed = round(pumps_converted.pumps_speed * 10) / 10;
  if (pumps_converted.pumps_speed >= 0) {
    dac_pump_set_auto_mode(false);
    dac_pump_set_duty_cycle(pumps_converted.pumps_speed);
  } else {
    // dac_pump_set_auto_mode(true);
  }
  return 0;
}

int primary_ts_status_handler(can_mgr_msg_t *msg) {
  primary_ts_status_t ts_status_raw;
  primary_ts_status_unpack(&ts_status_raw, msg->data,
                           PRIMARY_TS_STATUS_BYTE_SIZE);
  primary_ts_status_raw_to_conversion_struct(&ts_status_converted,
                                             &ts_status_raw);
  return 0;
}

int inverters_l_handler(can_mgr_msg_t *msg) {
  inverters_inv_l_rcv_t inv_raw;
  inverters_inv_l_rcv_converted_t inv_converted;
  inverters_inv_l_rcv_unpack(&inv_raw, msg->data,
                             INVERTERS_INV_L_RCV_BYTE_SIZE);
  inverters_inv_l_rcv_raw_to_conversion_struct(&inv_converted, &inv_raw);

  // TODO add conversion of inverter temperature
  //  if (inv_converted.rcv_mux == inverters_inv_l_rcv_rcv_mux_ID_4A_T_Igbt) {
  //    inv_temps[0] = inv_converted.t_igbt;
  //    float max = max(inv_temps[0], inv_temps[1]);

  //   if (radiator_get_auto_mode()) {
  //     radiator_auto_mode(max);
  //   }

  //   if (dac_pump_get_auto_mode()) {
  //     dac_pump_auto_mode(max);
  //   }
  // }

  return 0;
}

int inverters_r_handler(can_mgr_msg_t *msg) {
  inverters_inv_r_rcv_t inv_raw;
  inverters_inv_r_rcv_converted_t inv_converted;
  inverters_inv_r_rcv_unpack(&inv_raw, msg->data,
                             INVERTERS_INV_R_RCV_BYTE_SIZE);
  inverters_inv_r_rcv_raw_to_conversion_struct(&inv_converted, &inv_raw);

  // TODO add conversion of inverter temperature
  //  if (inv_converted.rcv_mux == inverters_inv_r_rcv_rcv_mux_ID_4A_T_Igbt) {
  //    inv_temps[1] = inv_converted.t_igbt;
  //    float max = max(inv_temps[0], inv_temps[1]);

  //   if (radiator_get_auto_mode()) {
  //     radiator_auto_mode(max);
  //   }

  //   if (dac_pump_get_auto_mode()) {
  //     dac_pump_auto_mode(max);
  //   }
  // }

  return 0;
}

void primary_lv_cells_voltage_send(void) {
  static uint32_t last_msg_time = 0;
  uint32_t current_time = HAL_GetTick();

  if ((current_time - last_msg_time) > PRIMARY_LV_CELLS_VOLTAGE_CYCLE_TIME_MS) {
    last_msg_time = current_time;
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
}

void primary_lv_cells_temp_send(void) {
  static uint32_t last_msg_time = 0;
  uint32_t current_time = HAL_GetTick();

  if ((current_time - last_msg_time) > PRIMARY_LV_CELLS_TEMP_CYCLE_TIME_MS) {
    last_msg_time = current_time;
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
}

void primary_lv_total_voltage_send(void) {
  static uint32_t last_msg_time = 0;
  uint32_t current_time = HAL_GetTick();

  if ((current_time - last_msg_time) > PRIMARY_LV_TOTAL_VOLTAGE_CYCLE_TIME_MS) {
    last_msg_time = current_time;
    primary_lv_total_voltage_converted_t converted;
    float voltages[CELL_COUNT];
    monitor_get_voltages(voltages);

    converted.total_voltage = voltages[0] + voltages[1] + voltages[2] +
                              voltages[3] + voltages[4] + voltages[5];
    CANLIB_PACK_MSG(primary, PRIMARY, lv_total_voltage, LV_TOTAL_VOLTAGE);

    ERROR_TOGGLE_IF(can_mgr_send(bms_lv_primary_can_id, &msg) != 0, CAN, 0,
                    HAL_GetTick());
  }
}

// TODO get lv status from fsm and add new statuses to can-lib
void primary_lv_status_send(void) {
  // static uint32_t last_msg_time = 0;
  // uint32_t current_time = HAL_GetTick();

  // if ((current_time - last_msg_time) > PRIMARY_LV_STATUS_CYCLE_TIME_MS) {
  //   last_msg_time = current_time;
  //   primary_lv_status_converted_t converted;

  //   CANLIB_PACK_MSG(primary, PRIMARY, lv_status, LV_STATUS);

  //   ERROR_TOGGLE_IF(can_mgr_send(bms_lv_primary_can_id, &msg) != 0, CAN, 0,
  //                   HAL_GetTick());
  // }
}

void primary_lv_errors_send(void) {
  static uint32_t last_msg_time = 0;
  uint32_t current_time = HAL_GetTick();

  if ((current_time - last_msg_time) > PRIMARY_LV_ERRORS_CYCLE_TIME_MS) {
    last_msg_time = current_time;
    primary_lv_errors_converted_t converted = {0};
    size_t running_count = error_get_running();
    size_t expired_count = error_get_expired();

    if (running_count > 0) {
      Error running_instance[running_count];
      error_dump_running(running_instance);
      for (size_t i = 0; i < running_count; i++) {
        switch (running_instance[i].group) {
        case CELL_UNDERVOLTAGE:
          converted.warnings_cell_undervoltage = 1;
          break;
        case CELL_OVERVOLTAGE:
          converted.warnings_cell_overvoltage = 1;
          break;
        case OPEN_WIRE:
          converted.warnings_battery_open_wire = 1;
          break;
        case CAN:
          converted.warnings_can = 1;
          break;
        case SPI:
          converted.warnings_spi = 1;
          break;
        case OVER_CURRENT:
          converted.warnings_over_current = 1;
          break;
        case CELL_UNDER_TEMPERATURE:
          converted.warnings_cell_under_temperature = 1;
          break;
        case CELL_OVER_TEMPERATURE:
          converted.warnings_cell_over_temperature = 1;
          break;
        case MCP23017:
          converted.warnings_mcp23017 = 1;
          break;
        case HEALTH:
          converted.warnings_mux = 1;
          break;
        default:
          break;
        }
      }
    }

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
    // converted.errors_bms_monitor = 1;
    // converted.warnings_can = 1;
    // converted.errors_mcp23017 = 1;
    CANLIB_PACK_MSG(primary, PRIMARY, lv_errors, LV_ERRORS);

    ERROR_TOGGLE_IF(can_mgr_send(bms_lv_primary_can_id, &msg) != 0, CAN, 0,
                    HAL_GetTick());
  }
}

// TODO implement saving of health_signals
void primary_lv_health_signals_send(void) {
  // static uint32_t last_msg_time = 0;
  // uint32_t current_time = HAL_GetTick();

  // if ((current_time - last_msg_time) >
  // PRIMARY_LV_HEALTH_SIGNALS_CYCLE_TIME_MS) {
  //   last_msg_time = current_time;
  //   primary_lv_health_signals_converted_t converted;
  //   CANLIB_PACK_MSG(primary, PRIMARY, lv_health_signals, LV_HEALTH_SIGNALS);

  //   ERROR_TOGGLE_IF(can_mgr_send(bms_lv_primary_can_id, &msg) != 0, CAN, 0,
  //                   HAL_GetTick());
  // }
}

void primary_lv_currents_send(void) {
  static uint32_t last_msg_time = 0;
  uint32_t current_time = HAL_GetTick();

  if ((current_time - last_msg_time) > PRIMARY_LV_CURRENTS_CYCLE_TIME_MS) {
    last_msg_time = current_time;
    primary_lv_currents_converted_t converted;
    extern float mux_sensors_mA[mux_sensors_n_values];

    converted.current_as_battery =
        mux_sensors_mA[mux_sensors_s_hall0_idx] / 1000.0;
    converted.current_lv_battery =
        mux_sensors_mA[mux_sensors_s_hall1_idx] / 1000.0;
    converted.current_charger =
        mux_sensors_mA[mux_sensors_s_hall2_idx] / 1000.0;
    CANLIB_PACK_MSG(primary, PRIMARY, lv_currents, LV_CURRENTS);

    ERROR_TOGGLE_IF(can_mgr_send(bms_lv_primary_can_id, &msg) != 0, CAN, 0,
                    HAL_GetTick());
  }
}

// TODO change name to hv_encl_1_fb and hvd_fb
void primary_lv_feedbacks_send(void) {
  static uint32_t last_msg_time = 0;
  uint32_t current_time = HAL_GetTick();

  if ((current_time - last_msg_time) > PRIMARY_LV_FEEDBACKS_CYCLE_TIME_MS) {
    last_msg_time = current_time;
    primary_lv_feedbacks_converted_t converted;
    extern float mux_fb_mV[mux_fb_n_values];
    extern uint8_t mcp23017_feedbacks_state[8];

    FEEDBACK_SET_STATE(bspd,
                       mux_fb_mV[mux_fb_bspd_fb_idx] > HIGH_LEVEL_THRESHOLD);
    FEEDBACK_SET_STATE(hvd,
                       mux_fb_mV[mux_fb_hvd_fb_idx] > HIGH_LEVEL_THRESHOLD);
    FEEDBACK_SET_STATE(lvms,
                       mux_fb_mV[mux_fb_lvms_fb_idx] > HIGH_LEVEL_THRESHOLD);
    FEEDBACK_SET_STATE(res,
                       mux_fb_mV[mux_fb_res_fb_idx] > HIGH_LEVEL_THRESHOLD);
    converted.feedbacks_lv_encl =
        (mux_fb_mV[mux_fb_lv_encl_idx] > HIGH_LEVEL_THRESHOLD) ? 1 : 0;
    FEEDBACK_SET_STATE(invc_lid, mux_fb_mV[mux_fb_hv_encl_1_fb_idx] >
                                     HIGH_LEVEL_THRESHOLD);
    FEEDBACK_SET_STATE(hv_encl_2, mux_fb_mV[mux_fb_hv_encl_2_fb_idx] >
                                      HIGH_LEVEL_THRESHOLD);
    FEEDBACK_SET_STATE(back_plate, mux_fb_mV[mux_fb_back_plate_fb_idx] >
                                       HIGH_LEVEL_THRESHOLD);
    FEEDBACK_SET_STATE(invc_interlock,
                       mux_fb_mV[mux_fb_hvd_fb_idx] > HIGH_LEVEL_THRESHOLD);
    FEEDBACK_SET_STATE(ams,
                       mux_fb_mV[mux_fb_ams_fb_idx] > HIGH_LEVEL_THRESHOLD);
    FEEDBACK_SET_STATE(asms,
                       mux_fb_mV[mux_fb_asms_fb_idx] > HIGH_LEVEL_THRESHOLD);
    FEEDBACK_SET_STATE(interlock, mux_fb_mV[mux_fb_interlock_fb_idx] >
                                      HIGH_LEVEL_THRESHOLD);
    converted.sd_start = mux_fb_mV[mux_fb_sd_start_idx] / 1000.0;
    converted.sd_end = mux_fb_mV[mux_fb_sd_end_idx] / 1000.0;

    converted.feedbacks_inverters_fb = mcp23017_feedbacks_state[0];
    converted.feedbacks_pcbs_fb = mcp23017_feedbacks_state[1];
    converted.feedbacks_pumps_fb = mcp23017_feedbacks_state[2];
    converted.feedbacks_shutdown_fb = mcp23017_feedbacks_state[3];
    converted.feedbacks_radiators_fb = mcp23017_feedbacks_state[4];
    converted.feedbacks_fan_fb = mcp23017_feedbacks_state[5];
    converted.feedbacks_as_actuation_fb = mcp23017_feedbacks_state[6];

    CANLIB_PACK_MSG(primary, PRIMARY, lv_feedbacks, LV_FEEDBACKS);

    ERROR_TOGGLE_IF(can_mgr_send(bms_lv_primary_can_id, &msg) != 0, CAN, 0,
                    HAL_GetTick());
  }
}

void primary_inverter_connection_status_send(void) {
  static uint32_t last_msg_time = 0;
  uint32_t current_time = HAL_GetTick();

  if ((current_time - last_msg_time) >
      PRIMARY_INVERTER_CONNECTION_STATUS_CYCLE_TIME_MS) {
    last_msg_time = current_time;
    primary_inverter_connection_status_converted_t converted;
    extern uint8_t inverter_state;

    converted.status = inverter_state;
    CANLIB_PACK_MSG(primary, PRIMARY, inverter_connection_status,
                    INVERTER_CONNECTION_STATUS);

    ERROR_TOGGLE_IF(can_mgr_send(bms_lv_primary_can_id, &msg) != 0, CAN, 0,
                    HAL_GetTick());
  }
}

void primary_lv_version_send(void) {
  static uint32_t last_msg_time = 0;
  uint32_t current_time = HAL_GetTick();

  if ((current_time - last_msg_time) > PRIMARY_LV_VERSION_CYCLE_TIME_MS) {
    last_msg_time = current_time;
    primary_lv_version_converted_t converted;
    converted.component_build_time = 0x1;
    converted.component_build_time = CANLIB_BUILD_TIME;
    CANLIB_PACK_MSG(primary, PRIMARY, lv_version, LV_VERSION);

    ERROR_TOGGLE_IF(can_mgr_send(bms_lv_primary_can_id, &msg) != 0, CAN, 0,
                    HAL_GetTick());
  }
}

void primary_cooling_status_send(void) {
  static uint32_t last_msg_time = 0;
  uint32_t current_time = HAL_GetTick();

  if ((current_time - last_msg_time) > PRIMARY_COOLING_STATUS_CYCLE_TIME_MS) {
    last_msg_time = current_time;
    primary_cooling_status_converted_t converted;
    converted.pumps_speed = dac_pump_get_duty_cycle();
    converted.radiators_speed = radiator_get_duty_cycle();
    CANLIB_PACK_MSG(primary, PRIMARY, cooling_status, COOLING_STATUS);

    ERROR_TOGGLE_IF(can_mgr_send(bms_lv_primary_can_id, &msg) != 0, CAN, 0,
                    HAL_GetTick());
  }
}