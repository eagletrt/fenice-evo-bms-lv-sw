#include "bms_cli.h"

uint32_t bms_cli_primask = 0;

void cs_enter(void) {
    bms_cli_primask = __get_PRIMASK();
    __disable_irq();
}

void cs_exit(void) {
    if (!bms_cli_primask)
        __enable_irq();
}

uint8_t serial_rx_buffer = 0;

void serial_rx(void) {
    HAL_UART_Receive_IT(&huart1, &serial_rx_buffer, 1);
}

#define SERIAL_TIMEOUT 200
void serial_tx(char *message, size_t size) {
    while (!(huart1.gState == HAL_UART_STATE_READY))
        ;
    HAL_UART_Transmit(&huart1, (uint8_t *)message, size, SERIAL_TIMEOUT);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (ucli_receive_data((char)serial_rx_buffer) != UCLI_RETURN_CODE_OK) {
        return;
    }
}

void print_commands(int argc, char args[][10]);
void print_voltages(int argc, char args[][10]);
void print_temperature(int argc, char args[][10]);
void print_health_status(int argc, char args[][10]);

ucli_command_t bms_cli_commands[] = {
    {.name = "commands", .function = &print_commands},
    {.name = "volt", .function = &print_voltages},
    {.name = "temp", .function = &print_temperature},
    {.name = "health", .function = &print_health_status},
};
const size_t bms_cli_commands_size = sizeof(bms_cli_commands) / sizeof(bms_cli_commands[0]);

#define BMS_CLI_BUFFER_SIZE (256U)
char bms_cli_buffer[BMS_CLI_BUFFER_SIZE];

/** Print all commands */
void print_commands(int argc, char args[][10]) {
    (void)argc;
    (void)args;
    int buff_idx             = 0;
    const char separator[]   = ", ";
    const char delimiter[]   = "\r\n";
    const char annoncement[] = "Available commands are: ";
    memcpy(bms_cli_buffer + buff_idx, delimiter, strlen(delimiter));
    buff_idx += strlen(delimiter);
    memcpy(bms_cli_buffer + buff_idx, annoncement, strlen(annoncement));
    buff_idx += strlen(annoncement);
    for (size_t i = 0; (i < bms_cli_commands_size); i++) {
        if ((buff_idx + strlen(bms_cli_commands[i].name) + strlen(separator)) >= (BMS_CLI_BUFFER_SIZE - strlen(delimiter))) {
            break;
        }
        memcpy(bms_cli_buffer + buff_idx, bms_cli_commands[i].name, strlen(bms_cli_commands[i].name));
        buff_idx += strlen(bms_cli_commands[i].name);
        memcpy(bms_cli_buffer + buff_idx, separator, strlen(separator));
        buff_idx += strlen(separator);
    }
    memcpy(bms_cli_buffer + buff_idx, delimiter, strlen(delimiter));
    buff_idx += strlen(delimiter);

    serial_tx(bms_cli_buffer, buff_idx);
}

/** Print voltages */
void print_voltages(int argc, char args[][10]) {
    (void)argc;
    (void)args;

    const char separator[]   = ", ";
    const char delimiter[]   = "\r\n";
    const char annoncement[] = "Cell voltages (in mV): ";
    int added_chars, buff_idx = 0;
    memcpy(bms_cli_buffer + buff_idx, delimiter, strlen(delimiter));
    buff_idx += strlen(delimiter);
    memcpy(bms_cli_buffer + buff_idx, annoncement, strlen(annoncement));
    buff_idx += strlen(annoncement);

    float voltages[CELL_COUNT] = {0};
    monitor_get_voltages(voltages);

    for (size_t icell = 0; icell < CELL_COUNT; icell++) {
        added_chars = snprintf(bms_cli_buffer + buff_idx, BMS_CLI_BUFFER_SIZE - buff_idx, "%lu", (uint32_t)(voltages[icell] * 1000.0f));
        if (added_chars < 0) {
            break;
        }
        buff_idx += added_chars;

        if ((buff_idx + strlen(separator)) >= (BMS_CLI_BUFFER_SIZE - strlen(delimiter))) {
            break;
        }
        memcpy(bms_cli_buffer + buff_idx, separator, strlen(separator));
        buff_idx += strlen(separator);
    }
    memcpy(bms_cli_buffer + buff_idx, delimiter, strlen(delimiter));
    buff_idx += strlen(delimiter);

    serial_tx(bms_cli_buffer, buff_idx);
}

/** Print temperatures */
void print_temperature(int argc, char args[][10]) {
    (void)argc;
    (void)args;

    const char separator[]   = ", ";
    const char delimiter[]   = "\r\n";
    const char annoncement[] = "Cell temperatures (in degrees Celsius): ";
    int added_chars, buff_idx = 0;
    memcpy(bms_cli_buffer + buff_idx, delimiter, strlen(delimiter));
    buff_idx += strlen(delimiter);
    memcpy(bms_cli_buffer + buff_idx, annoncement, strlen(annoncement));
    buff_idx += strlen(annoncement);

    float temperatures[TEMP_SENSOR_COUNT] = {0};
    monitor_get_temperatures(temperatures);

    for (size_t itemp = 0; itemp < CELL_COUNT; itemp++) {
        added_chars = snprintf(bms_cli_buffer + buff_idx, BMS_CLI_BUFFER_SIZE - buff_idx, "%.2f", temperatures[itemp]);
        if (added_chars < 0) {
            break;
        }
        buff_idx += added_chars;

        if ((buff_idx + strlen(separator)) >= (BMS_CLI_BUFFER_SIZE - strlen(delimiter))) {
            break;
        }
        memcpy(bms_cli_buffer + buff_idx, separator, strlen(separator));
        buff_idx += strlen(separator);
    }
    memcpy(bms_cli_buffer + buff_idx, delimiter, strlen(delimiter));
    buff_idx += strlen(delimiter);

    serial_tx(bms_cli_buffer, buff_idx);
}

/** Print health status */
void print_health_status(int argc, char args[][10]) {
    (void)argc;
    (void)args;

    // const char separator[] = ", ";
    const char delimiter[]   = "\r\n";
    const char annoncement[] = "Health status: ";
    int added_chars, buff_idx = 0;
    memcpy(bms_cli_buffer + buff_idx, delimiter, strlen(delimiter));
    buff_idx += strlen(delimiter);
    memcpy(bms_cli_buffer + buff_idx, annoncement, strlen(annoncement));
    buff_idx += strlen(annoncement);

    // TODO: display better the values
    /*
    const char health_values[] = {
        "SEGNO DELLA BATTERIA",
        "CORRENTE DELLA BATTERIA > " MIN_BATTERY_CURRENT_THRESHOLD_mA,
        "ICHARGER_INDEX",
        "BATTOUT_INDEX",
        "RELAY_OUT_INDEX",
        "LVMS_OUT_INDEX"
    };
    */

    extern uint8_t health_status;
    added_chars = snprintf(bms_cli_buffer + buff_idx, BMS_CLI_BUFFER_SIZE - buff_idx, "%d", (int)health_status);
    if (added_chars < 0) {
        // TODO handle errors
    }
    buff_idx += added_chars;

    memcpy(bms_cli_buffer + buff_idx, delimiter, strlen(delimiter));
    buff_idx += strlen(delimiter);

    serial_tx(bms_cli_buffer, buff_idx);
}

int bms_cli_init(void) {
    ucli_handler_t ucli_handler = {
        .enable_receive = &serial_rx,
        .send           = &serial_tx,
        .cs_enter       = &cs_enter,
        .cs_exit        = &cs_exit,
        .echo           = true,
    };

    if (ucli_init(ucli_handler) != UCLI_RETURN_CODE_OK) {
        return 1;
    }

    for (size_t icommand = 0; icommand < bms_cli_commands_size; icommand++) {
        if (ucli_add_command(bms_cli_commands[icommand]) != UCLI_RETURN_CODE_OK) {
            return 1;
        }
    }
    serial_rx();
    return 0;
}