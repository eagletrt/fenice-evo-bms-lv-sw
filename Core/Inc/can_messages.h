/**
 * @author Enrico Dalla Croce (Kalsifer-742)
 * @date 30/11/2023
*/

#include "can_manager.h"
#include "inverters_network.h"
#include "primary_network.h"

#ifndef can_messages_h
#define can_messages_h

void can_init_errors_handler(int can_manager_error_code);
void can_primary_ntw_handler(can_manager_message_t *msg);
void can_secondary_ntw_handler(can_manager_message_t *msg);

#endif