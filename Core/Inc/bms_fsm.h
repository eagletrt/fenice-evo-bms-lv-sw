/******************************************************************************
Finite State Machine
Project: fsm.dot
Description: bms_lv

Generated by gv_fsm ruby gem, see https://rubygems.org/gems/gv_fsm
gv_fsm version 0.3.4
Generation date: 2024-01-23 11:59:31 +0100
Generated from: fsm.dot
The finite state machine has:
  6 states
  7 transition functions
******************************************************************************/

#ifndef BMS_FSM_H
#define BMS_FSM_H
#include <stdlib.h>

// State data object
// By default set to void; override this typedef or load the proper
// header if you need
typedef void state_data_t;

// NOTHING SHALL BE CHANGED AFTER THIS LINE!

// List of states
typedef enum { STATE_INIT = 0, STATE_IDLE, STATE_ERROR, STATE_TSON, STATE_FLASHING, STATE_RUN, NUM_STATES, NO_CHANGE } state_t;

// State human-readable names
extern const char *state_names[];

// State function and state transition prototypes
typedef state_t state_func_t(state_data_t *data);
typedef void transition_func_t(state_data_t *data);

// State functions

// Function to be executed in state init
// valid return states: STATE_IDLE, STATE_ERROR
state_t do_init(state_data_t *data);

// Function to be executed in state idle
// valid return states: NO_CHANGE, STATE_IDLE, STATE_TSON, STATE_FLASHING,
// STATE_ERROR
state_t do_idle(state_data_t *data);

// Function to be executed in state error
// valid return states: NO_CHANGE
state_t do_error(state_data_t *data);

// Function to be executed in state tson
// valid return states: NO_CHANGE, STATE_IDLE, STATE_TSON, STATE_RUN
state_t do_tson(state_data_t *data);

// Function to be executed in state flashing
// valid return states: STATE_ERROR
state_t do_flashing(state_data_t *data);

// Function to be executed in state run
// valid return states: NO_CHANGE, STATE_IDLE, STATE_RUN, STATE_ERROR
state_t do_run(state_data_t *data);

// List of state functions
extern state_func_t *const state_table[NUM_STATES];

// Transition functions
void init_to_idle(state_data_t *data);
void to_error(state_data_t *data);
void idle_to_tson(state_data_t *data);
void idle_to_flashing(state_data_t *data);
void tson_to_idle(state_data_t *data);
void tson_to_run(state_data_t *data);
void run_to_idle(state_data_t *data);

// Table of transition functions
extern transition_func_t *const transition_table[NUM_STATES][NUM_STATES];

// state manager
state_t run_state(state_t cur_state, state_data_t *data);

#endif
