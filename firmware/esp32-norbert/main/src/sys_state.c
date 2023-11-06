#include "stdbool.h"

#include "sys_state.h"

static sys_state_t curr_sys_state; // Run assignment at startup

// It is assumed that these writes are atomic and there is no need for a mutex

sys_state_t get_sys_state() {
    return curr_sys_state;
}

bool sys_in_fault(){
    return curr_sys_state == FAULT;
}

void set_sys_state(sys_state_t state){
    curr_sys_state = state;
}