#ifndef SYS_STATE_H
#define SYS_STATE_h

typedef enum sys_state_t {
    AUTO_CTRL,
    MANUAL_CTRL,
    FAULT
} sys_state_t;

sys_state_t get_sys_state();
bool sys_in_fault();
void set_sys_state(sys_state_t state);

#endif // SYS_STATE_H