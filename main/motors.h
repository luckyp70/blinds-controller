#pragma once

typedef enum
{
    MOTOR_1 = 1,
    MOTOR_2 = 2,
} motor_id_t;

typedef enum
{
    MOTOR_STOPPED = 0,
    MOTOR_MOVING_UP = 1,
    MOTOR_MOVING_DOWN = 2,
} motor_state_t;

void motors_init(void);
void motor_up(motor_id_t motor_id);
void motor_down(motor_id_t motor_id);
void motor_stop(motor_id_t motor_id);

void motor_toggle_up(motor_id_t motor_id);
void motor_toggle_down(motor_id_t motor_id);
motor_state_t motor_get_state(motor_id_t motor_id);