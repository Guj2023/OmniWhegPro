#ifndef __SERVO_CONTROL_H__
#define __SERVO_CONTROL_H__
#include "common.h"

enum WHEEL_STATE
 {
    OPENED = 0,
	TO_OPEN,
	TO_CLOSE,
	CLOSED
};
extern enum WHEEL_STATE front_wheel_state;
extern enum WHEEL_STATE back_wheel_state;

void open_legs(int wheel_type);
void close_legs(int wheel_type);
void opening_legs(int type);
void set_leg(int type, int pwm_time);
enum WHEEL_STATE wheel_leg_switch_check(int wheel_type);
void wheel_leg_switch(int wheel_type);

#endif

