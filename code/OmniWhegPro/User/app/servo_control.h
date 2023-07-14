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
extern int wheg_vaule[4];


void open_legs(int wheel_type);
void close_legs(int wheel_type);
void opening_legs(int type);
enum WHEEL_STATE wheel_leg_switch_check(int wheel_type);
void wheel_leg_switch(int wheel_type);
void servo_control();
void wheel_leg_update()
void all_wheel_leg_switch();
void wheel_leg_remote_control()




#endif

