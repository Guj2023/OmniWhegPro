#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H
#include "common.h"


void set_motor_speed(int16_t motor_speed, uint8_t ID_INDEX, uint8_t can1_send_data[8]);
void motors_control();

#endif