#include "motor_control.h"

void set_motor_speed(int16_t motor_speed, uint8_t ID_INDEX, uint8_t can1_send_data[8])
{
		can1_send_data[2*ID_INDEX] = (motor_speed>>8);
		can1_send_data[2*ID_INDEX+1] = ((motor_speed<<8)>>8);
}

void motors_control()
{
		for(int i=0; i<4; i++)
			set_motor_speed(motor_speed[i], i, can1_send_data);
		write_can(USER_CAN1, SPEED_MODE, can1_send_data);
}

