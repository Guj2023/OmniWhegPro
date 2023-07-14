#include "servo_control.h"

enum WHEEL_STATE front_wheel_state = CLOSED;
enum WHEEL_STATE back_wheel_state = CLOSED;

int wheg_value[4] = {0};

void pwms_set()
{
	for(int id=0;id<4;id++)
	{
		switch (id)
		{
		case 0:
			set_pwm_param(PWM_IO1,PWM_MIDDLE - 5 * wheg_value[0]);break;
		case 1:
			set_pwm_param(PWM_IO4,PWM_MIDDLE + 5 * wheg_value[1]);break;
		case 2:
			set_pwm_param(PWM_IO3,PWM_MIDDLE + 5 * wheg_value[2]);break;
		case 3:
			set_pwm_param(PWM_IO2,PWM_MIDDLE - 5 * wheg_value[3]);break;
		default:
			break;
		}
	}
}

void set_leg(int id, int wheg_size)
{
	wheg_value[id] = wheg_size;
}

void open_legs(int wheel_type)
{
	  if(wheel_type == 0)
		{
			for(int i=0; i<PWM_LOOP_TIME; i++)
			{
				set_leg(0, i * 4);
				set_leg(1, i * 4);
				osDelay(DELAY_TIME);
			}
			front_wheel_state = OPENED;
		}
		else if (wheel_type == 1)
		{
			for(int i=0; i<PWM_LOOP_TIME; i++)
			{
				set_leg(2, i * 4);
				set_leg(3, i * 4);
		   		osDelay(DELAY_TIME);
			}
			back_wheel_state = OPENED;
		}
}


void close_legs(int wheel_type)
{
	  if(wheel_type == 0)
		{
			for(int i=0;i<PWM_LOOP_TIME;i++)
			{
				set_leg(0, 100 - i * 4);
				set_leg(1, 100 - i * 4);
				osDelay(DELAY_TIME);
			}
			front_wheel_state = CLOSED;
		}
	  else if(wheel_type == 1)	
		{
			for(int i=0;i<PWM_LOOP_TIME;i++)
			{
				set_leg(2, 100 - i * 4);
				set_leg(3, 100 - i * 4);
				osDelay(DELAY_TIME);
			}
		back_wheel_state = CLOSED;
		}
}

enum WHEEL_STATE wheel_leg_switch_check(int wheel_type)
{
	int button = wheel_type?rc.sw2:rc.sw1;
	enum WHEEL_STATE wheel_state = wheel_type?back_wheel_state:front_wheel_state;
	if(button == 1 && wheel_state == CLOSED)
		return TO_OPEN;
	else if (button == 2 && wheel_state == OPENED)
		return TO_CLOSE;
	else 
		return wheel_type?back_wheel_state:front_wheel_state;
}

void wheel_leg_update()
{
	front_wheel_state = wheel_leg_switch_check(0);
	back_wheel_state = wheel_leg_switch_check(1);
}

void wheel_leg_switch(int wheel_type)
{
	enum WHEEL_STATE wheel_state = wheel_type?back_wheel_state:front_wheel_state;
	if(wheel_leg_switch_check(wheel_type) == TO_OPEN)
		open_legs(wheel_type);
	else if(wheel_leg_switch_check(wheel_type) == TO_CLOSE)
		close_legs(wheel_type);
	else
		return;
}

void all_wheel_leg_switch()
{
	wheel_leg_switch(0);
	wheel_leg_switch(1);
}

void wheel_leg_remote_control()
{
	wheel_leg_update();
	all_wheel_leg_switch();
}