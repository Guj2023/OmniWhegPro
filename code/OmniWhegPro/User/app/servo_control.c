#include "servo_control.h"

enum WHEEL_STATE front_wheel_state = CLOSED;
enum WHEEL_STATE back_wheel_state = CLOSED;

void open_legs(int wheel_type)
{
	  if(wheel_type == 0)
		{
			for(int i=0; i<PWM_LOOP_TIME; i++)
			{
				set_pwm_param(PWM_IO1,PWM_MIDDLE - PWM_PER_TIME * i);
				set_pwm_param(PWM_IO4,PWM_MIDDLE + PWM_PER_TIME * i);
				osDelay(DELAY_TIME);
			}
			front_wheel_state = OPENED;
		}
		else if (wheel_type == 1)
		{
			for(int i=0; i<PWM_LOOP_TIME; i++)
			{
			 set_pwm_param(PWM_IO2,PWM_MIDDLE - PWM_PER_TIME * i);
			 set_pwm_param(PWM_IO3,PWM_MIDDLE + PWM_PER_TIME * i);
		   osDelay(DELAY_TIME);
			}
			back_wheel_state = OPENED;
		}
}
void opening_legs(int type)
{
	if(type == 0)
	{
		set_pwm_param(PWM_IO1,PWM_MIN);
		set_pwm_param(PWM_IO4,PWM_MAX);
	}
	else if(type == 1)
	{
		set_pwm_param(PWM_IO2,PWM_MIN);
		set_pwm_param(PWM_IO3,PWM_MAX);
	}
}
void close_legs(int wheel_type)
{
	  if(wheel_type == 0)
		{
			for(int i=0;i<PWM_LOOP_TIME;i++)
			{
				set_pwm_param(PWM_IO1,PWM_MIN + PWM_PER_TIME*i);
				set_pwm_param(PWM_IO4,PWM_MAX - PWM_PER_TIME*i);
				osDelay(DELAY_TIME);
			}
			front_wheel_state = CLOSED;
		}
	  else if(wheel_type == 1)	
		{
			for(int i=0;i<PWM_LOOP_TIME;i++)
			{
				set_pwm_param(PWM_IO2,PWM_MIN + PWM_PER_TIME*i);
				set_pwm_param(PWM_IO3,PWM_MAX - PWM_PER_TIME*i);
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

void wheel_leg_switch(int wheel_type)
{
		enum WHEEL_STATE wheel_state = wheel_type?back_wheel_state:front_wheel_state;
		if(wheel_leg_switch_check(wheel_type) == TO_OPEN)
			open_legs(wheel_type);
		else if(wheel_leg_switch_check(wheel_type) == TO_CLOSE)
			close_legs(wheel_type);
		else if(wheel_state == CLOSED)
			set_leg(wheel_type, PWM_MIDDLE);
		else if(wheel_state == OPENED)
			opening_legs(wheel_type);
		
}