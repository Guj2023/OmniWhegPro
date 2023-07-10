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
void set_leg(int type, int pwm_time)
{
	if(type == 0)
	{
		set_pwm_param(PWM_IO1, pwm_time);
		set_pwm_param(PWM_IO4, pwm_time);

	}
	else if (type == 1)
	{
		set_pwm_param(PWM_IO2, pwm_time);
		set_pwm_param(PWM_IO3, pwm_time);
	}
}
