#include "study_task.h"

int i=0;

int isOpen = 0;

void open_legs()
{
	for(i=1080;i<2400;i+=30)
	{
		set_pwm_param(PWM_IO1,i);
		osDelay(20);
	}
	isOpen = 1;
}

void close_legs()
{
	
		for(i=2400;i>1080;i-=30)
		{
			set_pwm_param(PWM_IO1,i);
			osDelay(20);
		}
		isOpen = 0;
}

void study_task(const void*argu){
	
	set_pwm_group_param(PWM_GROUP1,20000);
	
	start_pwm_output(PWM_IO1);
	
	can_device_init();
	
	uint8_t can1_send_data[8];

	int16_t current=0;

	uint8_t n;
	
	while(1)
	{
		
		current=rc.ch2 / 66;
		
		can1_send_data[0] = (current>>8);
		can1_send_data[1] = ((current<<8)>>8);
		write_can(USER_CAN1,0x202,can1_send_data);
		
		if(rc.sw2 == 1 && isOpen == 0)
		{
			open_legs();
		}
		
		else if(rc.sw2 == 2 && isOpen == 1)
		{
			close_legs();
		}

		osDelay(20);
	}
}
