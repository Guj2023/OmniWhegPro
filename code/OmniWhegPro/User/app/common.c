#include "common.h"


uint8_t can1_send_data[8];

int16_t body_speed[2];
	
double body_rotation;
	
int16_t motor_speed[4];

struct can_feedback feedback[4];

bool isControling = false;

void can_receive(uint32_t recv_id, uint8_t data[])
{
	if(recv_id == 0x208)
	{
		feedback[0].circle = (int16_t) ((data[0] << 8) | data[1]);
    feedback[0].position = (int16_t) ((data[2] << 8) | data[3]);
    feedback[0].speed = (int16_t) ((data[4] << 8) | data[5]);
    feedback[0].current = (int16_t) ((data[6] << 8) | data[7]);	
		write_led_io(LED_IO1, LED_ON);
	}
  else if (recv_id == 0x209)
  {
    feedback[0].voltage = data[0];
    feedback[0].temperature = data[1];
		feedback[0].statue = data[2];
		write_led_io(LED_IO1, LED_OFF);
  }
	else if(recv_id == 0x20a)
	{
		feedback[1].circle = (int16_t) ((data[0] << 8) | data[1]);
    feedback[1].position = (int16_t) ((data[2] << 8) | data[3]);
    feedback[1].speed = (int16_t) ((data[4] << 8) | data[5]);
    feedback[1].current = (int16_t) ((data[6] << 8) | data[7]);	
		write_led_io(LED_IO2, LED_ON);
}
  else if (recv_id == 0x20b)
  {
    feedback[1].voltage = data[0];
    feedback[1].temperature = data[1];
		feedback[1].statue = data[2];
		write_led_io(LED_IO2, LED_OFF);

	}
	else if(recv_id == 0x20c)
	{
		feedback[2].circle = (int16_t) ((data[0] << 8) | data[1]);
    feedback[2].position = (int16_t) ((data[2] << 8) | data[3]);
    feedback[2].speed = (int16_t) ((data[4] << 8) | data[5]);
    feedback[2].current = (int16_t) ((data[6] << 8) | data[7]);	
		write_led_io(LED_IO3, LED_ON);

	}
  else if (recv_id == 0x20d)
  {
    feedback[2].voltage = data[0];
    feedback[2].temperature = data[1];
		feedback[2].statue = data[2];
  	write_led_io(LED_IO3, LED_OFF);
  }
	else if(recv_id == 0x20e)
	{
		feedback[3].circle = (int16_t) ((data[0] << 8) | data[1]);
    feedback[3].position = (int16_t) ((data[2] << 8) | data[3]);
    feedback[3].speed = (int16_t) ((data[4] << 8) | data[5]);
    feedback[3].current = (int16_t) ((data[6] << 8) | data[7]);	
		write_led_io(LED_IO4, LED_ON);
	}
  else if (recv_id == 0x20f)
  {
    feedback[3].voltage = data[0];
    feedback[3].temperature = data[1];
		feedback[3].statue = data[2];
		write_led_io(LED_IO4, LED_OFF);

  }
	else
		return;
}

double fill_into(double error)
{
	while(error > 9000)
		error -= 9000;
	while(error < 0)
		error += 9000;
	return error;
}

void remoteControl(int Scalefactor)
{
			body_speed[0] = rc.ch1 / Scalefactor;
		
			body_speed[1] = rc.ch2 / Scalefactor;
	
			body_rotation = rc.ch3 / Scalefactor;
	
}

void motors_command_receive()
{
	 if(rc.sw1 == 3 && rc.sw2 == 3)
		 isControling = false;
	 else
		 isControling = true;
	 if(rc.ch4 >= 300 && rc.ch4 < 660)
		 autoAlignmentOneSide(true);
	 else if(rc.ch4 <= -300  && rc.ch4 < -660)
		 autoAlignmentOneSide(false);
	 else if(rc.ch4 == 660)
		autoAlignment();
	 else if(rc.ch4 == -660)
		autoAlignmentWithoutshifting();
	 else
		remoteControl(66);
}

void start_all()	
{
		// settle all parameters for servos
		set_leg(0, PWM_MIDDLE);
	
    set_leg(1, PWM_MIDDLE);
	
		set_pwm_group_param(PWM_GROUP1,20000);
		
		start_pwm_output(PWM_IO1);
	
		start_pwm_output(PWM_IO2);
	
		start_pwm_output(PWM_IO3);
	
		start_pwm_output(PWM_IO4);
	
	  // settle all paramaters for BLDCs
		can_device_init();
	
	  can_recv_callback_register(USER_CAN1, can_receive);
	
		can_receive_start();
	
		// settle all parameters for PID
		//pid_init_new(&aline_pid, 1, 0, 0, 5, 5);
}
