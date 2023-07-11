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
