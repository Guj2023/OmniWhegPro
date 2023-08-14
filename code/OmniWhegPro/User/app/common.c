#include "common.h"


uint8_t can1_send_data[8];

int16_t body_speed[2];
	
double body_rotation;
	
int16_t motor_speed[4];

uint8_t imu_test[8];

imu_t imu_data;

uint8_t debug_data[8];

uint8_t uart_recv[8];

uint8_t uart_wheg_value[4];

uint8_t uart_speed_value[4];

struct can_feedback feedback[4];

bool isControling = false;

char imu_message[50];

char wheel_message[50];

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

void uart_recv_callback(void)
{
	write_led_io(LED_IO1, LED_ON);

	uart_wheg_value[0] = uart_recv[0];
	uart_wheg_value[1] = uart_recv[1];
	uart_wheg_value[2] = uart_recv[2];
	uart_wheg_value[3] = uart_recv[3];

	uart_speed_value[0] = uart_recv[4];
	uart_speed_value[1] = uart_recv[5];
	uart_speed_value[2] = uart_recv[6];
	uart_speed_value[3] = uart_recv[7];
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

void mecanum_calculate(int16_t body_speed[2], double rotation, int16_t motor_speed[4])
{
	int r_vx = body_speed[0];
	int r_vy = body_speed[1];
	double r_vyaw = rotation;
	motor_speed[0] = ( - r_vy + r_vx + r_vyaw * MOTOR_TO_CENTER);
	motor_speed[1] = (r_vy + r_vx + r_vyaw * MOTOR_TO_CENTER);
	motor_speed[2] = (r_vy - r_vx + r_vyaw * MOTOR_TO_CENTER);
	motor_speed[3] = (- r_vy - r_vx + r_vyaw * MOTOR_TO_CENTER);

	LIMIT_MIN_MAX(motor_speed[0],-MAX_SPEED,MAX_SPEED);
	LIMIT_MIN_MAX(motor_speed[1],-MAX_SPEED,MAX_SPEED);
	LIMIT_MIN_MAX(motor_speed[2],-MAX_SPEED,MAX_SPEED);
	LIMIT_MIN_MAX(motor_speed[3],-MAX_SPEED,MAX_SPEED);
	
}

void remote_command_receive(int Scalefactor)
{
			body_speed[0] = rc.ch1 / Scalefactor;
		
			body_speed[1] = rc.ch2 / Scalefactor;
	
			body_rotation = rc.ch3 / Scalefactor;
	
}

void start_all()	
{
		// settle all parameters for servos
		set_leg(0, 0);
	
    	set_leg(1, 0);

		set_leg(2, 0);

		set_leg(3, 0);
	
		set_pwm_group_param(PWM_GROUP1,20000);
		
		start_pwm_output(PWM_IO1);
	
		start_pwm_output(PWM_IO2);
	
		start_pwm_output(PWM_IO3);
	
		start_pwm_output(PWM_IO4);
	
		can_device_init();
	
		can_recv_callback_register(USER_CAN1, can_receive);
	
		can_receive_start();
	
		uart_init(MY_UART, 9600, WORD_LEN_8B, STOP_BITS_1, PARITY_NONE);

		uart_recv_callback_register(MY_UART, uart_recv_callback);

		uart_receive_start(MY_UART, uart_recv, 8);
}

int stringlen(char *str)
{
	int i = 0;
	while(str[i] != '\0')
		i++;
	return i;
}

void send_imu_data2pc()
{
  int formatted_len = sprintf(imu_message, "i %.2f %.2f %.2f", imu_data.angle_x, imu_data.angle_y, imu_data.angle_z);
  sprintf(imu_message, "%s %d\r\n\0", imu_message, formatted_len);
    
  write_uart(MY_UART, (uint8_t*)(imu_message), sizeof(char) * stringlen(imu_message));
	
}

void send_wheel_info2pc(int id)
{
  int formatted_len = sprintf(wheel_message, "w %d %d %d %d %d %d", id, feedback[id].speed, 0, feedback[id].current, feedback[id].position, feedback[id].circle);
  sprintf(wheel_message, "%s %d\r\n", wheel_message, formatted_len);
    
  write_uart(MY_UART, (uint8_t*)(wheel_message), sizeof(char) * stringlen(wheel_message));
}
