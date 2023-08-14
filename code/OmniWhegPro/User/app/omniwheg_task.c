#include "omniwheg_task.h"

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
		remote_command_receive(66);
}


void main_loop()
{
		//LED?1??,??100?????
		write_led_io(LED_IO1,LED_ON);
		osDelay(100);
		write_led_io(LED_IO1,LED_OFF);
		
		//LED?2??,??100?????
		write_led_io(LED_IO2,LED_ON);
		osDelay(100);
		write_led_io(LED_IO2,LED_OFF);
		
		//LED?31??,??100?????		
		write_led_io(LED_IO3,LED_ON);
		osDelay(100);
		write_led_io(LED_IO3,LED_OFF);

		//LED?4??,??100?????
		write_led_io(LED_IO4,LED_ON);
		osDelay(100);
		write_led_io(LED_IO4,LED_OFF);

		//LED?5??,??100?????
		write_led_io(LED_IO5,LED_ON);
		osDelay(100);
		write_led_io(LED_IO5,LED_OFF);
		
		//LED?6??,??100?????
		write_led_io(LED_IO6,LED_ON);
		osDelay(100);
		write_led_io(LED_IO6,LED_OFF);

		//LED?7??,??100?????
		write_led_io(LED_IO7,LED_ON);
		osDelay(100);
		write_led_io(LED_IO7,LED_OFF);

		//LED?8??,??100?????
		write_led_io(LED_IO8,LED_ON);
		osDelay(100);
		write_led_io(LED_IO8,LED_OFF);

		write_uart(MY_UART,"hello world!\n",13);
}

void send_robot_info2pc()
{
	send_imu_data2pc();

	send_wheel_info2pc(0);
	send_wheel_info2pc(1);
	send_wheel_info2pc(2);
	send_wheel_info2pc(3);

}

void work_loop()
{
	get_imu_data(&imu_data);

	motors_command_receive();
	
	mecanum_calculate(body_speed, body_rotation, motor_speed);

	if(isControling)
		motors_control();

	wheel_leg_remote_control();
	
	pwms_set();

	//send_robot_info2pc();

	osDelay(20);
}


void omniwheg_task(const void*argu)
{
	start_all();
	while(FOREVER)
		work_loop();
}

void communication_task(const void*argu)
{
	osDelay(1000);
	while(FOREVER)
	{
		send_robot_info2pc();
		osDelay(100);
	}
}