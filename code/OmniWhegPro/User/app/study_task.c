#include "study_task.h"

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

	get_imu_data(&imu_data);

	send_imu_data2pc();

	osDelay(500);

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

	remote_command_receive(66);

	mecanum_calculate(body_speed, body_rotation, motor_speed);

	motors_control();

	wheel_leg_remote_control();

	pwms_set();

	send_robot_info2pc();

}


void study_task(const void*argu)
{
	start_all();
		
	while(FOREVER)
		main_loop();		
}

