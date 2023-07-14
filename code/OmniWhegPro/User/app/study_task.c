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
		remoteControl(66);
}


void main_loop()
{

	get_imu_data(&imu_data);

	sendImuData2pc();

	osDelay(500);

}

void sendRobotInfo()
{
	sendImuData2pc();

	sendWheelInfo2pc(0);
	sendWheelInfo2pc(1);
	sendWheelInfo2pc(2);
	sendWheelInfo2pc(3);

}

void work_loop()
{
	get_imu_data(&imu_data);

	remoteControl(66);

	mecanum_calculate(body_speed, body_rotation, motor_speed);

	motors_control();

	wheel_leg_remote_control();

	pwms_set();

	sendRobotInfo();

}


void study_task(const void*argu)
{
	start_all();
		
	while(FOREVER)
		main_loop();		
}

