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
		
		motors_command_receive();
	
		mecanum_calculate(body_speed, body_rotation, motor_speed);
		
		if(isControling && rc.ch4 != -660)
			motors_control();
		
		wheel_leg_switch(0);
	
 	  wheel_leg_switch(1);
		
		get_imu_data(&imu_data);
		
		imu_test[0] = ((int16_t)imu_data.angle_x>>8);
		imu_test[1] = (((int16_t)imu_data.angle_x)>>8);

		imu_test[2] = ((int16_t)imu_data.angle_y>>8);
		imu_test[3] = (((int16_t)imu_data.angle_y)>>8);

		imu_test[4] = ((int16_t)imu_data.angle_z>>8);
		imu_test[5] = (((int16_t)imu_data.angle_z)>>8);

		
		write_can(USER_CAN1, 0x510, imu_test);
		
		osDelay(5);

}


void study_task(const void*argu)
{
	start_all();
		
	while(FOREVER)
			main_loop();		
}
