#include "rm_hal_lib.h"
#include "cmsis_os.h"
#include "uart_device.h"
#include <stdbool.h>
#include "cmath"

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))
#define MOTOR_TO_CENTER 0.375
#define WHEEL_RADIUS  0.095
#define PI 3.1416
#define MAX_SPEED 20
#define MAX_CURRENT 128
#define SPEED_MODE 0x202
#define CURRENT_MODE 0x206
#define POSITION_MODE 0x204
#define FOREVER 1
//#define PWM_MAX 2160
//#define PWM_MIN 840

#define PWM_MAX 2000 //2160
#define PWM_MIN 1000  //840
#define PWM_MIDDLE 1500
#define PWM_LOOP_TIME 30
#define PWM_PER_TIME 11
#define DELAY_TIME 3

#define SPEED_PID_P 1
#define SPEED_PID_I 1
#define SPEED_PID_D 1
#define SPEED_PID_I_MAX 100
#define SPEED_PID_O_MAX 200

#define ALINE_THRESHOLD 100
#define ALINE_OFFSET_FRONT 1559
#define ALINE_OFFSET_BACK 5722

uint8_t debug_data[8];
uint8_t imu_test[8];
imu_t imu_data;

typedef struct _pid_struct_t
{
  int16_t kp;
  int16_t ki;
  int16_t kd;
  int16_t i_max;
  int16_t out_max;
  
  int16_t ref;      // target value
  int16_t fdb;      // feedback value
  int16_t err[2];   // error and last error

  int16_t p_out;
  int16_t i_out;
  int16_t d_out;
  int16_t output;
}pid_struct_t;

pid_struct_t aline_pid;

pid_struct_t speed_pid[4];

struct can_feedback
{
	int16_t circle;
	int16_t position;
	int16_t speed;
	int16_t current;
	int16_t voltage;
	uint8_t temperature;
	uint8_t statue;
} feedback[4];
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

void pid_init_new(pid_struct_t *pid,
              int16_t kp,
              int16_t ki,
              int16_t kd,
              int16_t i_max,
              int16_t out_max)
{
  pid->kp      = kp;
  pid->ki      = ki;
  pid->kd      = kd;
  pid->i_max   = i_max;
  pid->out_max = out_max;
}

/**
  * @brief  pid calculation
  * @param  pid struct
    @param  reference value
    @param  feedback value
  * @retval calculation result
  */
int16_t pid_calc_new(pid_struct_t *pid, int16_t ref, int16_t fdb)
{
  pid->ref = ref;
  pid->fdb = fdb;
  pid->err[1] = pid->err[0];
  pid->err[0] = pid->ref - pid->fdb;
  
  pid->p_out  = pid->kp * pid->err[0];
  pid->i_out += pid->ki * pid->err[0];
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);
  
  pid->output = pid->p_out + pid->i_out + pid->d_out;
  LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);
  return pid->output;
}


enum WHEEL_STATE
 {
  OPENED,
	TO_OPEN,
	TO_CLOSE,
	CLOSED
};

enum WHEEL_STATE front_wheel_state = CLOSED;
enum WHEEL_STATE back_wheel_state = CLOSED;
uint8_t can1_send_data[8];

int16_t body_speed[2];
	
double body_rotation;
	
int16_t motor_speed[4];

bool isControling = false;

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

void set_motor_speed(int16_t motor_speed, uint8_t ID_INDEX, uint8_t can1_send_data[8])
{
		can1_send_data[2*ID_INDEX] = (motor_speed>>8);
		can1_send_data[2*ID_INDEX+1] = ((motor_speed<<8)>>8);
}
void start_speed_pids()
{
		pid_init_new(speed_pid + 0, SPEED_PID_P, SPEED_PID_I, SPEED_PID_D, SPEED_PID_I_MAX, SPEED_PID_O_MAX);
		pid_init_new(speed_pid + 1, SPEED_PID_P, SPEED_PID_I, SPEED_PID_D, SPEED_PID_I_MAX, SPEED_PID_O_MAX);
		pid_init_new(speed_pid + 2, SPEED_PID_P, SPEED_PID_I, SPEED_PID_D, SPEED_PID_I_MAX, SPEED_PID_O_MAX);
		pid_init_new(speed_pid + 3, SPEED_PID_P, SPEED_PID_I, SPEED_PID_D, SPEED_PID_I_MAX, SPEED_PID_O_MAX);
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

void motors_control()
{
		for(int i=0; i<4; i++)
			set_motor_speed(motor_speed[i], i, can1_send_data);
		write_can(USER_CAN1, SPEED_MODE, can1_send_data);
}

double fill_into(double error)
{
	while(error > 9000)
		error -= 9000;
	while(error < 0)
		error += 9000;
	return error;
}

bool isShifted(double error, bool isFront)
{
	if(isFront)
		return (error >= ALINE_OFFSET_FRONT + ALINE_THRESHOLD || error <= ALINE_OFFSET_FRONT - ALINE_THRESHOLD);
	else
		return (error >= ALINE_OFFSET_BACK + ALINE_THRESHOLD || error <= ALINE_OFFSET_BACK - ALINE_THRESHOLD);
}

void sendDebugInfo(double error, bool isFront)
{
			debug_data[0] = ((int16_t)error>>8);
			debug_data[1] = (((int16_t)error<<8)>>8);
			isFront ? write_can(USER_CAN1,0x0404,debug_data) : write_can(USER_CAN1,0x0405,debug_data);
}

void sendAllDebugInfo(double error_front, double error_back)
{
	sendDebugInfo(error_front, true);
	sendDebugInfo(error_back, false);
}

void autoAlignment()
{
	
			double error_front = fill_into(feedback[0].position + feedback[1].position);	
			double error_back = fill_into(-feedback[2].position - feedback[3].position);
			
			body_speed[0] = ((isShifted(error_front, true) || isShifted(error_back, false))) ? -1 : 0;
			sendAllDebugInfo(error_front, error_back);
			body_speed[1] = 0;
			body_rotation = 0;
}

void autoAlignmentOneSide(bool isFront)
{
	
	double error = isFront ? fill_into(feedback[0].position + feedback[1].position) : fill_into(-feedback[2].position - feedback[3].position);
	body_speed[0] = isShifted(error, isFront) ? 1 : 0;
	sendDebugInfo(error,isFront);
	body_speed[1] = 0;
	body_rotation = 0;
}

void autoAlignmentWithoutshifting()
{
			double error_front = fill_into(feedback[0].position + feedback[1].position);	
			double error_back = fill_into(-feedback[2].position - feedback[3].position);
			sendAllDebugInfo(error_front, error_back);
	    uint8_t send_data[8];
	
			int speed_front = -isShifted(error_front, true);
	    int speed_back = -isShifted(error_back, false);
			set_motor_speed((int16_t)(0), 0, send_data);
			set_motor_speed((int16_t)(speed_front), 1, send_data);
			set_motor_speed((int16_t)(0), 2, send_data);
			set_motor_speed((int16_t)(speed_back), 3, send_data);
	
			write_can(USER_CAN1, SPEED_MODE, send_data);
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

void main_loop()
{
		
		motors_command_receive();
	
		mecanum_calculate(body_speed, body_rotation, motor_speed);
		
		if(isControling && rc.ch4 != -660)
			motors_control();
		
		wheel_leg_switch(0);
	
 	  wheel_leg_switch(1);
		
		get_imu_data(&imu_data);
		
		imu_test[0] = ((int16_t)i.angle_x>>8);
		imu_test[1] = (((int16_t)i.angle_x)>>8);

		imu_test[2] = ((int16_t)i.angle_y>>8);
		imu_test[3] = (((int16_t)i.angle_y)>>8);

		imu_test[4] = ((int16_t)i.angle_z>>8);
		imu_test[5] = (((int16_t)i.angle_z)>>8);

		
		write_can(USER_CAN1, 0x510, imu_test);
		
		osDelay(5);

}

void test_loop()
{
		
		int16_t error = (-feedback[2].position - feedback[3].position);	
	
		if(error>0)
			while(error > 9000)
				error -= 9000;
		else if(error < 0)
			while(error < 0)
				error += 9000;
	
		debug_data[0] = (error>>8);
		debug_data[1] = ((error<<8)>>8);
			
		write_can(USER_CAN1,0x0404,debug_data);
			
		if(error >= ALINE_OFFSET_BACK + ALINE_THRESHOLD || error <= ALINE_OFFSET_BACK - ALINE_THRESHOLD)
			body_speed[1] = -1;//command_speed;
		else 
	    body_speed[1] = 0; 
	
		body_speed[0] = 0;
	
		body_rotation = 0;
		
		mecanum_calculate(body_speed, body_rotation, motor_speed);

	  motors_control();
	
		osDelay(20);
}

void study_task(const void*argu)
{
	start_all();
		
	while(FOREVER)
			main_loop();		
}
