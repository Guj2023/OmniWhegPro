#ifndef __COMMON_H__
#define __COMMON_H__

#include "rm_hal_lib.h"
#include "cmsis_os.h"
#include "uart_device.h"
#include <stdbool.h>
#include "math.h"
//#include "cmath"

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

#define PWM_MAX 2000 //2160
#define PWM_MIN 1000  //840
#define PWM_MIDDLE 1500
#define PWM_LOOP_TIME 30
#define PWM_PER_TIME 11
#define DELAY_TIME 3


#define ALINE_THRESHOLD 100
#define ALINE_OFFSET_FRONT 1559
#define ALINE_OFFSET_BACK 5722

extern uint8_t debug_data[8];
extern uint8_t imu_test[8];
extern imu_t imu_data;
extern uint8_t can1_send_data[8];
extern int16_t body_speed[2];
extern double body_rotation;
extern int16_t motor_speed[4];
extern bool isControling;

struct can_feedback
{
	int16_t circle;
	int16_t position;
	int16_t speed;
	int16_t current;
	int16_t voltage;
	uint8_t temperature;
	uint8_t statue;
};

extern struct can_feedback feedback[4];

void can_receive(uint32_t recv_id, uint8_t data[]);

double fill_into(double error);

void remoteControl(int Scalefactor);

void motors_command_receive();

void start_all();
#endif