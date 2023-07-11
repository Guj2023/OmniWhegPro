#include "alignment.h"

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

