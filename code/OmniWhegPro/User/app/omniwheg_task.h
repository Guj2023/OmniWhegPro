#ifndef __OMNIWHEG_TASK_H__
#define __OMNIWHEG_TASK_H__

#include "common.h"
#include "servo_control.h"
#include "alignment.h"

void omniwheg_task(const void*argu);
void communication_task(const void*argu);
void main_loop();
void motors_command_receive();
void work_loop();
#endif
