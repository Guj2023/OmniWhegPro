#ifndef __STUDY_TASK_H__
#define __STUDY_TASK_H__

#include "common.h"
#include "servo_control.h"
#include "alignment.h"

void study_task(const void*argu);
void main_loop();
void motors_command_receive();

#endif
