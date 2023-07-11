#ifndef ALIGNMENT_H
#define ALIGNMENT_H
#include "common.h"

bool isShifted(double error, bool isFront);

void sendDebugInfo(double error, bool isFront);

void sendAllDebugInfo(double error_front, double error_back);

void autoAlignment();

void autoAlignmentOneSide(bool isFront);

void autoAlignmentWithoutshifting();
#endif