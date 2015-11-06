#ifndef INERTIAL_H_
#define INERTIAL_H_

#include "vio.h"

typedef void (*inertial_listener)(float *acc, float *gyro);

void inertial_enable();
void inertial_disable();

void inertial_setlistener(inertial_listener listener);

#endif
