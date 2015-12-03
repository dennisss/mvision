#ifndef INERTIAL_H_
#define INERTIAL_H_

#include "vio.h"

#include <stdint.h>

/*
#define INERTIAL_ACCEL 1
#define INERTIAL_GYRO 2

typedef struct {
    int type;
    int64_t timestamp;
    float vector[3];
} inertial_event;
*/

typedef void (*inertial_listener)(float *acc, float *gyro, uint64_t time);

void inertial_init(); // Call at least once to initialize timers, etc.
void inertial_enable(); // Call when you want it to start reading
void inertial_disable(); // Call when you want to stop reading

void inertial_setlistener(inertial_listener listener);





#endif
