// Code for getting raw IMU data

#include "inertial.h"

#include <stdlib.h>

#include <android/looper.h>
#include <android/sensor.h>


static ASensorManager *sensorManager;
static ASensorEventQueue *eventQueue;

static const ASensor *gyroSensor;
static const ASensor *magnSensor;
static const ASensor *accelSensor;


static inertial_listener current_listener = NULL;



//int accCounter = 0;
int64_t lastAccTime = 0;

//int gyroCounter = 0;
int64_t lastGyroTime = 0;

//int magCounter = 0;
int64_t lastMagTime = 0;


static int get_sensor_events(int fd, int events, void *data) {

	ASensorEvent eventBuffer[3];

	float accel[3] = {0,0,0};
	float rotat[3] = {0,0,0};
	float magnet[3];


	//float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0, mx = 0, my = 0, mz = 0;
	int64_t accTime;
	int64_t gyroTime;
	int64_t magTime;

	int n = ASensorEventQueue_getEvents(eventQueue, eventBuffer, 3);
	for(int i = 0; i < 2; i++){
		ASensorEvent &event = eventBuffer[i];

		if(event.type == ASENSOR_TYPE_ACCELEROMETER) {
		//	LOGI("accl(x,y,z,t): %f %f %f %lld", event.acceleration.x, event.acceleration.y, event.acceleration.z, event.timestamp);
			accel[0] = event.acceleration.x;
			accel[1] = event.acceleration.y;
			accel[2] = event.acceleration.z;

			accTime = event.timestamp;
		}
		else if(event.type == ASENSOR_TYPE_GYROSCOPE) {
			//LOGI("gyro(x,y,z,t): %f %f %f %lld", event.acceleration.x, event.acceleration.y, event.acceleration.z, event.timestamp);
			rotat[0] = event.acceleration.x;
			rotat[1] = event.acceleration.y;
			rotat[2] = event.acceleration.z;

			gyroTime = event.timestamp;
		}
		else if(event.type == ASENSOR_TYPE_MAGNETIC_FIELD) {
			magnet[0] = event.magnetic.x;
			magnet[1] = event.magnetic.y;
			magnet[2] = event.magnetic.z;

			magTime = event.timestamp;
		}


	//if(vio_inst->recorder.recording()){
	//	vio_inst->recorder.onData(accel, rotat);
	//}
	//vio_inst->impl.propagate_inertial(accel, rotat);


	}



	// TODO: Make sure this only triggers the listener if both accel and gyro were recorded

	if(current_listener != NULL){
		current_listener(accel, rotat);
	}



	float dAt = ((float)(accTime-lastAccTime))/1000000000.0;
	float dGt = ((float)(gyroTime-lastGyroTime))/1000000000.0;
	float dMt = ((float)(magTime-lastMagTime))/1000000000.0;

	//LOGI("%f %f %f", dAt, dGt, dMt);

	lastAccTime = accTime;
	lastGyroTime = gyroTime;
	lastMagTime = magTime;

	//LOGI("%d: %f %f %f %f %f %f %f %f %f",n,ax,ay,az,gx,gy,gz, mx, my, mz);




	//should return 1 to continue receiving callbacks, or 0 to unregister
	return 1;
}



void inertial_enable(){

    ALooper* looper = ALooper_forThread();

    if(looper == NULL)
        looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);


    sensorManager = ASensorManager_getInstance();

    gyroSensor = ASensorManager_getDefaultSensor(sensorManager, ASENSOR_TYPE_GYROSCOPE);
    accelSensor = ASensorManager_getDefaultSensor(sensorManager, ASENSOR_TYPE_ACCELEROMETER);
    //magnSensor = ASensorManager_getDefaultSensor(sensorManager, ASENSOR_TYPE_MAGNETIC_FIELD);


    void *sensor_data = malloc(1024); // TODO: Free this memory on cleanup

    eventQueue = ASensorManager_createEventQueue(sensorManager, looper, 1, get_sensor_events, sensor_data);

    ASensorEventQueue_enableSensor(eventQueue, accelSensor);
    ASensorEventQueue_enableSensor(eventQueue, gyroSensor);
    //ASensorEventQueue_enableSensor(eventQueue, magnSensor);

    //Sampling rate: 100Hz
    int a = ASensor_getMinDelay(accelSensor);
    int b = ASensor_getMinDelay(gyroSensor);
    //int c = ASensor_getMinDelay(magnSensor);

	LOGI("min-delay: %d, %d",a,b);
    //LOGI("min-delay: %d, %d, %d",a,b,c);
    ASensorEventQueue_setEventRate(eventQueue, accelSensor, 10000); // (1000L/SAMP_PER_SEC)*1000
    ASensorEventQueue_setEventRate(eventQueue, gyroSensor, 10000);
    //ASensorEventQueue_setEventRate(eventQueue, magnSensor, 10000);

    LOGI("sensorValue() - START");

}


void inertial_disable(){

    ASensorManager_destroyEventQueue(sensorManager, eventQueue);

}

void inertial_setlistener(inertial_listener l){
	current_listener = l;
}