#ifndef RECORDER_H_
#define RECORDER_H_

//#include "video_encode.h"

#include <stdio.h>
#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>

using namespace cv;


/*
	Saves a log of IMU and camera data captured into /sdcard/MVision

	The file structure is:

	- TIME.avi    : The full color stream from the camera
	- TIME.frames : The timestamps at which each frame in the video was captured
		- Each line is ascii and is of the form "FRAME# TIMESTAMP"
	- TIME.data    : The raw measurements from the accelerometer and gyroscope
		- Each line is ascii and is of the form "TIMESTAMP ACC_X ACC_Y ACC_Z ROT_X ROT_Y ROT_Z"


*/



class Recorder {

public:
	Recorder();

	void start();
	void stop();
	bool recording();



	void onFrame(Mat &frame);
	void onData(float *acc, float *gyro);

private:

	bool is_recording = false;


	long int timestamp;

	int framenum = 0;


	//VideoEncoder *encoder;

	//VideoWriter *videowriter;
	FILE *framefile;
	FILE *datafile;

};










#endif
