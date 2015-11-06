#include "recorder_old.h"

#include "vio.h"

#include <time.h>
#include <stdint.h>


Recorder::Recorder(){

}

static int64_t starttime;


static int64_t gettime(){
	struct timespec now;
	clock_gettime(CLOCK_MONOTONIC, &now);
	return (now.tv_sec * 1000000000) + now.tv_nsec;
}


void Recorder::start(){

	starttime = gettime();

	char name[256];

	long int t = static_cast<long int>(time(NULL));

	this->timestamp = t;

	// Start video
	sprintf(name, "/sdcard/MVision/%li.mpg", t);
	//encoder = new VideoEncoder(name);
	//videowriter = new VideoWriter(name, CV_FOURCC('M','J','P','G'), 24, Size(1920, 1080), true);

	sprintf(name, "/sdcard/MVision/%li.frames", t);
	framefile = fopen(name, "w+");

	sprintf(name, "/sdcard/MVision/%li.data", t);
	datafile = fopen(name, "w+");

	is_recording = true;



}

void Recorder::stop(){
	is_recording = false;

	//delete encoder;

	fflush(framefile);
	fclose(framefile);

	fflush(datafile);
	fclose(framefile);
}


bool Recorder::recording(){
	return is_recording;
}






void Recorder::onFrame(Mat &frame){

	LOGI("GOT FRAME");


	char name[256];
	//sprintf(name, "/sdcard/MVision/%li-%d.jpg", timestamp, framenum++);
	//imwrite(name, frame);

//	videowriter->write(frame);

//	encoder->encode(frame);

	fprintf(framefile, "%lli\n", gettime() - starttime);
}


void Recorder::onData(float *acc, float *gyro){
	LOGI("GOT DATA");
	fprintf(datafile, "%lli %f %f %f %f %f %f\n", gettime() - starttime, acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]);
}
