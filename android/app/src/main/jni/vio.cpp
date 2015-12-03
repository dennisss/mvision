
#include "vio.h"


#include <jni.h>
#include <time.h>

/*
#include <opencv2/core/core.hpp>

using namespace cv;
*/


#ifdef __cplusplus
extern "C" {
#endif




JNIEXPORT jlong JNICALL Java_me_denniss_mvision_vio_VisualInertialOdometry_ctor(JNIEnv* env){
    return (jlong) new VIO();
}

JNIEXPORT void JNICALL Java_me_denniss_mvision_vio_VisualInertialOdometry_destructor(JNIEnv* env, jlong inst){
    delete ((VIO *) inst);
}


JNIEXPORT void JNICALL Java_me_denniss_mvision_vio_VisualInertialOdometry_onNativeCameraFrame(JNIEnv* env, jlong inst, jlong frameptr){

    VIO *self = (VIO *) inst;
    //Mat &img = *(Mat *) frameptr;

    clock_t t = clock();

    //self->impl.update_image(img);

	//if(self->recorder.recording()){
	//	self->recorder.onFrame(img);
	//}



    t = clock() - t;
    LOGI("fps: %f", 1.0f / (((float)t)/CLOCKS_PER_SEC));
}


JNIEXPORT void JNICALL Java_me_denniss_mvision_vio_VisualInertialOdometry_nativeStart(JNIEnv* env, jlong inst){
    VIO *self = (VIO *) inst;
    self->start();
}

JNIEXPORT void JNICALL Java_me_denniss_mvision_vio_VisualInertialOdometry_nativeStop(JNIEnv* env, jlong inst){
    VIO *self = (VIO *) inst;
    self->stop();
}


JNIEXPORT void JNICALL Java_me_denniss_mvision_vio_VisualInertialOdometry_nativeStartRecording(JNIEnv* env, jlong inst){
    VIO *self = (VIO *) inst;
    //self->recorder.start();
}

JNIEXPORT void JNICALL Java_me_denniss_mvision_vio_VisualInertialOdometry_nativeStopRecording(JNIEnv* env, jlong inst){
    VIO *self = (VIO *) inst;
    //self->recorder.stop();
}

#ifdef __cplusplus
}





void VIO::start(){


    //this->startSensors();

}



void VIO::stop(){
    //this->stopSensors();

}


#endif
