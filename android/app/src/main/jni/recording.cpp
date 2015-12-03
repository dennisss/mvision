
#include "inertial.h"

#include <jni.h>

#include <stdio.h>
#include <unistd.h>
#include <string>


using namespace std;

/* Native part of the RecordingFragment for recording Inertial data */


//static FILE *framefile;
static FILE *datafile;



static void record_inertial(float *acc, float *gyro, uint64_t time){
    fprintf(datafile, "%llu %f %f %f %f %f %f\n", time, acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]);
}



#ifdef __cplusplus
extern "C" {
#endif




JNIEXPORT void JNICALL Java_me_denniss_mvision_RecordingFragment_startNativeRecording(JNIEnv* env, jobject obj, jstring jprefix) {

    inertial_init();

    string prefix = env->GetStringUTFChars(jprefix, NULL);

    datafile = fopen((prefix + ".data").c_str(), "w+");

    // Set listener
    inertial_setlistener(record_inertial);
    inertial_enable();
}

JNIEXPORT void JNICALL Java_me_denniss_mvision_RecordingFragment_stopNativeRecording(JNIEnv* env, jobject obj) {

    // Stop listening
    inertial_disable();
    inertial_setlistener(NULL);

    usleep(10000); // 10 milliseconds should allow for the final sensor readings to be recorded

    fflush(datafile);
    fclose(datafile);

    LOGI("SENSORS DONE");
}


#ifdef __cplusplus
}
#endif
