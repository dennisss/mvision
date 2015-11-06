
#include "inertial.h"

#include <jni.h>

#include <stdio.h>
#include <unistd.h>
#include <string>


using namespace std;

/* Native part of the RecordingFragment for recording Inertial data */


//static FILE *framefile;
static FILE *datafile;


static int64_t starttime;


static int64_t gettime(){
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec * 1000000000) + now.tv_nsec;
}




static void record_inertial(float *acc, float *gyro){

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    fprintf(datafile, "%lld.%.9ld", (long long)ts.tv_sec, ts.tv_nsec);
    fprintf(datafile, " %f %f %f %f %f %f\n", acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]);
}



#ifdef __cplusplus
extern "C" {
#endif




JNIEXPORT void JNICALL Java_me_denniss_mvision_RecordingFragment_startNativeRecording(JNIEnv* env, jobject obj, jstring jprefix) {

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);


    string prefix = env->GetStringUTFChars(jprefix, NULL);

    datafile = fopen((prefix + ".data").c_str(), "w+");

    fprintf(datafile, "%lld.%.9ld START\n", (long long)ts.tv_sec, ts.tv_nsec);


    // Set listener
    inertial_setlistener(record_inertial);
    inertial_enable();
}

JNIEXPORT void JNICALL Java_me_denniss_mvision_RecordingFragment_stopNativeRecording(JNIEnv* env, jobject obj) {

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);


    // Stop listening
    inertial_disable();
    inertial_setlistener(NULL);

    usleep(10000); // 10 milliseconds should allow for the final sensor readings to be recorded

    fprintf(datafile, "%lld.%.9ld STOP\n", (long long)ts.tv_sec, ts.tv_nsec);

    fflush(datafile);
    fclose(datafile);

    LOGI("SENSORS DONEEEEEE");

}


#ifdef __cplusplus
}
#endif
