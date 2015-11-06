#ifndef VIO_H_
#define VIO_H_


//#include "msckf.h"

//#include "recorder_old.h"


#ifdef __ANDROID__

#include <android/log.h>
#define LOG_TAG "libvio"
#define LOGI(fmt, args...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, fmt, ##args)
#define LOGD(fmt, args...) __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, fmt, ##args)
#define LOGE(fmt, args...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, fmt, ##args)

#endif


class VIO {


public:

    void start();
    void stop();


    //MSCKF impl;

	//Recorder recorder;

private:

    void startSensors();
    void stopSensors();
};

// Emits IMU readings
class InertialProvider {


};




#endif
