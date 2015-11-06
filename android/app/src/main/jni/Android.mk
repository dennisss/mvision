LOCAL_PATH := $(call my-dir)
EXTERNAL:= $(call my-dir)/../../../../ext

include $(CLEAR_VARS)


LOCAL_C_INCLUDES := $(EXTERNAL)/eigen


#OPENCV_CAMERA_MODULES := off
#OPENCV_INSTALL_MODULES := off
#OPENCV_LIB_TYPE := SHARED
include $(EXTERNAL)/opencv/sdk/native/jni/OpenCV.mk



LOCAL_MODULE := libvio
#LOCAL_SHARED_LIBRARIES :=
#LOCAL_STATIC_LIBRARIES := libavcodec libavutil
LOCAL_CFLAGS := -std=c++11 -O3 #-D__STDC_CONSTANT_MACROS # This last definition is required by ffmpeg

LOCAL_SRC_FILES := init.cpp \
                   vio.cpp \
                   recording.cpp \
                   inertial.cpp



#LOCAL_LDLIBS := -llog
include $(BUILD_SHARED_LIBRARY)





#include $(EXTERNAL)/ffmpeg/Android.mk
