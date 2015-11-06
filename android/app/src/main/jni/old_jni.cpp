JavaVM *jvm;

vector<jobject> java_listeners;


jclass msgClass, dataPointClass;


// Forwards messages to java
void jni_onmessage(int type, char *data, int dsize){
    // Ignore data points if the data listener has not been set yet
    if(java_listeners.size() == 0)
        return;


    JNIEnv *env;

    bool tempEnv = false;
    int res = jvm->GetEnv((void **)&env, JNI_VERSION_1_6);
    if(res == JNI_OK){

    }
    else{
        if(res == JNI_EDETACHED) {
            //LOGE("JNI ENV Not attached");

            tempEnv = true;

            if (jvm->AttachCurrentThread(&env, NULL) != 0) {
                LOGE("JNI ENV Failed to attach");
                return;
            }

        }
        else if(res == JNI_EVERSION){
            LOGE("JNI ENV Version Mismatch");
            return;
        }
        else{
            LOGE("JNI ENV Unknown Error");
            return;
        }
    }



    jmethodID msgCtor = env->GetMethodID(msgClass, "<init>", "()V");
    jfieldID msgType = env->GetFieldID(msgClass, "type", "I");
    jfieldID msgData = env->GetFieldID(msgClass, "data", "Ljava/lang/Object;");


    if (env->ExceptionCheck()) {
        env->ExceptionDescribe();
    }


    // Initialize the java message object that will hold the transfered data
    jobject msgObj = env->NewObject(msgClass, msgCtor);
    env->SetIntField(msgObj, msgType, type);

    if(type == CTRL_MESSAGE_POSE){ // A native JNI data transfer implementation for fast pose updates

        jmethodID ctor = env->GetMethodID(dataPointClass, "<init>", "()V");
        jfieldID fPos = env->GetFieldID(dataPointClass, "pos", "[F");
        jfieldID fOrient = env->GetFieldID(dataPointClass, "orient", "[F");

        // Create a java array in which to store the data
        int npoints = dsize / sizeof(data_point);

        jobjectArray arr = env->NewObjectArray(npoints, dataPointClass, NULL);
        for(int i = 0; i < npoints; i++) {

            data_point *nPoint = ((data_point *)data) + i;
            jobject jPoint = env->NewObject(dataPointClass, ctor);

            // Get instance position array
            jobject jPosField = env->GetObjectField(jPoint, fPos);
            jfloatArray *jPosArr = reinterpret_cast<jfloatArray*>(&jPosField);

            // Get instance orientation array
            jobject jOrientField = env->GetObjectField(jPoint, fOrient);
            jfloatArray *jOrientArr = reinterpret_cast<jfloatArray*>(&jOrientField);

            // Put the data into the point
            env->SetFloatArrayRegion(*jPosArr, 0, 3, nPoint->pos);
            env->SetFloatArrayRegion(*jOrientArr, 0, 4, nPoint->orient);


            // Put the point into the array
            env->SetObjectArrayElement(arr, i, jPoint);

            env->DeleteLocalRef(jPoint);
            env->DeleteLocalRef(jPosField);
            env->DeleteLocalRef(jOrientField);
        }

        env->SetObjectField(msgObj, msgData, arr);

        env->DeleteLocalRef(arr);


    }
    else{ // Non-native types will be passed to java as ByteBuffers
        if(dsize > 0){
            jobject jBuffer = env->NewDirectByteBuffer(data, dsize);
            env->SetObjectField(msgObj, msgData, jBuffer);
        }
    }

    for(int i = 0; i < java_listeners.size(); i++){
        jobject l = java_listeners[i];

        jclass clazz =  env->GetObjectClass(l);
        jmethodID method = env->GetMethodID(clazz, "onMessage", "(Ledu/upenn/grasp/quaddata/net/Message;)V");

        env->CallVoidMethod(l, method, msgObj);

        env->DeleteLocalRef(clazz);
    }

    env->DeleteLocalRef(msgObj);

    if(tempEnv){
        //LOGI("Detach");
        jvm->DetachCurrentThread();
    }
}


















//  edu.upenn.grasp.quaddata.ControlProvider.start
JNIEXPORT void JNICALL Java_edu_upenn_grasp_quaddata_net_ControlProvider_start(JNIEnv* env, jobject obj){
ControlProvider &inst = ControlProvider::inst();

inst.start();
inst.addMessageListener(jni_onmessage);
}

JNIEXPORT void JNICALL Java_edu_upenn_grasp_quaddata_net_ControlProvider_stop(JNIEnv* env, jobject obj){
ControlProvider &inst = ControlProvider::inst();

inst.stop();
inst.removeMessageListener(jni_onmessage);
}




JNIEXPORT void JNICALL Java_edu_upenn_grasp_quaddata_net_ControlProvider_connect(JNIEnv* env, jobject obj){
ControlProvider::inst().connect();
}


JNIEXPORT void JNICALL Java_edu_upenn_grasp_quaddata_net_ControlProvider_send(JNIEnv* env, jobject obj, jobject msg){
jfieldID msgType = env->GetFieldID(msgClass, "type", "I");

int type = env->GetIntField(msg, msgType);

ControlProvider &inst = ControlProvider::inst();

inst.send(type, NULL, 0);
}

JNIEXPORT void JNICALL Java_edu_upenn_grasp_quaddata_net_ControlProvider_sendSetPose(JNIEnv* env, jobject obj, jfloat x, jfloat y, jfloat z){

data_point p;
p.pos[0] = x;
p.pos[1] = y;
p.pos[2] = z;

ControlProvider::inst().send(CTRL_COMMAND_SETPOSE, (char*)&p, sizeof(data_point));
}



JNIEXPORT void JNICALL Java_edu_upenn_grasp_quaddata_net_ControlProvider_addMessageListener(JNIEnv* env, jobject obj, jobject l){
ControlProvider &inst = ControlProvider::inst();

// Ensure not double listening
for(int i = 0; i < java_listeners.size(); i++){
if(env->IsSameObject(java_listeners[i], l))
return;
}

jobject lref = reinterpret_cast<jobject>(env->NewGlobalRef(l));

java_listeners.push_back(lref);
LOGI("ADDED!");
}

JNIEXPORT void JNICALL Java_edu_upenn_grasp_quaddata_net_ControlProvider_removeMessageListener(JNIEnv* env, jobject obj, jobject l){
ControlProvider &inst = ControlProvider::inst();

for(int i = 0; i < java_listeners.size(); i++){
if(env->IsSameObject(java_listeners[i], l)){
env->DeleteGlobalRef(java_listeners[i]);
java_listeners.erase(java_listeners.begin() + i);
LOGI("REMOVED!");
break;
}
}

}



JNIEXPORT jstring JNICALL Java_edu_upenn_grasp_quaddata_net_ControlProvider_getIpAddress(JNIEnv* env, jobject obj){
    ControlProvider &inst = ControlProvider::inst();

    string ip = inst.getIpAddress();
    return env->NewStringUTF(ip.c_str());
}

JNIEXPORT jboolean JNICALL Java_edu_upenn_grasp_quaddata_net_ControlProvider_setIpAddress(JNIEnv* env, jobject obj, jstring jstr){
    ControlProvider &inst = ControlProvider::inst();

    string ip = (char *)env->GetStringUTFChars(jstr, NULL);
    return inst.setIpAddress(ip);
}


JNIEXPORT jint JNICALL Java_edu_upenn_grasp_quaddata_net_ControlProvider_getPort(JNIEnv* env, jobject obj){
    ControlProvider &inst = ControlProvider::inst();

    return  inst.getPort();
}

JNIEXPORT void JNICALL Java_edu_upenn_grasp_quaddata_net_ControlProvider_setPort(JNIEnv* env, jobject obj, jint port){
ControlProvider &inst = ControlProvider::inst();

inst.setPort(port);
}
