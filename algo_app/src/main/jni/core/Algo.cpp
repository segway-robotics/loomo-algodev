#include "jniLocomotion.h"
#include "jniVision.h"
#include "jniAlgo.h"
#include "RawDataUtil.h"

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <stdlib.h>
#include <unistd.h>

#include "ninebot_log.h"

using namespace ninebot_algo;
using namespace sdk_vision;

jobject mBaseJavaObj;

jmethodID playSoundWarning_id;
jmethodID connectTango_id;
jmethodID disconnectTango_id;
jmethodID getDeviceId_id;
jmethodID uploadRawMap_id;
jmethodID downloadOptimizedMap_id;
jmethodID enableWifiAp_id;

void playSoundWarning(JNIEnv *env, jint id, jint soundLengthMillis) {
    env->CallVoidMethod(mBaseJavaObj, playSoundWarning_id, id, soundLengthMillis);
}

bool connectTango(JNIEnv *env) {
    return env->CallBooleanMethod(mBaseJavaObj, connectTango_id);
}

bool disconnectTango(JNIEnv *env) {
    return env->CallBooleanMethod(mBaseJavaObj, disconnectTango_id);
}

bool uploadRawMap(JNIEnv *env, std::string native_rawmap_path) {
    jstring rawmap_path = env->NewStringUTF(native_rawmap_path.c_str());
    return env->CallBooleanMethod(mBaseJavaObj, uploadRawMap_id, rawmap_path);
}

bool downloadOptimizedMap(JNIEnv *env, std::string native_optimizedmap_url, std::string native_md5_sum,
                          std::string native_save_path, std::string native_filename) {
    jstring optimizedmap_url = env->NewStringUTF(native_optimizedmap_url.c_str());
    jstring md5_sum = env->NewStringUTF(native_md5_sum.c_str());
    jstring save_path = env->NewStringUTF(native_save_path.c_str());
    jstring filename = env->NewStringUTF(native_filename.c_str());
    return env->CallBooleanMethod(mBaseJavaObj, downloadOptimizedMap_id, optimizedmap_url, md5_sum,
                                 save_path, filename);
}

std::string getDeviceId(JNIEnv *env) {
    jstring j_device_id = static_cast<jstring>(env->CallObjectMethod(mBaseJavaObj, getDeviceId_id));
    const char *device_id_chars = NULL;
    const int SIZE = 512;
    char buf[SIZE];

    device_id_chars = j_device_id ? env->GetStringUTFChars(j_device_id, NULL) : NULL;
    if (!device_id_chars) {
        ALOGE("device id is null");
        return "";
    }

    return std::string(device_id_chars);
}

void enableWifiAp(JNIEnv *env) {
    return env->CallVoidMethod(mBaseJavaObj, enableWifiAp_id);
}

JNIEXPORT void JNICALL init(JNIEnv *env, jobject obj, jobject instance)
{
    ALOGD("Algo init start...");
    jclass baseJavaClass = env->GetObjectClass(instance);

    playSoundWarning_id = env->GetMethodID(baseJavaClass, "playSoundWarning", "(II)V");
    connectTango_id = env->GetMethodID(baseJavaClass, "connectTango", "()Z");
    disconnectTango_id = env->GetMethodID(baseJavaClass, "disconnectTango", "()Z");
    getDeviceId_id = env->GetMethodID(baseJavaClass, "getDeviceId", "()Ljava/lang/String;");
    uploadRawMap_id = env->GetMethodID(baseJavaClass, "uploadRawMap", "(Ljava/lang/String;)Z");
    downloadOptimizedMap_id = env->GetMethodID(baseJavaClass, "downloadOptimizedMap",
                                               "(Ljava/lang/String;Ljava/lang/String;"
                                               "Ljava/lang/String;Ljava/lang/String;)Z");
    enableWifiAp_id = env->GetMethodID(baseJavaClass, "enableWifiAp", "()V");

    mBaseJavaObj = env->NewGlobalRef(instance);

    env->DeleteLocalRef(jobject(baseJavaClass));
    ALOGD("Algo init end...");
}

static int readFromFile(const char *path, char* buf, size_t size, bool throwError)
{
    if (!path)
        return -1;

    int fd = open(path, O_RDONLY, 0);
    if (fd < 0) {
        if (throwError) {
            ALOGE("Could not open '%s'", path);
        }
        return -1;
    }

    // Pass (size - 1) to append '\0' at last
    ssize_t count = read(fd, buf, size - 1);
    if (count > 0) {
        while (count > 0 && buf[count-1] == '\n')
            count--;
        count-=3;
        buf[count] = '\0';
    } else {
        buf[0] = '\0';
    }

    close(fd);
    return count;
}

JNIEXPORT jstring JNICALL readSysfs(JNIEnv* env, jobject obj, jstring jPath)
{
    const char *path = NULL;
    const int SIZE = 512;
    char buf[SIZE];

    path = jPath ? env->GetStringUTFChars(jPath, NULL) : NULL;
    if (!path) {
        ALOGE("path is null");
        return NULL;
    }

    if (readFromFile(path, buf, SIZE, false) > 0) {
        env->ReleaseStringUTFChars(jPath, path);
        return env->NewStringUTF(buf);
    } else {
        env->ReleaseStringUTFChars(jPath, path);
        return NULL;
    }
}

static JNINativeMethod methodTable[] = {
        {"native_readSysfs", "(Ljava/lang/String;)Ljava/lang/String;", (void*)readSysfs},
        {"init","(Lcom/segway/robot/algo/algo_app/Algo;)V", (void *) init},
        {"testAlgoStart","(Z)V", (void *) testAlgoStart},
        {"testAlgoExit","()V", (void *) testAlgoExit},
        {"recordAlgoStart","(Z)V", (void *) recordAlgoStart},
        {"recordAlgoStop","()V", (void *) recordAlgoStop},
        {"nativeShow","(Landroid/graphics/Bitmap;)Z", (void *) nativeShow},
        {"getDebugInfo","()Ljava/lang/String;", (void *) getDebugInfo},
        {"isOpenglView","()Z", (void *) isOpenglView},
        {"CameraConfigureType","()I", (void *) CameraConfigureType},
        {"newRect","(Landroid/graphics/Rect;)V", (void *) newRect},
        {"funcA","()V", (void *) funcA},
        {"funcB","()V", (void *) funcB},
        {"funcC","()V", (void *) funcC},
        {"remoteControl","(FF)V", (void *) remoteControl},
        {"render","()V",(void *) render},
        {"onGlSurfaceCreated","()V",(void *) onGlSurfaceCreated},
        {"setupGraphic","(II)V",(void *) setupGraphic},
        {"nativeOnTouchEvent","(IIFFFF)V",(void *) nativeOnTouchEvent},
        {"nativeHeadTouchEvent","(I)V",(void *) headTouchEvent},
        {"nativeIsEmergencyStop","(Z)V",(void *) isEmergencyStop},
        {"nativeOnWheelSliding","(I)V",(void *) nativeOnWheelSliding},
};

const char* aprServicePath = "com/segway/robot/algo/algo_app/AlgoNativeInterface";

const jboolean PACKAGE_IS_SERVICE = true;
int init_algo(JNIEnv * env)
{
	ALOGD ("Jni init algo start...");

    jclass activityClass = env->FindClass(aprServicePath);
	if (!activityClass)
	{
		ALOGE("failed to get %s class reference", aprServicePath);
		return -1;
	}

    env->RegisterNatives(activityClass, methodTable, sizeof(methodTable) / sizeof(methodTable[0]));

    if (sdk_locomotion::init_locomotion(env)) {
        ALOGE("failed to init locomotion");
        return -1;
    }

    if (sdk_vision::init_vision(env)) {
        ALOGE("failed to init vision");
        return -1;
    }

    ALOGD ("Jni init algo end...");
    return 0;
}


//}
