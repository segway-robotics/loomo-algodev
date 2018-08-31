#ifndef JNI_ALGO_H_
#define JNI_ALGO_H_

#include <jni.h>
#include <android/log.h>
#include <android/bitmap.h>
#include <sys/types.h>
#include <assert.h>
#include <stdlib.h>
#include <stdint.h>
#include "string"
#include "RawDataUtil.h"

JNIEXPORT void JNICALL testAlgoStart(JNIEnv *env, jobject obj, jboolean isSim);
JNIEXPORT void JNICALL testAlgoExit(JNIEnv *env, jobject obj);
JNIEXPORT void JNICALL recordAlgoStart(JNIEnv *env, jobject obj, jboolean isTmp);
JNIEXPORT void JNICALL recordAlgoStop(JNIEnv *env, jobject obj);
JNIEXPORT void JNICALL funcA(JNIEnv *env, jobject obj);
JNIEXPORT void JNICALL funcB(JNIEnv *env, jobject obj);
JNIEXPORT void JNICALL funcC(JNIEnv *env, jobject obj);
JNIEXPORT void JNICALL remoteControl(JNIEnv *env, jobject obj, jfloat v, jfloat w);
JNIEXPORT void JNICALL headTouchEvent(JNIEnv* env, jobject obj, jint event);
JNIEXPORT void JNICALL isEmergencyStop(JNIEnv* env, jobject obj, jboolean is_stop);
JNIEXPORT jboolean JNICALL nativeShow(JNIEnv *env, jobject obj, jobject bitmap);
JNIEXPORT jboolean JNICALL isOpenglView(JNIEnv *env, jobject obj);
JNIEXPORT jstring JNICALL getDebugInfo(JNIEnv *env, jobject obj);
JNIEXPORT void JNICALL newRect(JNIEnv *env, jobject obj, jobject rect);

JNIEXPORT void JNICALL render(JNIEnv *env, jobject obj);
JNIEXPORT void JNICALL onGlSurfaceCreated(JNIEnv *env, jobject obj);
JNIEXPORT void JNICALL setupGraphic(JNIEnv *env, jobject obj, jint width, jint height);
JNIEXPORT void JNICALL nativeOnTouchEvent(JNIEnv *env, jobject obj, int touch_count,
                                          int event, float x0, float y0, float x1, float y1);
JNIEXPORT void JNICALL nativeOnWheelSliding(JNIEnv *env, jobject obj, jint event);

JNIEXPORT jint JNICALL CameraConfigureType(JNIEnv *env, jobject obj);
void playSoundWarning(JNIEnv *env, jint id, jint soundLengthMillis = 800);
bool connectTango(JNIEnv *env);
bool disconnectTango(JNIEnv *env);
std::string getDeviceId(JNIEnv *env);
bool uploadRawMap(JNIEnv *env, std::string native_rawmap_path);
bool downloadOptimizedMap(JNIEnv *env, std::string native_optimizedmap_url,
                          std::string native_md5_sum,
                          std::string native_save_path,
                          std::string native_filename);
void enableWifiAp(JNIEnv *env);
// for init
int init_algo(JNIEnv *env);

#endif
