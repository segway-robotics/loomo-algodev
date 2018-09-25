#include "jniAlgo.h"
#include "jniVision.h"
#include "ImuProcess.h"

#include "ninebot_log.h"

using namespace ninebot_algo;

bool mAlgoStarted = false;
/***********************************************************
********** Test (two algo threads) *************************
***********************************************************/
#if defined(APP_TEST)
#include "AlgoTest.h"

RawData *pAlgoMainRawData = NULL;
test_algo::AlgoTest* pAlgoMain = NULL;
std::string app_name = "APP_TEST";

JNIEXPORT void JNICALL testAlgoStart(JNIEnv *env, jobject obj, jboolean isSim)
{
    pAlgoMainRawData = new RawData();
    RawData::StartSensors(isSim, "/sdcard/RawDataRec/tmp/");
    pAlgoMainRawData->attachJVM();
    pAlgoMain = new test_algo::AlgoTest(pAlgoMainRawData, 10);
    test_algo::test_params_t cParams;
    pAlgoMain->init(cParams);
    pAlgoMain->start();
    mAlgoStarted = true;
}

/***********************************************************
********** CA *************************************
***********************************************************/
#elif defined(APP_LOCALMAPPING_TEST)
#include "AlgoLocalmappingTest.h"

RawData *pAlgoMainRawData = NULL;
localmapping_test::AlgoLocalmappingTest* pAlgoMain = NULL;

OutputBucket ob[2];
std::string app_name = "APP_Localmapping_Test";

JNIEXPORT void JNICALL testAlgoStart(JNIEnv *env, jobject obj, jboolean isSim)
{
    enableWifiAp(env);
    pAlgoMainRawData = new RawData();
    RawData::StartSensors(isSim, "/sdcard/RawDataRec/tmp/");
    pAlgoMainRawData->attachJVM();
    pAlgoMain = new localmapping_test::AlgoLocalmappingTest(pAlgoMainRawData, 100);
    pAlgoMain->start();
    mAlgoStarted = true;
}

/***********************************************************
********** TENSORFLOW SAMPLE *******************************
***********************************************************/
#elif defined(APP_TENSORFLOW_SAMPLE)

#include "AlgoTensorflowSample.h"
RawData *pAlgoMainRawData = NULL;
ninebot_tensorflow::AlgoTensorflowSample* pAlgoMain = NULL;
std::string app_name = "APP_TENSORFLOW_SAMPLE";

JNIEXPORT void JNICALL testAlgoStart(JNIEnv *env, jobject obj, jboolean isSim)
{
    pAlgoMainRawData = new RawData();
    RawData::StartSensors(isSim, "/sdcard/RawDataRec/tmp/");
    pAlgoMainRawData->attachJVM();
    pAlgoMain = new ninebot_tensorflow::AlgoTensorflowSample(pAlgoMainRawData, 33, true);
    pAlgoMain->start();
    mAlgoStarted = true;
}

/***********************************************************
********** Socket *************************
***********************************************************/
#elif defined(APP_SOCKET)
#include "AlgoSocket.h"

RawData *pAlgoMainRawData = NULL;
socket_algo::AlgoSocket* pAlgoMain = NULL;
std::string app_name = "APP_SOCKET";

JNIEXPORT void JNICALL testAlgoStart(JNIEnv *env, jobject obj, jboolean isSim)
{
    pAlgoMainRawData = new RawData();
    RawData::StartSensors(isSim, "/sdcard/RawDataRec/tmp/");
    pAlgoMainRawData->attachJVM();
    pAlgoMain = new socket_algo::AlgoSocket(pAlgoMainRawData, 50);
    socket_algo::test_params_t cParams;
    pAlgoMain->init(cParams);
    pAlgoMain->start();
    mAlgoStarted = true;
}

#endif

int camera_index = 2;
bool useAutoExp = false;
bool useVLS = false;

JNIEXPORT void JNICALL funcA(JNIEnv *env, jobject obj){
#if defined(APP_TEST)
    if(pAlgoMain){
        pAlgoMain->toggleImgStream();
        //useVLS = !useVLS;
        //pAlgoMain->setVLSopen(useVLS);
    }
#elif defined(APP_LANDMARK_FOLLOWER)
    if(pAlgoMain){
        pAlgoMain->setVelocity(0.0, 0.1);
    }
#elif defined(APP_LOCALMAPPING_TEST)
    if(pAlgoMain){
        pAlgoMain->clearMap(1);
    }
#elif defined(APP_SOCKET)
    if(pAlgoMain){
        pAlgoMain->switchSafetyControl();
    }    
#endif
}

JNIEXPORT void JNICALL funcB(JNIEnv *env, jobject obj){
#if defined(APP_TEST)
    if(pAlgoMain){
        pAlgoMain->startMotionTest();
    }
#endif
}

JNIEXPORT void JNICALL funcC(JNIEnv *env, jobject obj){
#if defined(APP_TEST)
    if(pAlgoMain){
        pAlgoMain->changeTfTestMode();
    }
#endif
}

JNIEXPORT void JNICALL remoteControl(JNIEnv *env, jobject obj, jfloat v, jfloat w)
{
    if(pAlgoMain)
    {
#if defined(APP_LOCALMAPPING_TEST)
        //pAlgoMain->remoteControl(v,w);
#endif
        ALOGD("remoteControl: %f, %f", v, w);
    }
}

// event: 1 touched, 0 released
JNIEXPORT void JNICALL headTouchEvent(JNIEnv* env, jobject obj, jint event)
{
    ALOGD("headTouchEvent: %d", event);
}

// event: 1 touched, 0 released
JNIEXPORT void JNICALL isEmergencyStop(JNIEnv* env, jobject obj, jboolean is_stop)
{
    ALOGD("isEmergencyStop: %d", is_stop);
}

// event: TODO: 0-left wheel, 1-right wheel
JNIEXPORT void JNICALL nativeOnWheelSliding(JNIEnv *env, jobject obj, jint event)
{
    ALOGD("slide event: %d", event);
}


JNIEXPORT void JNICALL testAlgoExit(JNIEnv *env, jobject obj)
{
    if(!mAlgoStarted)
        return;
    mAlgoStarted = false;
    if(pAlgoMain){
        pAlgoMain->exit();
        delete pAlgoMain;
        pAlgoMain = NULL;
    }
    if(pAlgoMainRawData){
        delete pAlgoMainRawData;
        pAlgoMainRawData = NULL;
    }
    RawData::StopSensors();
}

JNIEXPORT void JNICALL recordAlgoStart(JNIEnv *env, jobject obj, jboolean isTmp)
{
    if(pAlgoMainRawData){
        //int64_t t0 = pAlgoMainRawData->getCurrentTimestampSys();
        int64_t t0 = 0; // TODO: remove time offset
        std::string rfolder;
        rfolder = "/sdcard/RawDataRec/";
        std::string cmd_str_mk = "mkdir \"" + rfolder + "\"";
        system(cmd_str_mk.c_str());
        if(isTmp)
            rfolder = "/sdcard/RawDataRec/tmp/";
        else
        {
            time_t t = time(NULL);
            struct tm tm = *localtime(&t);
            char dt[100];
            sprintf(dt, "%04d-%02d-%02d_%02d-%02d-%02d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
            rfolder = "/sdcard/RawDataRec/"+ string(dt) + "/";
        }
#if defined(APP_TEST)
        pAlgoMainRawData->StartRecording(rfolder,t0);
        pAlgoMain->startPoseRecord(rfolder,t0);
#elif defined(APP_SOCKET)
        pAlgoMain->startPoseRecord(rfolder,t0);        
#else
        pAlgoMainRawData->StartRecording(rfolder,t0);
#endif

        if(IMUProcess::getInstance())
            IMUProcess::getInstance()->startIMUrecording(rfolder,t0);
        //record camera parameter
        //CalibrationInfoDS4T calib = sdk_vision::getCalibrationInfoDS4T(env);
        CalibrationInfoDS4T calib;
        pAlgoMainRawData->getCalibrationDS4T(calib);
        std::string camerapara_name = rfolder + "cameraParam.txt";
        std::string calibStr = pAlgoMainRawData->printDS4TCalibration(calib);
        std::ofstream camera_para_file;
        camera_para_file.open(camerapara_name, std::ofstream::out);
        camera_para_file<< calibStr.c_str();
        camera_para_file.close();
        //record main camera parameter
        RawCameraIntrinsics camerapara;
        pAlgoMainRawData->getMaincamParam(camerapara);
        std::string camerapara_name2 = rfolder + "maincamParam.txt";
        std::ofstream camera_para_file2;
        camera_para_file2.open(camerapara_name2, std::ofstream::out);
        camera_para_file2<< camerapara.focalLengthX << " " << camerapara.focalLengthY << " " << camerapara.pricipalPointX << " " << camerapara.pricipalPointY << " " << camerapara.distortion[0] << " " << camerapara.distortion[1] << " " << camerapara.distortion[2] << " " << camerapara.distortion[3] << " " << camerapara.distortion[4];
        camera_para_file2.close();
    }
}

JNIEXPORT void JNICALL recordAlgoStop(JNIEnv *env, jobject obj)
{
    if(pAlgoMainRawData){
#if defined(APP_TEST)
        pAlgoMain->stopPoseRecord();
        pAlgoMainRawData->StopRecording();
#else
        pAlgoMainRawData->StopRecording();
#endif
        if(IMUProcess::getInstance())
            IMUProcess::getInstance()->stopIMUrecording();
    }
}

// GL view related
JNIEXPORT jboolean JNICALL isOpenglView(JNIEnv *env, jobject obj) {
#if defined(APP_VIO_INTEL)
    return JNI_TRUE;
#else
    return JNI_FALSE;
#endif
}

JNIEXPORT void JNICALL render(JNIEnv *env, jobject obj){
#if defined(APP_VIO_INTEL)
    if(pAlgoMain)
        pAlgoMain->Render();
#endif
}

JNIEXPORT void JNICALL onGlSurfaceCreated(JNIEnv*, jobject) {
#if defined(APP_VIO_INTEL)
    if(pAlgoMain)
        pAlgoMain->InitializeGLContent();
#endif
}

JNIEXPORT void JNICALL setupGraphic(JNIEnv*, jobject, jint width, jint height) {
#if defined(APP_VIO_INTEL)
    if(pAlgoMain)
        pAlgoMain->SetViewPort(width, height);
#endif
}

JNIEXPORT void JNICALL nativeOnTouchEvent(JNIEnv *env, jobject obj, int touch_count, int event, float x0, float y0, float x1, float y1){
#if defined(APP_VIO_INTEL)
    TouchEvent touch_event =
            static_cast<TouchEvent>(event);
    if(pAlgoMain)
        pAlgoMain->OnTouchEvent(touch_count, touch_event, x0, y0, x1, y1);
#endif
}

JNIEXPORT jboolean JNICALL nativeShow(JNIEnv *env, jobject obj, jobject bitmap)
{
    AndroidBitmapInfo  info;
    void*              pixels;
    int ret = 0;

    if ((ret = AndroidBitmap_getInfo(env, bitmap, &info)) < 0) {
        ALOGD("AndroidBitmap_getInfo() failed ! error=%d", ret);
        return JNI_FALSE;
    }
    if (info.format != ANDROID_BITMAP_FORMAT_RGBA_8888) {
        ALOGD("Bitmap format is not RGBA_8888 !");
    }

    while ((ret = AndroidBitmap_lockPixels(env, bitmap, &pixels)) < 0) {
        //        ALOGD("jniNativeShowMap AndroidBitmap_lockPixels() failed ! error=%d", ret);
    }

    ret=JNI_TRUE;

    if(pAlgoMain==NULL || mAlgoStarted==false)
        ret = JNI_FALSE;

    if(ret==JNI_TRUE) {
        if (!(pAlgoMain->showScreen(pixels)))
            ret = JNI_FALSE;
    }

    AndroidBitmap_unlockPixels(env, bitmap);
    return ret;

}

JNIEXPORT jstring JNICALL getDebugInfo(JNIEnv *env, jobject obj) {
    //char sample[100]  = "";

    string contents;
    if(pAlgoMain && mAlgoStarted){
        contents = app_name + " - run time: " + ToString(pAlgoMain->runTime()) + " ms";
    }
    else
        contents = app_name + " - run time: ";

#if (defined(APP_LOCALMAPPING_TEST))
    if(pAlgoMain)
        contents = contents + pAlgoMain->getDebugString();
#endif

    return env->NewStringUTF(contents.c_str());
}

JNIEXPORT void JNICALL newRect(JNIEnv *env, jobject obj, jobject rect)
{
    jclass cls = env->GetObjectClass(rect);

    jfieldID left_id = env->GetFieldID(cls, "left", "I");
    jfieldID top_id = env->GetFieldID(cls, "top", "I");
    jfieldID right_id = env->GetFieldID(cls, "right", "I");
    jfieldID bottom_id = env->GetFieldID(cls, "bottom", "I");

    int left = env->GetIntField(rect, left_id);
    int top = env->GetIntField(rect, top_id);
    int right = env->GetIntField(rect, right_id);
    int bottom = env->GetIntField(rect, bottom_id);

    env->DeleteLocalRef(cls);

    // if canvas is 640x360, then need factor=2 to upscale the rect
    cv::Rect roi;
    roi.x = left*2;
    roi.y = top*2;
    roi.width = (right-left)*2;
    roi.height = (bottom-top)*2;
    ALOGD("newRect:%d,%d,%d,%d",roi.x,roi.y,roi.width,roi.height);
}

void checkFallAction(float ir1, float ir2)
{
}

enum CONFIGURE_CAMERA_LIST
{
    USE_DEPTH = 1,
    USE_DS4COLOR_VGA = 2,
    USE_FISHEYE = 4,
    USE_PLATFORMCAM = 8,
    USE_DS4COLOR_QVGA = 16,
};

// MACRO SIMULATION info to java
// TODO: depth has to be enabled to get accurate fisheye/IMU system ts
JNIEXPORT jint JNICALL CameraConfigureType(JNIEnv *env, jobject obj) {
#if defined(APP_LOCALMAPPING_TEST)
    return USE_DEPTH | USE_FISHEYE;
#elif defined(APP_TEST)
    return USE_DEPTH | USE_FISHEYE | USE_DS4COLOR_VGA | USE_PLATFORMCAM;
#elif defined(APP_TENSORFLOW_SAMPLE)
    return USE_PLATFORMCAM;
#elif defined(APP_SOCKET)
    return USE_DEPTH | USE_FISHEYE;
#else
    return USE_DEPTH | USE_FISHEYE;
#endif
}


