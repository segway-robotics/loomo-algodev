#ifndef JNI_VISION_H_
#define JNI_VISION_H_

#include <jni.h>
#include <android/log.h>
#include <android/bitmap.h>
#include <sys/types.h>
#include <assert.h>
#include <stdlib.h>
#include <stdint.h>
#include "string"
#include "RawDataUtil.h"


namespace ninebot_algo {
	namespace sdk_vision {

		struct FrameInfo {
			bool success;
			int stride;
			int frameNum;
			int exposure;
			int64_t timeStamp;
			int64_t platformTimestamp;
			int resolution;
		};

		enum MotionServiceType {
			MOTION_SERVICE_NONE = 0,
			MOTION_SERVICE_GYRO = 0x0001,
			MOTION_SERVICE_ACCEL = 0x0002,
			MOTION_SERVICE_FISHEYE = 0x0004,
			MOTION_SERVICE_DEPTH = 0x0008,
		};

		enum MotionSensorType {
			MOTION_SENSOR_GYRO = 1,
			MOTION_SENSOR_ACCEL,
			MOTION_SENSOR_MAGNOMETER,
		};

		struct SensorFrame {
			uint64_t timestamp;
			MotionSensorType type;
			float x;
			float y;
			float z;
		};

		struct MotionFrameHeader {
			uint64_t timestamp;
			uint64_t seq;
			MotionSensorType type;
		};

		struct MotionSensorFrame {
			struct MotionFrameHeader header;
			float x;
			float y;
			float z;
			long platformTimestamp;
		};

		struct PlatformTimestampedMotionSensorFrame {
			struct MotionFrameHeader header;
			float x;
			float y;
			float z;
			long platformTimestamp;
		};

		int64_t getTimeStamp(JNIEnv *env);

		int64_t getTimeStampSysRSdelta(JNIEnv *env);

		float getIMUTemperatureDevice(JNIEnv *env);

		ninebot_algo::CalibrationInfoDS4T getCalibrationInfoDS4T(JNIEnv *env);

		ninebot_algo::CalibrationInfoExtDepth getCalibrationInfoExtDepth(JNIEnv *env);

		FrameInfo getFrameInfo(JNIEnv *env, jobject frame);

		void *getBuffer(JNIEnv *env, jobject frame);

		jobject getLatestColorFrame(JNIEnv *env);

		jobject getLatestDepthFrame(JNIEnv *env);

		jobject getLatestFishEyeFrame(JNIEnv *env);

		jobject getLatestExtDepthFrame(JNIEnv *env, jint id);

		jobject getLatestPlatformFrame(JNIEnv *env);

		void returnColorFrame(JNIEnv *env, jobject frame);

		void returnDepthFrame(JNIEnv *env, jobject frame);

		void returnFishEyeFrame(JNIEnv *env, jobject frame);

		void returnExtDepthFrame(JNIEnv *env, jobject frame);

		void returnPlatformFrame(JNIEnv *env, jobject frame);

		float getFishEyeManualExposure(JNIEnv *env);

		void setPlatformAutoExposureROI(JNIEnv *env, int left, int top, int right, int bottom);

		void setFishEyeAutoExposureROI(JNIEnv *env, int left, int top, int right, int bottom);

		void setFishEyeEvCompensationJvm(JNIEnv *env, float compensation);

		void enableFishEyeAutoExposureJvm(JNIEnv *env, bool enable);

		void setFishEyeManualExposureJvm(JNIEnv *env, float exposure);

		void setFishEyeManualGainJvm(JNIEnv *env, float gain);

		JNIEnv *attach();

		void detach();

		void setPersonRect(JNIEnv *env, std::vector <ninebot_algo::DetectData> d_data);

		NINEBOT_EXPORT int init_vision(JNIEnv *env);

	}
}

#endif
