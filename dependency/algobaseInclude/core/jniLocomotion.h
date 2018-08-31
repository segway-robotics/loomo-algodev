#ifndef JNI_LOCOMOTION_H_
#define JNI_LOCOMOTION_H_

#include <jni.h>
#include <android/log.h>
#include <android/bitmap.h>
#include <sys/types.h>
#include <assert.h>
#include <stdlib.h>
#include <stdint.h>
#include "string"
#include "RawDataUtil.h"
#include "RawData.h"
#include <vector>

namespace ninebot_algo {
	namespace sdk_locomotion {
#define TIME_DIFF_ERROR_THRE_MS 150
#define TIME_DIFF_CRITICAL_THRE_MS 300 // TODO: to reduce to 500 in GX EVT3

		typedef int (*odom_pose_cb_handler)(int64_t, float, float, float, float, float);

		// for base control
		ninebot_algo::StampedTwist getClosestRobotOdometry(int64_t ts);

		ninebot_algo::StampedTwist getLatestRobotOdometry();

        NINEBOT_EXPORT void setOdometryPoseCallbackHandler(odom_pose_cb_handler cb, RawData* pRawData);

		NINEBOT_EXPORT ninebot_algo::StampedTwist getClosestRobotOdometryLowLvl(int64_t ts);

		NINEBOT_EXPORT ninebot_algo::StampedTwist getLatestRobotOdometryLowLvl();

		NINEBOT_EXPORT std::string getRobotSensorStatus();

		bool getIsBaseTimestampError();

		bool getIsExternalTimestampError();

		uint64_t getBaseErrorCode();

		ninebot_algo::StampedFloat getRobotUltrasonic();

		ninebot_algo::StampedFloat getBodyPressure();

        ninebot_algo::StampedUltrasonics getRobotExternalUltrasonic();

        ninebot_algo::StampedIr getBodyIRsensor();

		ninebot_algo::StampedFloat getHeadPitchAngle();

		ninebot_algo::StampedFloat getHeadYawAngle();

		ninebot_algo::StampedFloat getHeadRollAngle();

		ninebot_algo::StampedFloat getMaincamPitchAngle();

		ninebot_algo::StampedFloat getMaincamYawAngle();

		ninebot_algo::StampedFloat getMaincamRollAngle();

		ninebot_algo::StampedBasePos getBasePos();

		ninebot_algo::StampedBasePos getBodyPos();

		ninebot_algo::StampedBaseWheelInfo getWheelSpeedInfo();

		ninebot_algo::StampedBumper getBaseBumper();

		ninebot_algo::StampedInfraredArray getBaseInfrared();

		void setMode(JNIEnv *env, int mode);

		void setPushingMode(JNIEnv *env, int mode);

		void setLinearVelocity(JNIEnv *env, float velocity);

		void setAngularVelocity(JNIEnv *env, float velocity);

		void setYawAngularVelocity(JNIEnv *env, float angle);

		void setPitchAngularVelocity(JNIEnv *env, float angle);

		void setJointYaw(JNIEnv *env, float angle);

		void setJointPitch(JNIEnv *env, float angle);

		void setSmoothFactor(JNIEnv *env, int yaw, int pitch, int roll);

		int getCartMile(JNIEnv *env) ;

		void startVLS(JNIEnv *env, bool calibrator_enable);

		void stopVLS(JNIEnv *env);
		
		ninebot_tf::tf_message getTfData(JNIEnv *env, string tgt, string src, int64_t timestamp, int time_trh_ms);
		
        bool setCustomizedTFframe(JNIEnv *env, ninebot_tf::tf_message cpp_tf);

		int getMassiveTfDataLoco(JNIEnv *env, ninebot_tf::vector_req_t *requestList,ninebot_tf::vector_tf_msg_t *returnList);

		int resetFrameData(JNIEnv *env, std::string farme_name);

		void checkFallAction(float ir1, float ir2);

		void resetLocomotion();

		NINEBOT_EXPORT int init_locomotion(JNIEnv *env);
	}
}
#endif
