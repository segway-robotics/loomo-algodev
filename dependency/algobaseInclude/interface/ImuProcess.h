#ifndef IMU_PROCESS_H_
#define IMU_PROCESS_H_

#include "RawDataUtil.h"
#include "AlgoBase.h"
#include "string"
#include "vector"
#include "iostream"
#include "fstream"
#include <stdlib.h>
#include <stdint.h>

namespace ninebot_algo
{
	class ImuOrientation;
	class IMUCallbackData;
	class NINEBOT_EXPORT IMUProcess{
	public:
		IMUProcess();
		~IMUProcess();

		static IMUProcess* getInstance();
		static void delInstance();

		void setIMUCallbackAlgoPtr(AlgoBase* algoPtr);
		void clearIMUCallbackPtr();
		void setGyroData(float* gyro, int64_t timestamp, int64_t timestampPlatform);
		void setAccelData(float* accel, int64_t timestamp, int64_t timestampPlatform);
		void startIMUrecording(std::string save_folder, int64_t save_time);
		void stopIMUrecording();
		void processIMU();
		StampedOrientation getLatestIMUOrientation();
		StampedOrientation getCloestIMUOrientation(int64_t ts);
		static IMUProcess* mInstance;

	private:
		std::vector <MTPoint> imuAccelMap;
		std::vector <MTPoint> imuGyroMap;
		std::ofstream imuRecordFile;
		std::ofstream imuRecordFileGyro, imuRecordFileAccel;
		int64_t imu_time_start_recording;
		bool imu_isRecording;
		ImuOrientation *pimu_orientation_;
		MTPoint gyro_d;
		MTPoint accel_d;
		IMUCallbackData* imuCallbackPtr;

	};
}

#endif