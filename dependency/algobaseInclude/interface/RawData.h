/*! define the base class for interfacing with hardware input/output
 * need to implement class ProxyDevice for live streaming, or class ProxySim for offline simulation
 * 
 * Filename: RawData.h
 * Version: 0.80
 * This file is common to many different projects, DO NOT CHANGE THIS FILE unless discussed with Zichong Chen
 * Each modification shall come with a version number update
 * Algo team, Ninebot Inc., 2016
 */

#ifndef RAWDATA_H_
#define RAWDATA_H_

#include <chrono>
#include <thread>
#include <mutex>
#include <fstream>
#include <atomic>
#include <queue>
#include "RawDataUtil.h"
#include "tf_data_type.h"
#ifndef WIN32
#include <jni.h>
#endif
#include "opencv2/opencv.hpp"

//#define SIMULATION

using namespace std;

namespace ninebot_algo
{
	enum ROBOT_ERROR_CODE
	{
		ROBOTCODE_NO_ERROR = 0,
		ROBOTCODE_BASE_TIMESTAMP_ERROR = 1,
		ROBOTCODE_CAMERAS_TIMESTAMP_ERROR = 2,
		ROBOTCODE_EXTERNALULTRASONIC_TIMESTAMP_ERROR = 4,

	};

	enum ROBOT_MODEL
	{
		MODEL_LOOMO_G1 = 0,
		MODEL_LOOMO_GX_A = 1,
		MODEL_LOOMO_GX_B = 2,
	};

class ProxyDevice;
class ProxySim;
class AlgoBase;

class NINEBOT_EXPORT RawData

{
public:
	RawData() ;
	~RawData();

	/*! Retrieve robot model, defined as enum ROBOT_MODEL*/
	static ROBOT_MODEL retrieveRobotModel();
	/*! Retrieve robot serial number */
    static string retrieveRobotSerialNumber();
	/*! Start streaming sensors in LIVE or SIMULATION mode, static function shall be used in main() */
	static void StartSensors(bool isSim = false, const string files_path = "");
	/*! Stop hardware sensors, static function shall be used in main() */
	static void StopSensors();

	/*! Retrieve fisheye image, the timestamped result is stored in fisheye. If isRecordDuplicate=true, then duplicate images are stored, otherwise not */
	virtual void retrieveFisheye(StampedMat &fisheye, bool isRecordDuplicate = true) ;
	/*! Retrieve fisheye exposure time (shutter time in millisecond) */
	virtual float retrieveFisheyeShutter();
	/*! Retrieve IMU temperature */
	virtual float retrieveIMUTemperature();
	/*! Retrieve depth image, the timestamped result is stored in depth */
	virtual void retrieveDepth(StampedMat &depth, bool isRecordDuplicate = true);
	/*! Retrieve color image, the timestamped result is stored in color */
	virtual void retrieveColor(StampedMat &color, bool isRecordDuplicate = true);
	/*! Retrieve DS4 color image, the timestamped result is stored in color */
	virtual void retrieveDS4Color(StampedMat &color, bool isRecordDuplicate = true);
	/*! Retrieve DS5 left depth image, the timestamped result is stored in ds5 */
	virtual void retrieveDS5Left(StampedMat &ds5, bool isRecordDuplicate = true);
	/*! Retrieve DS5 right depth image, the timestamped result is stored in ds5 */
    virtual void retrieveDS5Right(StampedMat &ds5, bool isRecordDuplicate = true);
	/*! Retrieve robot detail error code, defined as bits, see enum RawDataErrorCode*/
	virtual uint64_t retrieveDetailDeviceErrorCode();
	/*! Retrieve robot error code, defined as bits, see enum ROBOT_ERROR_CODE*/
	virtual int retrieveDeviceErrorCode();
	/*! Retrieve ultrasonic signal, the timestamped result is stored in ultrasonic, unit: mm */
	virtual void retrieveUltrasonic(StampedFloat &ultrasonic);
	/*! Retrieve body pressure sensor value */
	virtual void retrievePressure(StampedFloat &pressure);
	/*! Retrieve external ultrasonics signal, the timestamped result is stored in ultrasonics, unit: mm */
	virtual void retrieveExternalUltrasonics(StampedUltrasonics &ultrasonics);
	/*! Retrieve IR range sensor signal, the timestamped result is stored in ir_sensors, unit: mm */
	virtual void retrieveIRsensor(StampedIr &ir_sensors);
	/*! Retrieve odometry signal that is closest to timestamp ts, the timestamped result is stored in odom
	 * If ts<0, return the most recent values */
	virtual void retrieveOdometry(StampedTwist &odom, int64_t ts);
	/*! Retrieve gravity orientation (in Eular angles) that is closest to timestamp ts, the timestamped result is stored in imu
	 * If ts<0, return the most recent values */
	virtual void retrieveGOrientation(StampedOrientation &imu, int64_t ts);
	/*! Get depth camera calibration data, the result is stored in camerapara */
	virtual void getMaincamParam(RawCameraIntrinsics &camerapara);
	/*! Get DS4T camera calibration data, the result is stored in calib */
	virtual void getCalibrationDS4T(CalibrationInfoDS4T &calib);
	/*! Get ExtDepth camera calibration data, the result is stored in calib */
	virtual void getCalibrationExtDepth(CalibrationInfoExtDepth &calib);
	/*! Print DS4T camera calibration data to a readable string */
	virtual std::string printDS4TCalibration(CalibrationInfoDS4T& calib);
	/*! Retrieve head pose, yaw, pitch, and roll */
	virtual void retrieveHeadPos(StampedHeadPos &headpos);
	/*! Retrieve base wheel info, left and right wheel current and speed (m/s) */
	virtual void retrieveBaseWheelInfo(StampedBaseWheelInfo &wheel_info);
	/*! Retrieve base velocity (v & w) */
	virtual void retrieveBaseVelocity(StampedVelocity &velocity);
	/*! Retrieve maincam pose, yaw, pitch, and roll */
	virtual void retrieveMaincamPos(StampedHeadPos &maincampos);
	/*! Retrieve base pose, yaw, pitch, and roll */
    virtual void retrieveBasePos(StampedBasePos &basepos);
    /*! Retrieve body pose, yaw, pitch, and roll */
    virtual void retrieveBodyPos(StampedBasePos &basepos);
    /*! Retrieve base bumper sensors */
    virtual void retrieveBaseBumper(StampedBumper &bumper);
   /*! Retrieve base infrared sensors */
    virtual void retrieveBaseInfrared(StampedInfraredArray &infrared);
	/*! Retrieve uwb data */
	virtual void retrieveUwbData(StampedUwbData &uwb);
	/*! get current Realsense timestamp in us */
    virtual int64_t getCurrentTimestampRS();
	/*! get current Android system timestamp in us */
	virtual int64_t getCurrentTimestampSys();
	/*! get current Android system to Realsense timestamp delta in us */
	virtual int64_t getCurrentTimestampSysRSdelta();
	/*! get miles recorded by the encoder  */
	virtual int getCartMile();

	/*!< Below functions are used in LIVE mode */
	/*! Start recording when live streaming, it should be called only in one RawData instance
	 * @param save_folder The folder that data shall be recorded
	 * @param save_time The current Realsense timestamp as the starting time, so recorded timestamp begins with 0
	 * @param isSaveJpg To save fisheye as JPG(compressed) or PGM
	*/
	virtual void StartRecording(std::string save_folder, int64_t save_time, bool isSaveJpg = true);
	/*! Stop recording when live streaming, it should be called only in one RawData instance */
	virtual void StopRecording();
	/*! Send movement command to the base, it should be called only in one RawData instance
	 * @param v Translational velocity command
	 * @param w Rotational velocity command
	 * @param ts Current timestamp
	 * unit: m/s, rad/s
	*/
	virtual void ExecuteCmd(float v,float w, int64_t current_ts = 0);
	/*! Send speed movement command to the head, it should be called only in one RawData instance
     * @param yaw Yaw velocity command
     * @param pitch Pitch velocity command
     * @param en_sel 0: both yaw and pitch, 1: only yaw, 2: only pitch
     * unit: rad/s
    */
	virtual void ExecuteHeadSpeed(float yaw, float pitch, int en_sel);
	/*! Send position movement command to the head, it should be called only in one RawData instance
     * @param yaw_position Yaw position command
     * @param pitch_position Pitch position command
     * @param en_sel 0: both yaw and pitch, 1: only yaw, 2: only pitch
     * unit: rad
    */
	virtual void ExecuteHeadPos(float yaw_position, float pitch_position, int en_sel);
    /*! Send mode command to the head, it should be called only in one RawData instance
     * @param mode Head mode: 0 - position mode, 1 - velocity mode, 2 - pitch position mode, yaw velocity mode, 3 - yaw position mode, pitch velocity mode
    */
	virtual void ExecuteHeadMode(int mode);
    /*! Send smooth factor command to the head, it should be called only in one RawData instance
     * @param yaw Yaw factor
     * @param pitch Pitch factor
     * @param roll Roll factor
     * unit: 0~100
    */
	virtual void ExecuteHeadSmoothFactor(int yaw, int pitch, int roll);
	/*! Send platform auto exposure ROI to the camera */
	virtual void ExecutePlatformAutoExposureROI(int left, int top, int right, int bottom);
	/*! Send fisheye auto exposure ROI to the camera */
	virtual void ExecuteFisheyeAutoExposureROI(int left, int top, int right, int bottom);
	/*! Send fisheye configuration to the camera */
	virtual void ExecuteFishEyeEvCompensation(float compensation);
	virtual void ExecuteFishEyeAutoExposureEnable(bool enable);
	virtual void ExecuteFishEyeManualExposure(float exposure);
	virtual void ExecuteFishEyeManualGain(float gain);
	/*! Set person detect Rect [LEGACY]
 	* @param data vector of DetectData
	*/
	void SetRect(std::vector<DetectData> data);
	/* IMU callback registration */
    void registerIMUCallback(AlgoBase* ptr);
	void deregisterIMUCallback();
	// used for wheel odometry calibration in live
	void setEncoderCountToRot(const double count_to_left_rot, const double count_to_right_rot);

#ifndef WIN32
	/*! Attach and detach of JVM, necessary for Android APP, due to native and Java linkage
	 * Do not need explicit use in most case, already embedded in AlgoBase::run() */
	void attachJVM();
	void detachJVM();
	/*! Get JVM env according to current thread id, useful to access some Java native functions */
    void registerJVMEnv(JNIEnv* env);
	JNIEnv* getEnv(int tid);
#endif
	/*!< Below functions are used in SIMULATION mode */
	/*! Start simulation by loading saved files, static function shall be used in main() only once
	 * @param files_path The folder that simulation data shall be loaded
	*/
	/*! Get current and total simulation tick (counter in milliseconds) */
	void getSimTicks(int64_t &current_ticks,int64_t &total_ticks,int64_t &start_tick);
	/*! Set current simulation tick (counter in milliseconds) */
	void setSimCurrentTicks(const int64_t current_ticks);
	/*! retrieve recorded base movement command, the result is stored in cmd */
	virtual void retrieveCmd(StampedCmd &cmd);

	/*! this is lookup for the transform about the tgt frame in the src frame
	 * @param src source frame
	 * @param tgt target frame 
	 * @timestamp timestamp that looking for , and -1 means find the latest one
	 * @time_trh_ms means the treshold that can be accecpt , unit is millsecond.
	*/ 
	ninebot_tf::tf_message GetTfBewteenFrames(std::string tgt, std::string src, int64_t timestamp, int time_trh_ms);

	/*! create a customized TF frame if not exist, and feed in observations
     * @param tf tfmsg includes src and tgt frame id, ts, and transformation
    */
	bool setCustomizedTf(ninebot_tf::tf_message tf);
    bool setCustomizedTf(string tgt, string src, int64_t ts, float qx, float qy, float qz, float qw, float tx, float ty, float tz);
    int getMassiveTfData(ninebot_tf::vector_req_t *requestList,ninebot_tf::vector_tf_msg_t *returnList);
	int resetTfFrameData(std::string frame_name);
    // VLS related
    void startVLS(bool calibrator_enable);
    void stopVLS();

private:
	static ROBOT_MODEL getRobotModel();
	RawData(RawData const& rhs);             // prevent copying
	RawData& operator=(RawData const& rhs);  // prevent assignment

	int64_t colords4_t, depth_t, fisheye_t, color_t, ds5l_t, ds5r_t;
	StampedOrientation imu_orientation_;
	StampedCmd cmd_;
	static std::mutex mutexRawImage_;
	static std::mutex mutexSensors_;
	static int rawdata_cnt;
	int rawdata_id;

	static ProxySim *p_proxysim_;
	static ProxyDevice *p_proxy_;
	static int mIsSim;
	static int mIsRecording;
	static float base_width;
	static ROBOT_MODEL robot_model_;

	//the below is for record function
	int64_t time_start_recording_ ;

	std::ofstream fisheye_record_file_;
	std::ofstream color_record_file_;
	std::ofstream colords4_record_file_;
	std::ofstream depth_record_file_;
	std::ofstream ds5l_record_file_;
	std::ofstream ds5r_record_file_;
	std::ofstream imu_record_file_;
	std::ofstream imu_temp_record_file_;
	std::ofstream uwb_record_file_;
	std::ofstream odom_record_file_;
	std::ofstream ultrasonic_record_file_;
	std::ofstream external_ultrasonics_record_file_;
	std::ofstream ir_record_file_;
	std::ofstream headpose_record_file_;
	std::ofstream maincampose_record_file_;
	std::ofstream basepose_record_file_;
	std::ofstream bodypose_record_file_;
	std::ofstream bumper_record_file_;
	std::ofstream infraredarray_record_file_;
	std::ofstream cmd_file_;
	std::ofstream imgstat_file_;

	std::string save_folder_;
	std::string fisheye_name_ ;
	std::string depth_name_ ;
	std::string color_name_ ;
	std::string colords4_name_ ;
	std::string ds5l_name_ ;
	std::string ds5r_name_ ;
	std::string imu_name_  ;
	std::string imu_temp_name_  ;
	std::string uwb_name_  ;
	std::string odom_name_ ;
	std::string ultrasonic_name_  ;
	std::string external_ultrasonics_name_  ;
	std::string ir_name_  ;
	std::string headpose_name_  ;
	std::string maincampose_name_  ;
	std::string basepose_name_  ;
	std::string bodypose_name_  ;
	std::string bumper_name_  ;
	std::string infraredarray_name_  ;
	std::string cmd_name_;
	std::string imgstat_name_;

	void img_recorder();
	std::atomic<bool> is_recording_ ;
	thread mRecordThread;
	std::mutex mMutexRecorder;
	bool m_save_fisheye_jpg;
	std::queue<std::pair<std::string, cv::Mat>> fifo_fisheye;
	std::queue<std::pair<std::string, cv::Mat>> fifo_depth;
	std::queue<std::pair<std::string, cv::Mat>> fifo_ds4color;
	std::queue<std::pair<std::string, cv::Mat>> fifo_color;
	std::queue<std::pair<std::string, cv::Mat>> fifo_ds5l;
	std::queue<std::pair<std::string, cv::Mat>> fifo_ds5r;



	int fisheye_count_;
	int color_count_;
	int colords4_count_;
	int depth_count_;
	int ds5l_count_;
	int ds5r_count_;
	int fisheye_loss_count_;
    int color_loss_count_;
    int colords4_loss_count_;
    int depth_loss_count_;
    int ds5l_loss_count_;
    int ds5r_loss_count_;
};

} // namespace ninebot_algo
#endif
