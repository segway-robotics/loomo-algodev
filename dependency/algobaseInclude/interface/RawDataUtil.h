/*! define basic struture for RawData.h
 * 
 * Filename: RawDataUtil.h
 * Version: 0.60
 * This file is common to many different projects, DO NOT CHANGE THIS FILE unless discussed with Zichong Chen
 * Each modification shall come with a version number update
 * Algo team, Ninebot Inc., 2016
 */

#ifndef RAWDATAUTIL_H
#define RAWDATAUTIL_H
#include <string>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <vector>
#include "tf_data_type.h"
#include "opencv2/opencv.hpp"

#if defined(_MSC_VER)
#define NINEBOT_EXPORT __declspec(dllexport)
#define NINEBOT_ALIGN
#else /* compiler doesn't support __declspec() */
#define NINEBOT_EXPORT __attribute__ ((visibility("default")))
#define NINEBOT_ALIGN __attribute__((packed))
#endif

namespace ninebot_algo
{

enum RawDataErrorCode : uint64_t {
    RAWDATA_NO_ERROR 						= 0 ,
    RAWDATA_FISHEYE_TIMESTAMP_ERROR 		= (1<<0) ,
    RAWDATA_DS4COLOR_TIMESTAMP_ERROR 		= (1<<1) ,
    RAWDATA_COLOR_TIMESTAMP_ERROR 			= (1<<2) ,
    RAWDATA_DEPTH_TIMESTAMP_ERROR 			= (1<<3) ,
    RAWDATA_EXTDEPTHL_TIMESTAMP_ERROR		= (1<<4) ,
    RAWDATA_EXTDEPTHR_TIMESTAMP_ERROR		= (1<<5) ,
    RAWDATA_TICK_TIMESTAMP_ERROR			= (1<<6) ,
    RAWDATA_ODOMETRY_TIMESTAMP_ERROR		= (1<<7) ,
    RAWDATA_BUMPER_TIMESTAMP_ERROR			= (1<<8) ,
    RAWDATA_ULTRASONIC_TIMESTAMP_ERROR		= (1<<9) ,
    RAWDATA_BODYINFRARED_TIMESTAMP_ERROR	= (1<<10) ,
    RAWDATA_FRONTINFRARED_TIMESTAMP_ERROR	= (1<<11) ,
    RAWDATA_LEFTINFRARED_TIMESTAMP_ERROR	= (1<<12) ,
    RAWDATA_REARINFRARED_TIMESTAMP_ERROR	= (1<<13) ,
    RAWDATA_RIGHTINFRARED_TIMESTAMP_ERROR	= (1<<14) ,
    RAWDATA_BASEPOSE_TIMESTAMP_ERROR		= (1<<15) ,
    RAWDATA_WHEELSPEED_TIMESTAMP_ERROR		= (1<<16) ,
    RAWDATA_BODYPOSE_TIMESTAMP_ERROR		= (1<<17) ,
    RAWDATA_JOINTYAW_TIMESTAMP_ERROR		= (1<<18) ,
};

struct RawCameraIntrinsics {
    float focalLengthX;
    float focalLengthY;
    float pricipalPointX;
    float pricipalPointY;
    float distortion[5];
};

struct RawCameraintrinsics2 {
    float kf[3][3];
    float distortion[5];
};

struct SensorIntrinsics {
    float x_y;
    float x_z;
    float y_x;
    float y_z;
    float z_x;
    float z_y;
    float bias_x;
    float bias_y;
    float bias_z;
    float scale_x;
    float scale_y;
    float scale_z;
    float biasVar[3];
    float varNoise[3];
};

struct RawCameraExtrinsics {
    float x;
    float y;
    float z;
    float rotation[3][3];
};

struct MotionModuleCalibration {
    RawCameraintrinsics2 fishEyeIntrinsics;
    SensorIntrinsics accelIntrinsics;
    SensorIntrinsics gyroIntrinsics;
    RawCameraExtrinsics fishEyeToDepth;
    RawCameraExtrinsics depthToImu;
    RawCameraExtrinsics fishEyeToImu;
    RawCameraExtrinsics colorToImu;
};

struct CalibrationInfoExtDepth {
    RawCameraIntrinsics left;
    RawCameraIntrinsics right;
};

struct CalibrationInfoDS4T {
    MotionModuleCalibration motion;
    RawCameraIntrinsics color;
    RawCameraIntrinsics depth;
    RawCameraExtrinsics depth2color;
};

template <typename T>
std::string ToString(T val, int prec = 2)
{
	std::stringstream stream;
	stream << std::fixed << std::setprecision(prec) << val;
	return stream.str();
}

typedef struct DetectionStruct
{
	int		x;
	int		y;
	int		width;
	int		height;
	int64_t	timestamp;
}DetectData;

struct NINEBOT_ALIGN lidar_data {
    uint16_t index;
    double speed;
    uint16_t distance;
    bool is_distance_invalid;
    bool is_strength_warning;
    uint16_t strength;
};

struct NINEBOT_EXPORT lidar_data_t {
    uint16_t dimension; // 360
    lidar_data data[360];
};

struct NINEBOT_EXPORT StampedLidar
{
public:
	lidar_data_t lidar;
	int64_t timestamp;
	StampedLidar() :lidar(), timestamp(-1) {}
	StampedLidar(lidar_data_t p, int64_t time_stamp)
	{
		lidar = p; timestamp = time_stamp;
	}
};

struct  NINEBOT_EXPORT Pose
{
public:
	double x;
	double y;
	double orientation;
public:
	Pose():x(0),y(0),orientation(0){}
	Pose(float x_,float y_,float orientation_)
	{
		x= x_;y=y_;orientation=orientation_;
	}
};
struct  NINEBOT_EXPORT Velocity
{
public:
	float linear_velocity;
	float angular_velocity;
public:
	Velocity():linear_velocity(0),angular_velocity(0){}
	Velocity(float v,float w)
	{
		linear_velocity=v;
		angular_velocity=w;
	}
};

struct  NINEBOT_EXPORT Twist
{
public:
	Pose pose;
	Velocity velocity;
	int left_ticks,right_ticks;
public:
	Twist():pose(0.0f,0.0f,0.0f),velocity(0.0f,0.0f){}
	Twist(Pose p,Velocity v){
		pose = p; velocity =v;
	}
};

struct  NINEBOT_EXPORT UwbData
{
public:
	int id;
	float direction;
	float distance;
	float quality;
public:
	UwbData():id(0),direction(0.0f),distance(0.0f),quality(0.0f){}
};

struct NINEBOT_EXPORT StampedUltrasonics
{
public:
	float leftFront;
	float leftRear;
	float rear;
	float rightRear;
	float rightFront;
	int64_t timestampLeftFront;
	int64_t timestampLeftRear;
	int64_t timestampRear;
	int64_t timestampRightRear;
	int64_t timestampRightFront;
    StampedUltrasonics():leftFront(0.0f),leftRear(0.0f),rear(0.0f),rightRear(0.0f),rightFront(0.0f),
					 timestampLeftFront(-1), timestampLeftRear(-1), timestampRear(-1), timestampRightRear(-1), timestampRightFront(-1){}
	StampedUltrasonics(float ultra_leftFront, float ultra_leftRear,
		float ultra_rear, float ultra_rightRear, float ultra_rightFront,
		int64_t time_stampLeftFront, int64_t time_stampLeftRear,
		int64_t time_stampRear, int64_t time_stampRightRear, int64_t time_stampRightFront)
	{
		leftFront = ultra_leftFront;
		leftRear = ultra_leftRear;
		rear = ultra_rear;
		rightRear = ultra_rightRear;
		rightFront = ultra_rightFront;
		timestampLeftFront = time_stampLeftFront;
		timestampLeftRear = time_stampLeftRear;
		timestampRear = time_stampRear;
		timestampRightRear = time_stampRightRear;
		timestampRightFront = time_stampRightFront;
	}
};
	
struct NINEBOT_EXPORT Orientation
{
public:
	float yaw;
	float pitch;
	float roll;
	Orientation():yaw(0),pitch(0),roll(0){}
	Orientation(float y,float p,float r)
	{
		yaw=y;pitch=p;roll=r;
	}
};

struct NINEBOT_EXPORT MTPoint
{
	float x, y, z;
	int64_t tHw;
	int64_t ts;

};

struct NINEBOT_EXPORT StampedIMU
{
public:
	float ax,ay,az;
	float gx,gy,gz;
	int64_t tHw;
	int64_t ts;
	StampedIMU():ax(0),ay(0),az(0),gx(0),gy(0),gz(0),tHw(0),ts(0){}
};

struct NINEBOT_EXPORT StampedMat
{
public:
	cv::Mat image;
	int64_t exposure;
	int64_t timestampHw;
	int64_t timestampSys;
	StampedMat():image(),timestampHw(-1),timestampSys(-1){}
	StampedMat(cv::Mat img, int64_t time_stamp)
	{
		image = img.clone(); timestampSys = time_stamp;
		timestampHw = 0; exposure = 0;
	}
};

struct NINEBOT_EXPORT StampedPose
{
public:
	Pose pose;
	int64_t timestamp;
	StampedPose():pose(),timestamp(-1){}
	StampedPose(Pose p, int64_t time_stamp)
	{
		pose = p; timestamp = time_stamp;
	}
};

struct NINEBOT_EXPORT StampedTwist
{
public:
	Twist twist;
	int64_t timestamp;
	StampedTwist():twist(),timestamp(-1){}
	StampedTwist(Twist tw, int64_t time_stamp)
	{
		twist = tw; timestamp = time_stamp;
	}
};

struct NINEBOT_EXPORT StampedUwbData
{
public:
	UwbData uwb;
	int64_t timestamp;
	StampedUwbData():uwb(),timestamp(-1){}
};

struct NINEBOT_EXPORT StampedFloat
{
public:
	float value;
	int64_t timestamp;
	StampedFloat():value(0),timestamp(-1){}
	StampedFloat(float val, int64_t time_stamp)
	{
		value = val; timestamp = time_stamp;
	}
};

struct NINEBOT_EXPORT StampedOrientation
{
public:
	Orientation orientation;
	int64_t timestamp;
	StampedOrientation():orientation(0,0,0),timestamp(-1){}
	StampedOrientation(Orientation ori,int64_t time_stamp)
	{
		orientation = ori;timestamp=time_stamp;
	}
};

struct NINEBOT_EXPORT StampedCameraPose
{
public:
	std::vector<float> pose;
	int64_t timestamp;
	StampedCameraPose(): timestamp(-1){ pose.resize(12,0.0f);}
};

struct NINEBOT_EXPORT CameraIntrinsics
{
public:
	float fx;
	float fy;
	float cx;
	float cy;
	CameraIntrinsics(): fx(0.0f),fy(0.0f),cx(0.0f),cy(0.0f){}
	CameraIntrinsics(float f_x,float f_y,float c_x,float c_y)
	{
		fx = f_x; fy = f_y; cx = c_x; cy = c_y;
	}
};

struct NINEBOT_EXPORT Cmd
{
	public:
		float linearv;
		float angularw;
	 
		Cmd():linearv(0),angularw(0) {}
		Cmd (float linear_v,float angular_w)
		{
			 
			 linearv = linear_v;
			 angularw=angular_w;
		}
};
struct NINEBOT_EXPORT StampedCmd
{
	public:
		int64_t timestamp;
		Cmd cmd;
		StampedCmd():timestamp(-1),cmd(0,0){}
		StampedCmd(Cmd command,int64_t time_stamp)
		{
			 cmd =command;	
			 timestamp=time_stamp;
		}
};
struct NINEBOT_EXPORT StampedIr
{
	public:
		float leftir;
		float rightir;
		int64_t timestamp;
		StampedIr():leftir(0),rightir(0),timestamp(-1){}
		StampedIr(int64_t time_stamp,float left_ir,float right_ir)
		{
			 timestamp=time_stamp;
			 leftir = left_ir;
			 rightir= right_ir;
		}
};

struct NINEBOT_EXPORT StampedBumper
{
	public:
		bool trigger[6];
		int64_t timestamp;
		StampedBumper():timestamp(-1){ trigger[0]=trigger[1]=trigger[2]=trigger[3]=trigger[4]=trigger[5]=false;}
};

struct NINEBOT_EXPORT StampedInfraredArray
{
	public:
		float ir[12];
		int64_t timestamp[12];
		StampedInfraredArray(){ for(int i=0;i<12;i++) ir[i]=0.0f, timestamp[i]=-1;}
};

struct NINEBOT_EXPORT StampedHeadPos
{
	public:
		float yaw;
		float pitch;
		float roll;
		int64_t timestampYaw;
		int64_t timestampPitch;
		int64_t timestampRoll;
		StampedHeadPos():yaw(0),pitch(0),roll(0),timestampYaw(-1),timestampPitch(-1),timestampRoll(-1){}
};

struct NINEBOT_EXPORT StampedBaseWheelInfo
{
public:
	float leftSpeed;
	float rightSpeed;
	float leftCurrent;
	float rightCurrent;
	int64_t timestamp;
	StampedBaseWheelInfo():
			leftSpeed(0),rightSpeed(0),
			leftCurrent(0),rightCurrent(0),
			timestamp(-1){}
};

struct NINEBOT_EXPORT StampedVelocity
{
public:
	Velocity vel;
	int64_t timestamp;
	StampedVelocity():vel(),timestamp(-1){}
	StampedVelocity(Velocity v, int64_t time_stamp)
	{
		vel = v; timestamp = time_stamp;
	}
};

struct NINEBOT_EXPORT StampedBasePos
{
	public:
		float yaw;
		float pitch;
		float roll;
		int64_t timestamp;
		StampedBasePos():yaw(0),pitch(0),roll(0),timestamp(-1){}
		StampedBasePos(int64_t time_stamp,float yaw_,float pitch_,float roll_)
		{
			 timestamp=time_stamp;
			 yaw = yaw_;
			 pitch = pitch_;
			 roll = roll_;
		}
};

struct NINEBOT_EXPORT StampedImage
{
	public:
		std::string imagename;
		int64_t timestampHw;
		int64_t timestampSys;
		int64_t exposure;
	StampedImage():imagename(),timestampHw(-1),timestampSys(-1){}
};

class Timer
{
public:
	Timer():m_begin(std::chrono::high_resolution_clock::now()){}
	void tic() {m_begin = std::chrono::high_resolution_clock::now();}
	int64_t toc()const
	{
		return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - m_begin).count();
	}
private:
	std::chrono::time_point<std::chrono::high_resolution_clock> m_begin;
};

} // namespace ninebot_algo

#endif
