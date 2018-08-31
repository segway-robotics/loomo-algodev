/*! define basic utility for AlgoUtils.h
 *
 * Filename: AlgoUtils.h
 * Version: 0.30
 * Algo team, Ninebot Inc., 2017
 */

#ifndef _ALGO_UTILS_H
#define _ALGO_UTILS_H

#define _USE_MATH_DEFINES

#include <math.h>
#include <string>
#include <fstream>
#include "Eigen/Geometry"
#include "RawDataUtil.h"
#include "opencv2/opencv.hpp"

namespace ninebot_algo {
	// unit is meter
	static constexpr float  NECK_WIDTH = 0.01346;
	static constexpr float  NECK_HEIGHT = 0.49369;
	static constexpr float  REALSENSE_WIDTH = 0.04288;
	static constexpr float  REALSENSE_HEIGHT = 0.03542;
	static constexpr float  FISHEYE_LENGTH = 0.04610;
	static constexpr float  DEPTH_LENGTH = 0.01649;
	static constexpr float  DS4COLOR_LENGTH = -0.04195;
	static constexpr float  ULTRASONIC_WIDTH = 0.08;
	static constexpr float  ULTRASONIC_HEIGHT = 0.45106;
	static constexpr float  PLATFORM_CENTER_HEIGHT = 0.10442;
	static constexpr float  PLATFORM_COLOR_WIDTH = 0.03060;
	static constexpr float  PLATFORM_COLOR_HEIGHT = 0.01822;
	static constexpr float  PLATFORM_COLOR_LENGTH = 0.05434;

	enum RealsenseSensorType
	{
		Fisheye = 1,
		DS4Color,
		Depth
	};

	// TF related conversion
	StampedPose NINEBOT_EXPORT tfmsgTo2DPose(const ninebot_tf::tf_message & tf_msg);
	Eigen::Isometry3f NINEBOT_EXPORT tfmsgToPose(const ninebot_tf::tf_message & tf_msg);
	Eigen::Isometry3d NINEBOT_EXPORT tfmsgToIsometry3d(const ninebot_tf::tf_message& tf_msg);
	ninebot_tf::tf_message NINEBOT_EXPORT packageTfData(std::string tgt, std::string src, int64_t ts, float qx, float qy, float qz, float qw, float tx, float ty, float tz);
	Eigen::Isometry3f NINEBOT_EXPORT PoseToCamcoor(const Eigen::Isometry3f & pose_odom);

	// geometry conversion
	Eigen::Vector3f NINEBOT_EXPORT quatToEuler(const Eigen::Quaternionf & quat);
	Eigen::Vector3d NINEBOT_EXPORT quatToEuler(const Eigen::Quaterniond & quat);
	Eigen::Quaternionf getQuatCamOdo();
	Eigen::Quaternionf getQuatOdoCam();
	Eigen::Isometry3f NINEBOT_EXPORT getBasePose(const StampedTwist raw_odometry, const StampedBasePos raw_basepos);
	Eigen::Isometry3f getUltrasonicPose(const StampedTwist raw_odometry, const StampedBasePos raw_basepos);
	Eigen::Isometry3f NINEBOT_EXPORT getNeckPose(const StampedTwist raw_odometry, const StampedBasePos raw_basepos);
	/* Get robot Pose from realsense camera, both on odometry coordinate */
	Eigen::Isometry3f getBasePose(const Eigen::Isometry3f realsense_pose, const StampedHeadPos raw_headpos,
								  RealsenseSensorType realsense_sensor);
	/* Get robot base pose on odometry coordinate from realsense camera with camera coordinate */
	Eigen::Isometry3f getBasePoseFromCamcoor(const Eigen::Isometry3f realsense_pose, const StampedHeadPos raw_headpos,
											 RealsenseSensorType realsense_sensor);
	/* Get robot base pose from realsense camera, both on camera coordinate */
	Eigen::Isometry3f getBasePoseFromCamcoorToCamcoor(const Eigen::Isometry3f realsense_pose, const StampedHeadPos raw_headpos,
													  RealsenseSensorType realsense_sensor);
	/* Get realsense camera pose on odometry coordinate from robot raw data*/
	Eigen::Isometry3f NINEBOT_EXPORT getRsPose(const StampedTwist raw_odometry, const StampedHeadPos raw_headpos,
								const StampedBasePos raw_basepos, RealsenseSensorType realsense_sensor);
	/* Get realsense camera pose on realsense coordinate from robot raw data*/
	Eigen::Isometry3f NINEBOT_EXPORT getRsPoseToCamcoor(const StampedTwist raw_odometry, const StampedHeadPos raw_headpos,
										 const StampedBasePos raw_basepos, RealsenseSensorType realsense_sensor);
	/* Get platform center pose on odometry coordinate from robot raw data*/
	Eigen::Isometry3f NINEBOT_EXPORT getPlatformCenterPose(const StampedTwist raw_odometry, const StampedHeadPos raw_headpos,
								const StampedBasePos raw_basepos);
	/* Get platform center pose on realsense coordinate from robot raw data*/
	Eigen::Isometry3f getPlatformCenterPoseToCamcoor(const StampedTwist raw_odometry, const StampedHeadPos raw_headpos,
										 const StampedBasePos raw_basepos);
	/* Get platform color camera pose on odometry coordinate from robot raw data*/
	Eigen::Isometry3f NINEBOT_EXPORT getPlatformColorPose(const StampedTwist raw_odometry, const StampedHeadPos raw_headpos,
								const StampedBasePos raw_basepos);
	/* Get platform color camera pose on realsense coordinate from robot raw data*/
	Eigen::Isometry3f getPlatformColorPoseToCamcoor(const StampedTwist raw_odometry, const StampedHeadPos raw_headpos,
										 const StampedBasePos raw_basepos);

	/** Modifies the given angle to translate it into the [0,2pi] range.
     * \note Take care of not instancing this template for integer numbers, since it only works for float, double and long double.
     */
	template <class T>
	inline void wrapTo2PiInPlace(T &a) {
		 bool was_neg = a<0;
		 a = fmod(a, static_cast<T>(2.0*M_PI) );
		if (was_neg) a+=static_cast<T>(2.0*M_PI);
	}

	/** Modifies the given angle to translate it into the [0,2pi] range.
	 * \note Take care of not instancing this template for integer numbers, since it only works for float, double and long double.
	 */
	template <class T>
	inline T wrapTo2Pi(T a) {
		 wrapTo2PiInPlace(a);
		 return a;
	}

	/** Modifies the given angle to translate it into the [-pi,pi] range.
	 * \note Take care of not instancing this template for integer numbers, since it only works for float, double and long double.
	 */
	template <class T>
	inline T wrapToPi(T a) {
		 return wrapTo2Pi( a + static_cast<T>(M_PI) )-static_cast<T>(M_PI);
	}

	/** Modifies the given angle to translate it into the ]-pi,pi] range.
	 * \note Take care of not instancing this template for integer numbers, since it only works for float, double and long double.
	 */
	template <class T>
	inline void wrapToPiInPlace(T &a) {
		 a = wrapToPi(a);
	}

	// get closet data from a increase deque on timestamp.
	template<typename T>
	T getClosest(const int64_t& time_stamp, const std::deque<T>& buffer) {
		T closest_data;
		if (buffer.empty()) {
			closest_data.timestamp = -1;
			return closest_data;
		}

		unsigned int buf_size = buffer.size();
		if(time_stamp <= buffer[0].timestamp)
			closest_data = buffer[0];
		else if (time_stamp >= buffer[buf_size-1].timestamp)
			closest_data = buffer[buf_size-1];
		else {
			for(unsigned int i = buf_size-1; i > 0; --i) {
				if(buffer[i].timestamp >= time_stamp && buffer[i-1].timestamp < time_stamp) {
					closest_data = buffer[i];
					if(buffer[i].timestamp-time_stamp > time_stamp-buffer[i-1].timestamp)
						closest_data = buffer[i-1];

					break;
				}
			}
		}

		return closest_data;
	}

	// get closet data from a increase deque with std::pair data on timestamp.
	template<typename T>
	T getClosestPair(const int64_t& time_stamp, const std::deque<T>& buffer) {
		T closest_data;
		if (buffer.empty()) {
			closest_data.first = -1;
			return closest_data;
		}

		unsigned int buf_size = buffer.size();
		if (time_stamp <= buffer[0].first)
			closest_data = buffer[0];
		else if (time_stamp >= buffer[buf_size - 1].first)
			closest_data = buffer[buf_size - 1];
		else {
			for (unsigned int i = buf_size - 1; i > 0; --i) {
				if (buffer[i].first >= time_stamp && buffer[i - 1].first < time_stamp) {
					closest_data = buffer[i];
					if (buffer[i].first - time_stamp > time_stamp - buffer[i - 1].first)
						closest_data = buffer[i - 1];
					break;
				}
				else
					closest_data.first = -1;
			}
		}

		return closest_data;
	}
	
	template <typename T>
	double gray2color(cv::Mat img_gray, cv::Mat& img_color, double maxVal=0)
	{
		if(img_gray.channels() != 1)
		{
			//cout << "in showFakeColor Func the input image must be gray" << endl;
			return 0;
		}

		img_color = cv::Mat(img_gray.rows, img_gray.cols, CV_8UC3);

		double minValue, maxValue;

		if(maxVal == 0)
			cv::minMaxLoc(img_gray, &minValue, &maxValue);
		else
			maxValue = maxVal;

		cv::Mat img_gray_float(img_gray.rows, img_gray.cols, CV_32FC1);

		for(int y = 0; y < img_gray_float.rows; y++)
		{
			T *ptr_img_gray = img_gray.ptr<T>(y);
			float *ptr_img_gray_float = img_gray_float.ptr<float>(y);

			for(int x = 0; x < img_gray_float.cols; x++)
			{
				ptr_img_gray_float[x] = MIN( float(ptr_img_gray[x]) / maxValue , 1.0f );
			}
		}

		float tmp = 0;
		for(int y = 0; y < img_gray.rows; y++)
		{		
			float *ptr = img_gray_float.ptr<float>(y);
			cv::Vec3b *ptr_rgb = img_color.ptr<cv::Vec3b>(y);
			for(int x = 0; x < img_gray.cols; x++)
			{
				tmp = ptr[x]*255;

				if (0 == tmp)
				{
					ptr_rgb[x][0] = 0;
					ptr_rgb[x][1] = 0;
					ptr_rgb[x][2] = 0;
				}
				else if (tmp <= 51)
				{
					ptr_rgb[x][0] = 255;
					ptr_rgb[x][1] = tmp * 5;
					ptr_rgb[x][2] = 0;
				}
				else if (tmp <= 102)
				{
					tmp-=51;

					ptr_rgb[x][0] = 255-tmp*5;
					ptr_rgb[x][1] = 255;
					ptr_rgb[x][2] = 0;
				}
				else if (tmp <= 153)
				{
					tmp-=102;

					ptr_rgb[x][0] = 0;
					ptr_rgb[x][1] = 255;
					ptr_rgb[x][2] = tmp*5;
				}
				else if (tmp <= 204)
				{
					tmp-=153;

					ptr_rgb[x][0] = 0;
					ptr_rgb[x][1] = 255-uchar(128.0*tmp/51.0+0.5);
					ptr_rgb[x][2] = 255;
				}
				else
				{
					tmp-=204;

					ptr_rgb[x][0] = 0;
					ptr_rgb[x][1] = 127-uchar(127.0*tmp/51.0+0.5);
					ptr_rgb[x][2] = 255;
				}
			}
		}
		return maxValue;
	}

	inline bool is_exists (const std::string& name) {
		std::ifstream f(name.c_str());
		return f.good();
	}
}
#endif //_ALGO_UTILS_H
