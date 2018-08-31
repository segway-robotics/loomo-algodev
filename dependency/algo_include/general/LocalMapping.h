#ifndef _LOCAL_MAPPING_H_
#define _LOCAL_MAPPING_H_

#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <Eigen/Dense>
#include <RawDataUtil.h>

class OccupancyGridMap2D;
class Observation;

namespace ninebot_algo
{
	namespace local_mapping
	{

		class NINEBOT_EXPORT LocalMapping
		{
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			/**
			* @breif This function construct function, setting map size and resolution.
			* Coordination system is as follows, the center of coordinate is the wheel base center.
			*		z   x
			*		|  /
			*		| /
			*	y---|/
			* @param mapsize: distance between map left to right boundary (same distance between top to buttom boundary) [meter]
			* @param resolution: dimension of one map cell [meter]
			*/				     
			LocalMapping(float mapsize = 6.0f,
						 float resolution = 0.1f);

			~LocalMapping();

			/**
			* @breif set depth camera intrinsic parameter.
			* @param cx 
			* @param cy
			* @param fx 
			* @param fy
			* @param scalar distance scale[millimeter], 1000 millimeter == 1 meter
			* @param depth_width width of depth image, default depth size is 320x240
			*/
			void setDepthCameraParams(float cx, float cy, float fx, float fy, float scalar, int depth_width = 320);

			/**
			* @breif set depth sense range.
			* @param range_distance_max max sense distacne in z axis(realsense coordinate system) 
			* @param range_distance_min mix sense distacne in z axis(realsense coordinate system)
			* @param range_height_max max sense height in y axis(realsense coordinate system)
			* @param range_height_min min sense height in y axis(realsense coordinate system)
			*/
			void setDepthRange(float range_distance_max = 3.0f, 
							   float range_distance_min = 0.35f, 
							   float range_height_max = 1.0,
							   float range_height_min = 0.1);
			
			/**
			* @breif setting depth occupancy map engine params
			* @param maxOccupancyUpdateCertaintyDepth: update certainty 0.5~1
			* @param ptsInOneCellThresholdDepth: the point count threshold in each cell to filter noise [depth point cloud conversion to 2D map]
			* @param insertOnlyNearestPtsDepth: if only insert the nearest occupied cell or all 3D - 2D cells
			* @param extendFovAsFree: extend the FOV of depth camera so that side lobe of depth view is freed (in rads), must be larger than realsense FOV (~54 degree)
			*/
			void setDepthMapParams(float maxOccupancyUpdateCertaintyDepth = 0.8, int ptsInOneCellThresholdDepth = 30, bool insertOnlyNearestPtsDepth = false, float extendFovAsFree = -1);

			/**
			* @breif set lidar occupancy map engine params
			* @param maxOccupancyUpdateCertaintyLidar: update certainty 0.5~1
			* @param isUnknownAsFree: is unknown value of lidar sensor treated as free space
			* @param lidarNumMultiplier: to multiply the number of lidar rays by interpolation
			*/
			void setLidarMapParams(float maxOccupancyUpdateCertaintyLidar = 0.6, bool isUnknownAsFree = true, int lidarNumMultiplier = 2);
			/**
			* @breif set valid range of lidar sensors
			*/
			void setLidarRange(float range_distance_max);
			/**
			* @breif set valid range of ultrasonic sensors
			*/
			void setUltrasonicRange(float range_distance_max);
			/**
			* @breif set valid ranges of base infrared sensors
			* @param range_distance_max[]
			* id: {	frontRight, frontCenter, frontLeft, leftRear, leftFront, leftFrontCorner,
			*		rearLeft, rearCenter, rearRight, rightRear, rightFront, rightFrontCorner }
			*/
			void setBaseInfraredRange(float range_distance_max[]);
			/**
			* @breif set valid ranges of sonar fusion sensors
			* @param range_distance_max[]
			* id: Infrared {	frontRight, frontCenter, frontLeft, leftRear, leftFront, leftFrontCorner,
			*		rearLeft, rearCenter, rearRight, rightRear, rightFront, rightFrontCorner }
			* @param clearFarDistance: to clear far distance region if reading exceeds range_distance_max
			*/
			void setSonarFusionParams(float range_distance_max[], bool clearFarDistance);
			/**
			* @breif set falldown threshold of IR sensors
			*/
			void setFalldownThreshold(float threshold);
			/**
			* @breif ds5 occupancy map engine params
			*/
			void setDs5CameraParams(float cx_left, float cy_left, float fx_left, float fy_left, float cx_right, float cy_right, float fx_right, float fy_right, float scalar);
			void setDs5Range(float range_distance_max, float range_distance_min, float range_height_max, float range_height_min);
			void setDs5MapParams(float extendFovAsFree);

			/**
			* @breif clear local map
			*/
			void clearMap();

			// process different sensor input
			void processDepthFrame(StampedMat &depth, StampedPose &odom, StampedHeadPos &headPose, StampedBasePos &basePose); // legacy, to be removed in future
			void processDepthFrame(StampedMat &depth, StampedPose &odom, Eigen::Isometry3f &depthpose);
			void processLidarFrame(StampedLidar &lidarFront, StampedLidar &lidarRear, StampedPose &odom);
			void processUltrasonicFrame(StampedFloat &ultra, StampedUltrasonics &ultraE, StampedPose &odom);
			void processDualDs5Frame(const StampedMat& depth_left, const Eigen::Isometry3f &depthpose_left, const StampedMat& depth_right, const Eigen::Isometry3f &depthpose_right, const StampedPose &odom, const int sel = 2);
			// latch_time_bumper_falldown (seconds)
			void processAuxSensorGXFrame(const StampedFloat ultrasonic, const StampedInfraredArray irs, const StampedBumper bumper, const StampedIr fallsensor, const StampedPose &odomPose, const float latch_time_bumper_falldown = 10);
			/**
			* @breif local obstacle map, same orientation with global coordinate
			*		 deafult: 6m x 6m, crop from FixMap
			*		 Note that, pixel value represents the grid corner, but not the middle of pixel
			* @return:(width, height) pair of the map
			* @param probmap probability map, occupacied value is 0~100, meaning occupacied probability, 255 meaning other tracking or show label
			* @param isExtractSubpixelFromGlobalmap using subpixel algorithm when extract localmap from the global fixmap (this is bad when detect only an isolated single point)
			* @param newpos extract local map from the desired pose, if empty pose is provided, then the latest pose that is fed is used
			*/
			std::pair<int, int> getDepthMap(cv::Mat& probmap, bool isExtractSubpixelFromGlobalmap, StampedPose newpos);
			// depth map with front ultrasonic
			std::pair<int, int> getDepthMapWithFrontUltrasonic(cv::Mat& probmap, bool isExtractSubpixelFromGlobalmap, StampedPose newpos, float ultrasonic);
			// lidar map
			std::pair<int, int> getLidarMap(cv::Mat& probmap, bool isExtractSubpixelFromGlobalmap, StampedPose newpos);
			// fused map
			std::pair<int, int> getFusedMap(cv::Mat& probmap, bool isExtractSubpixelFromGlobalmap, StampedPose newpos, int robot_model = 0);
			// ds5 map
			std::pair<int, int> getDs5Map(cv::Mat& probmap, bool isExtractSubpixelFromGlobalmap, StampedPose newpos);

			/**
			* @breif global obstacle map, same orientation with global coordinate
			*		 deafult: 30m x 30m, defined in structs.h MAP_X_MAX,MAP_X_MIN,MAP_Y_MAX,MAP_Y_MIN
			* @param fix_map occupancy probability map, occupacied value is 0~100, meaning occupacied probability, 255 meaning other tracking or show label
			* @param mapCenter map center pose in global coordination(odometry coordination),[x,y,orientation]
			* @param width  map width 
			* @param height map height 
			* @param sel 0-depth (default), 1-lidar, 2-ultrasonic
			*/
			void getFixMap(cv::Mat &fix_map, Eigen::Vector3d &mapCenter, int &width, int &height, int sel=0);

			/**
			* @breif draw on localmap and the robot pose (flipped to have the correct visual coordinate for opencv:imshow)
			*/
			cv::Mat showLocalMapWithRobotPose(cv::Mat& localMap, float scale, float fov = 60, float headyaw = 0, int robot_model = 1);

			/**
			* @breif get map dimention, same meaning with LocalMapping construction
			*/
			void getMapDimention(float &mapsize);

			/**
			* @breif get map resolution, same meaning with LocalMapping construction
			*/
			void getMapResolution(float &resolution);
			
			/**
			* @breif show robot pose
			*/
			void showRobotPose(cv::Mat &showImg, Eigen::Vector3d cur_pose, Eigen::Vector3d origin_point, float resolution, int width, int height);
			
			/**
			* @breif internal use below, to determine if obstacle exist in front
			* @param the prob map
			* @param distance criteria for determine obstacle
			*/
			int checkCollision(cv::Mat& localMap, float dist_danger);

			/**
			* @breif internal use, only process one frame, not keep history
			* @param depth depth image
			* @param odompose odometry pose[x,y,orientation]
			* @param head_yaw head pose yaw
			* @param basepose base pose[roll, pitch, yaw]
			* @param show_red show debug information or not
			*/
			cv::Mat step_frame_depth(const cv::Mat &depth, const Eigen::Vector3f &odompose, const float head_yaw, const Eigen::Vector3f &basepose, bool show_red);
			cv::Mat step_frame_lidar(const StampedLidar &LidarFrontData, const StampedLidar &lidarRearData, const Eigen::Vector3f &odompose);
			cv::Mat step_frame_ds5(const StampedMat& depth_left, const Eigen::Isometry3f &depthpose_left, const StampedMat& depth_right, const Eigen::Isometry3f &depthpose_right, const StampedPose &odom, const int sel=2);

			/* set value at specific location */
			void setMapValue(float x, float y, int val, float radius = 0.0);
			/* clear gx map (depth and ds5) with a front FOV */
			void clearGxMapFovRegion(const StampedPose odom, const float fov);

		private:
			OccupancyGridMap2D *pGridMap_;
			std::vector<double> robot_pos_;
			float max_;
			float resolution_;
			int width_, height_;
			cv::Point2f center_;
			Eigen::Vector3d mapCenter_;
			int64 time_prev_depth;
			int64 time_prev_lidarF;
			int64 time_prev_lidarR;
			int64 time_prev_ultrasonic;
			int64 time_prev_ds5_left, time_prev_ds5_right;
			int64 time_prev_aux;

			float max_x_, max_y_, min_x_, min_y_;
			int outWidth_, outHeight_;
			float mapsize_;

		};
	}
}
#endif
