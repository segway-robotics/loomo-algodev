#include "AlgoLocalmappingTest.h"
#include <unistd.h>
#include <set>
#include "AlgoUtils.h"
#include "jniAlgo.h"

#undef DEFAULT_LOG_TAG
#define DEFAULT_LOG_TAG "LocalMapping"

namespace ninebot_algo
{
	namespace localmapping_test
	{
		using namespace std;
		using namespace cv;

		AlgoLocalmappingTest::~AlgoLocalmappingTest()
		{
            mRawDataInterface->ExecuteHeadMode(0);
            mRawDataInterface->ExecuteHeadSmoothFactor(0,0,0);
            mRawDataInterface->ExecuteHeadPos(0, 0, 0);

            if(m_p_local_mapping) {
                delete m_p_local_mapping;
                m_p_local_mapping = NULL;
            }

            if(m_p_localmapping_rawdata != NULL) {
                delete m_p_localmapping_rawdata;
                m_p_localmapping_rawdata = NULL;
            }
		}

		void AlgoLocalmappingTest::clearMap(int sel)
		{
		    if(m_p_local_mapping){
                if(sel==0)
                    m_p_local_mapping->clearMap();
                else if(sel==1){
                    ninebot_tf::tf_message cur_pose_tf = mRawDataInterface->GetTfBewteenFrames(
                            "base_center_ground_frame", "world_odom_frame" , raw_depth.timestampSys, 200);
                    StampedPose cur_odom_pos = tfmsgTo2DPose(cur_pose_tf);
                    m_p_local_mapping->clearGxMapFovRegion(cur_odom_pos, 120.f*M_PI/180.f);
                }
            }
		}


		bool AlgoLocalmappingTest::init()
		{
            vector<Eigen::Vector2f> ref_trajectory;

            robot_model = RawData::retrieveRobotModel();
            mRawDataInterface->ExecuteHeadMode(0);
            mRawDataInterface->ExecuteHeadSmoothFactor(0,0,0);
            mRawDataInterface->ExecuteHeadPos(0, 0, 0);

    		// record 
			m_ptime = 10;

            return true;
		}

        bool AlgoLocalmappingTest::step()
        {
            auto start = std::chrono::high_resolution_clock::now();
			prepare_localmap_and_pose_for_controller_g1();

            /*! Calculate the algorithm runtime */
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> elapsed = end - start;
	        {
		        std::lock_guard<std::mutex> lock(mMutexTimer);
		        m_ptime = elapsed.count()*0.5 + m_ptime*0.5;
	        }

            drawCanvas();
            setDisplayData();

            return true;
        }

        bool AlgoLocalmappingTest::prepare_localmap_and_pose_for_controller_g1() {
            // ultrasonic
            StampedFloat raw_ultrasonic;
            StampedUltrasonics raw_external_ultrasonics;
            mRawDataInterface->retrieveUltrasonic(raw_ultrasonic);
            mRawDataInterface->retrieveExternalUltrasonics(raw_external_ultrasonics);

            // generate mapping
            mRawDataInterface->retrieveDepth(raw_depth, true);

			if(raw_depth.timestampSys == 0) {
                ALOGD("AlgoLocalmappingTest: local mapping:: depth wrong");
                return false;
            }
            StampedMat local_depth(raw_depth.image, raw_depth.timestampSys);

            // merge all TF requests
            ninebot_tf::vector_req_t reqList;
            ninebot_tf::vector_tf_msg_t resList;
            reqList.push_back(ninebot_tf::tf_request_message("base_center_ground_frame", "world_odom_frame", raw_depth.timestampSys, 500));
            reqList.push_back(ninebot_tf::tf_request_message("rsdepth_center_neck_fix_frame", "world_odom_frame", raw_depth.timestampSys, 500));
            mRawDataInterface->getMassiveTfData(&reqList,&resList);
            if(resList.size()!=2){
                ALOGE("getMassiveTfData wrong");
                return false;
            }

            ninebot_tf::tf_message tf_msg = resList[0];
            ninebot_tf::tf_message tf_msg2 = resList[1];
            if(tf_msg.err==0 && tf_msg2.err==0){
                StampedPose odom_pos = tfmsgTo2DPose(tf_msg);
                Eigen::Isometry3f depth_pose = PoseToCamcoor(tfmsgToPose(tf_msg2));
                // process depth
                m_p_local_mapping->processDepthFrame(local_depth, odom_pos, depth_pose);
            }
            else{
                ALOGE("Could not find tf pose with specified depth ts, error code: %d, %d", tf_msg.err, tf_msg2.err);
            }
			m_p_local_mapping->getDepthMap(local_map_for_controller, false, tfmsgTo2DPose(resList[0]));

			StampedPose local_odo = tfmsgTo2DPose(resList[0]);
            m_cur_pose[0] = float(local_odo.pose.x);
            m_cur_pose[1] = float(local_odo.pose.y);
            m_cur_pose[2] = float(local_odo.pose.orientation);

            return true;
        }

		void AlgoLocalmappingTest::scanHead(float pitch, float yaw_limit_degree)
		{
            // rorate head
			StampedHeadPos raw_headpos;
			mRawDataInterface->retrieveHeadPos(raw_headpos);

			float angle_target = abs(yaw_limit_degree)*3.1415/180;

            if(scan_round==0){
				if((raw_headpos.yaw>-angle_target)/* && m_allow_navigation*/) {
                    //mRawDataInterface->ExecuteHeadPos(-angle_target, pitch, 0);
                    mRawDataInterface->ExecuteHeadPos(0, pitch, 2);
                    mRawDataInterface->ExecuteHeadSpeed(-m_head_speed, 0, 1);
                    ALOGD("scanHead:%f, scan_round:%d, speed:%f", raw_headpos.yaw*180/3.14, scan_round, -m_head_speed);
                }
				else
					scan_round = 1;
			}
			else{
                if((raw_headpos.yaw<angle_target)/* && m_allow_navigation*/){
                    //mRawDataInterface->ExecuteHeadPos(angle_target, pitch, 0);
                    mRawDataInterface->ExecuteHeadPos(0, pitch, 2);
                    mRawDataInterface->ExecuteHeadSpeed(m_head_speed, 0, 1);
                    ALOGD("scanHead:%f, scan_round:%d, speed:%f", raw_headpos.yaw*180/3.14, scan_round, m_head_speed);
                }
				else
					scan_round = 0;
			}
		}

        bool AlgoLocalmappingTest::initLocalMapping()
        {
            if(m_p_local_mapping) {
                delete m_p_local_mapping;
                m_p_local_mapping = NULL;
            }
            float mapsize = 6.0;
            m_map_resolution = 0.05;
            m_p_local_mapping = new ninebot_algo::local_mapping::LocalMapping(mapsize, m_map_resolution);
            if(m_p_local_mapping == NULL)
                return false;
            CalibrationInfoDS4T calib;
            mRawDataInterface->getCalibrationDS4T(calib);
            float fx = calib.depth.focalLengthX;
            float fy = calib.depth.focalLengthY;
            float px = calib.depth.pricipalPointX;
            float py = calib.depth.pricipalPointY;
            m_p_local_mapping->setLidarRange(5.0);
			m_p_local_mapping->setLidarMapParams(0.6, true);
			m_p_local_mapping->setDepthCameraParams(px, py, fx, fy, 1000);
            m_p_local_mapping->setDepthRange(3.5, 0.35, 0.9, 0.1);
            m_p_local_mapping->setDepthMapParams(0.8, 30, false, 64*M_PI/180.0);
            m_p_local_mapping->setUltrasonicRange(0.5);
            m_map_width =  mapsize / m_map_resolution;
            m_map_height = mapsize / m_map_resolution;

            return true;
        }

		std::string AlgoLocalmappingTest::getDebugString()
		{
			std::string str;
            int raw_errorcode = mRawDataInterface->retrieveDeviceErrorCode();
            if (raw_errorcode != 0)
                str = ", err:"+ToString(raw_errorcode);
			return str;
		}

		float AlgoLocalmappingTest::runTime()
		{
			std::lock_guard<std::mutex> lock(mMutexTimer);
			return m_ptime;
		}

        void AlgoLocalmappingTest::add_show_fov(float cur_angle, float fov_angle, cv::Mat &show_img, float radius_per_meter)
        {
            cv::Point2f center = cv::Point2f(show_img.cols / 2, show_img.rows / 2);
            cv::Point2f left, right;
            float range = 50;
            if (cur_angle > M_PI)
                cur_angle -= 2 * M_PI;
            if (cur_angle < -M_PI)
                cur_angle += 2 * M_PI;
            left.x = center.x + range * cos(cur_angle - fov_angle / 2);
            left.y = center.y + range * sin(cur_angle - fov_angle / 2);

            right.x = center.x + range * cos(cur_angle + fov_angle / 2);
            right.y = center.y + range * sin(cur_angle + fov_angle / 2);

            cv::line(show_img, center, left, cv::Scalar(0, 0, 255), 0.5, cv::LINE_4);
            cv::line(show_img, center, right, cv::Scalar(0, 0, 255), 0.5, cv::LINE_4);
            if (radius_per_meter > 0) {
                cv::circle(show_img, center, 1 * radius_per_meter, cv::Scalar(255, 160, 160), 1.5);
                cv::circle(show_img, center, 2 * radius_per_meter, cv::Scalar(255, 160, 160), 1.5);
                cv::circle(show_img, center, 3 * radius_per_meter, cv::Scalar(255, 160, 160), 1.5);
                cv::line(show_img, cv::Point2f(show_img.cols / 2, 0), cv::Point2f(show_img.cols / 2, show_img.rows), cv::Scalar(255, 160, 160), 1.5);
                cv::line(show_img, cv::Point2f(0, show_img.rows / 2), cv::Point2f(show_img.cols, show_img.rows / 2), cv::Scalar(255, 160, 160), 1.5);

            }
        }

        void AlgoLocalmappingTest::drawLocalMapping(){
            {
                if(local_map_for_controller.empty())
                    return;
                std::lock_guard<std::mutex> lock(m_mutex_display);
                cv::Mat show = m_p_local_mapping->showLocalMapWithRobotPose(local_map_for_controller, 300.f/m_map_width, (robot_model==2?136:60), 0, robot_model);
                cv::Mat ca1 = m_canvas(cv::Rect(20,20,show.cols, show.rows));

                show.copyTo(ca1);
            }

            return;
        }

		void AlgoLocalmappingTest::drawCanvas() {
			if (m_is_render) {
                drawLocalMapping();
			}

			return ;
		}

		bool AlgoLocalmappingTest::showScreen(void* pixels) {
			{
				std::lock_guard<std::mutex> lock(m_mutex_display);
				if (m_display_image.empty())
					return false;
				cv::cvtColor(m_display_image, m_display_data, CV_BGR2RGBA);
			}
			memcpy(pixels, (void *)m_display_data.data, m_display_data.cols * m_display_data.rows * 4);
			return true;
		}

		void AlgoLocalmappingTest::setDisplayData() {
			std::lock_guard<std::mutex> lock(m_mutex_display);
			m_display_image = m_canvas.clone();
		}

	} // namespace tango_nav
} // namespace ninebot_algo
