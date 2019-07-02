#include "AlgoFollow.h"
#include "ninebot_log.h"
#include "AlgoUtils.h"
#include "Quaternion.h"
#include "Vector3.h"

#include <algorithm>
#include "SocketServer.h"

#define PI 3.1415

namespace ninebot_algo
{
	namespace follow_algo
	{
		using namespace std;
		using namespace cv;

        AlgoFollow::AlgoFollow(RawData *rawInterface, int run_sleep_ms, bool isRender)
                :AlgoBase(rawInterface,run_sleep_ms,true)
        {
                mRawDataInterface->ExecuteHeadMode(0);
                mRawDataInterface->ExecuteHeadPos(0, 0.7, 0);            
                m_is_init_succed = false;
                m_ptime = 10;
                m_isRender = isRender;
                pose_isRecording = false;
                canvas = cv::Mat::zeros( cv::Size(640, 360), CV_8UC3 );
                m_safety_control = true;
                m_is_track_head = false;
                m_is_track_vehicle = false;
                m_p_local_mapping = NULL;
                m_target_distance = -1.0f;
                m_target_theta = 0.0f;
                init();
        }

		AlgoFollow::~AlgoFollow() {
            this->stopPoseRecord();
            
            mRawDataInterface->ExecuteHeadMode(0);
            mRawDataInterface->ExecuteHeadPos(0, 0.7, 0);      

            if(m_p_local_mapping) {
                delete m_p_local_mapping;
                m_p_local_mapping = NULL;
            }

            if(m_p_localmapping_rawdata != NULL) {
                delete m_p_localmapping_rawdata;
                m_p_localmapping_rawdata = NULL;
            }

            if(m_p_server != NULL) {
                delete m_p_server;
                m_p_server = NULL;
            }

            if(m_p_head_yaw_tracker != NULL) {
                delete m_p_head_yaw_tracker;
                m_p_head_yaw_tracker = NULL;
            }

            if(m_p_head_pitch_tracker != NULL) {
                delete m_p_head_pitch_tracker;
                m_p_head_pitch_tracker = NULL;
            }

            if(m_p_vehicle_tracker != NULL) {
                delete m_p_vehicle_tracker;
                m_p_vehicle_tracker = NULL;
            }            

            // depth 
            if (m_p_depth_processor){
                delete(m_p_depth_processor);
            }         

            delete[] bounding_box;   
		}

		bool AlgoFollow::init()
		{
            mRawDataInterface->ExecuteHeadMode(0);
            mRawDataInterface->ExecuteHeadPos(0, 0.7, 0);

			raw_depth.image = cv::Mat::zeros(cv::Size(320, 240), CV_16UC1);
			raw_depth.timestampSys = 0;
			t_old = 0;
		    imgstream_en = true;
            nStep = 0;

			mRawDataInterface->getMaincamParam(raw_camerapara);
			mRawDataInterface->getCalibrationDS4T(calib);

            string serial = RawData::retrieveRobotSerialNumber();
			ALOGD("robot serial: %s", serial.c_str());
			ALOGD("robot model: %d", RawData::retrieveRobotModel());
			m_is_init_succed = true;

            m_p_server = new SocketServer;
            m_timestamp_start = mRawDataInterface->getCurrentTimestampSys();

            initLocalMapping();

            float head_kp, head_ki, head_kd;
            float vehicle_kp, vehicle_ki, vehicle_kd;
            loadConfig(head_kp, head_ki, head_kd, vehicle_kp, vehicle_ki, vehicle_kd);

            m_p_head_yaw_tracker = new PID(0.1, 0.5, -0.5, head_kp,  head_ki, head_kd);
            m_p_head_pitch_tracker = new PID(0.1, 0.1, -0.1, 0.1, 0.01, 0.01);
            m_p_vehicle_tracker = new PID(0.1, 1.0, -0.5, vehicle_kp, vehicle_ki, vehicle_kd);

            m_direction_head_test = 0;

            // depth 
            m_p_depth_processor = new DepthPreprocess();

            // detection 
            m_is_detected = false;
            bounding_box = new float[5];

			return m_is_init_succed;
		}

		void AlgoFollow::UpdateAccel(MTPoint val)
		{
			raw_accel = val;
		}

		void AlgoFollow::UpdateGyro(MTPoint val)
		{
			raw_gyro = val;
		}

		void AlgoFollow::toggleImgStream()
		{
			imgstream_en = !imgstream_en;
		}

		void AlgoFollow::setVLSopen(bool en)
        {
            if(en)
                mRawDataInterface->startVLS(false);
            else
                mRawDataInterface->stopVLS();
        }

        void AlgoFollow::startPoseRecord(std::string save_folder, int64_t save_time)
        {
            pose_isRecording = true;
            nStep = 0;
            m_folder_socket = "/sdcard/socket/";
            createFolder(m_folder_socket);        
            m_state_file.open(m_folder_socket + "state.txt");    
        }

        void AlgoFollow::stopPoseRecord()
        {
            pose_isRecording = false;
            m_state_file.close();
        }

		bool AlgoFollow::step()
		{
			/*! Get start timestamp for calculating algorithm runtime */
			auto start = std::chrono::high_resolution_clock::now();

			if(!m_is_init_succed){
				ALOGD("AlgoFollow: init false");
				return false;
			}

            if(imgstream_en){
                mRawDataInterface->retrieveDepth(raw_depth, false);
                if(raw_depth.timestampSys==0)
                {
                    ALOGD("depth wrong");
                    return false;
                }
                // check if depth is duplicate
                if(t_old==raw_depth.timestampSys)
                {
                    ALOGD("depth duplicate: %lld",raw_depth.timestampSys/1000);
                    return true;
                }
                else
                {
                    t_old=raw_depth.timestampSys;
                    ALOGD("depth correct");
                } 
            }

            /*! **********************************************************************
             * **** Local 
             * ********************************************************************* */         
            mRawDataInterface->retrieveOdometry(raw_odometry, -1);
            prepare_localmap_and_pose_for_controller_g1();

            /*! **********************************************************************
             * **** Cloud
             * ********************************************************************* */         
            this->stepServer();

			/*! **********************************************************************
			 * **** Processing the algorithm with all input and sensor data **********
			 * ********************************************************************* */
			 string contents;

			 if(m_isRender)
			 {
				/*! Copy internal canvas to intermediate buffer mDisplayIm */
                renderDisplay();
                setDisplayData();
			}

			/*! Calculate the algorithm runtime */
			auto end = std::chrono::high_resolution_clock::now();
			std::chrono::duration<double, std::milli> elapsed = end-start;
			ALOGD("step time: %f",elapsed.count());
			{
				std::lock_guard<std::mutex> lock(mMutexTimer);
				m_ptime = elapsed.count()*0.5 + m_ptime*0.5;
			}

			return true;
		}

        void AlgoFollow::stepServer() {

            // generate mapping
            // mRawDataInterface->retrieveDS4Color(raw_colords4, true);
            mRawDataInterface->retrieveColor(raw_color, true);

            ALOGD("raw_color: (%lld,%d,%d,%d)", raw_color.timestampSys, raw_color.image.cols, raw_color.image.rows, raw_color.image.channels()); 

            if (raw_color.image.empty()) {
                ALOGW("empty raw_color image");
                mRawDataInterface->ExecuteCmd(0.0f, 0.0f, mRawDataInterface->getCurrentTimestampSys()); 
                return;
            }

            if (m_p_server->isStopped()) {
                mRawDataInterface->ExecuteCmd(0.0f, 0.0f, mRawDataInterface->getCurrentTimestampSys()); 
                ALOGW("server stopped");
                return;
            }

            if (!m_p_server->isConnected()) {
                mRawDataInterface->ExecuteCmd(0.0f, 0.0f, mRawDataInterface->getCurrentTimestampSys());
                ALOGW("server disconnected");
                mRawDataInterface->ExecuteHeadMode(0);
                mRawDataInterface->ExecuteHeadPos(0, 0.7, 0);      
                return;
            }

            cv::Mat image_send;
            cv::resize(raw_color.image, image_send, cv::Size(640/m_down_scale, 480/m_down_scale)); 
            int info_send_image = m_p_server->sendImage(image_send, 640/m_down_scale, 480/m_down_scale);
            if (info_send_image < 0) {
                ALOGW("server send iamge failed");
                mRawDataInterface->ExecuteHeadMode(0);
                mRawDataInterface->ExecuteHeadPos(0, 0.7, 0);
                return;
            }
            else {
                ALOGW("server send iamge succeeded");
            }

            // Receive Bounding Box coordinates (5 floats)
            float* floats_recv = new float[5];
            int rcv_info_test = m_p_server->recvFloats(floats_recv, 5);
            if (rcv_info_test < 0) {
                ALOGD("server rcv float failed");
                return;
            }
            else {
                for (int i = 0; i < 5; i++)
                {
                    bounding_box[i] = *(float*)&floats_recv[i];
                    ALOGD("server bounding_box #%d: %.2f", i, bounding_box[i]);
                }
            }
            delete[] floats_recv;
            
            m_roi_color.width = int(bounding_box[2] * m_down_scale);
            m_roi_color.height = int(bounding_box[3] * m_down_scale);
            m_roi_color.x = (bounding_box[0] - 320/m_down_scale) * m_down_scale + 320;
            m_roi_color.y = (bounding_box[1] - 240/m_down_scale) * m_down_scale + 240;
            // convert from center to top-left corner 
            m_roi_color.x = int(m_roi_color.x - m_roi_color.width/2); 
            m_roi_color.y = int(m_roi_color.y - m_roi_color.height/2); 

            if (bounding_box[4] > 0.5) 
                m_is_detected = true; 
            else 
                m_is_detected = false; 

            float target_theta_wrt_head =0.0f; 
            if (m_is_detected) {
                ExtractTarget(m_target_distance, target_theta_wrt_head);
                m_target_theta = target_theta_wrt_head + raw_headpos.yaw;
            }
            else { 
                m_target_distance = -1.0f;
                m_target_theta = 0.0f;
                target_theta_wrt_head = 0.0f;
            }

            this->trackHead(m_target_theta);
            this->trackVehicle(m_target_distance, m_target_theta);
            ALOGD("Vehicle Target: target_distance = %f, target_theta_wrt_head = %f", m_target_distance, target_theta_wrt_head);

            return;
        }

        void AlgoFollow::switchHeadTracker() {
            m_is_track_head = !m_is_track_head;
        }

        void AlgoFollow::switchVehicleTracker() {
            m_is_track_vehicle = !m_is_track_vehicle;
            m_is_track_head = m_is_track_vehicle;
        }

        void AlgoFollow::safeControl(float v, float w) {
            if (m_safety_control) {
                const float kCloseObstacleThres = 1.0;
                if(m_ultrasonic_average < kCloseObstacleThres*1000){
                    if (v > 0 && std::abs(v/std::fmax(0.01,w)) > 0.8){
                        StampedVelocity velocity;
                        mRawDataInterface->retrieveBaseVelocity(velocity);
                        float v_emergency = std::min(0.0, 0.2-velocity.vel.linear_velocity);
                        ALOGE("command dangerous: (%f,%f), current vel: (%f,%f), ultrasonic_average = %f: v_emergency = %f", v, w, velocity.vel.linear_velocity, velocity.vel.angular_velocity, m_ultrasonic_average, v_emergency);
                        mRawDataInterface->ExecuteCmd(v_emergency, 0.0f, 0);
                        return;
                    }
                }
            }

            mRawDataInterface->ExecuteCmd(v, w, 0);
            ALOGD("command safe: (%f,%f)", v, w);
        }

		float AlgoFollow::runTime()
		{
			std::lock_guard<std::mutex> lock(mMutexTimer);
			return m_ptime;
		}

		bool AlgoFollow::showScreen(void* pixels) // canvas 640x360, RGBA format
		{
			{
				std::lock_guard<std::mutex> lock(mMutexDisplay);
				if(mDisplayIm.empty())
					return false;
				cv::cvtColor(mDisplayIm, mDisplayData, CV_BGR2RGBA);
			}
			memcpy(pixels, (void *)mDisplayData.data, mDisplayData.cols * mDisplayData.rows * 4);
			return true;
		}

		void AlgoFollow::setDisplayData()
		{
			std::lock_guard<std::mutex> lock(mMutexDisplay);
			mDisplayIm = canvas.clone();
		}

        void AlgoFollow::createFolder(std::string new_folder_name)
        {
            std::string cmd_str_rm = "rm -rf \"" + new_folder_name + "\"";
            system(cmd_str_rm.c_str());
            ALOGD("Command %s was executed. ", cmd_str_rm.c_str());          
            std::string cmd_str_mk = "mkdir \"" + new_folder_name + "\"";
            system(cmd_str_mk.c_str());
            ALOGD("Command %s was executed. ", cmd_str_mk.c_str());
        }

        std::string AlgoFollow::getDebugString()
        {
            std::string str;

            if (m_safety_control){
                str = "，safety ON"; 
            }
            else {
                str = "，safety OFF"; 
            }

            int raw_errorcode = mRawDataInterface->retrieveDeviceErrorCode();
            if (raw_errorcode != 0)
                str += ", err:"+ToString(raw_errorcode);

            return str;
        }

        bool AlgoFollow::initLocalMapping()
        {
            if(m_p_local_mapping) {
                delete m_p_local_mapping;
                m_p_local_mapping = NULL;
            }
            float mapsize = 8.0;
            float m_map_resolution = 0.05;
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
            m_p_local_mapping->setDepthMapParams(1.0, 10, false, -1);
            m_p_local_mapping->setUltrasonicRange(0.5);
            m_map_width =  mapsize / m_map_resolution;
            m_map_height = mapsize / m_map_resolution;

            return true;
        }

        bool AlgoFollow::prepare_localmap_and_pose_for_controller_g1() {
            // generate mapping
            mRawDataInterface->retrieveDepth(raw_depth, true);

            if(raw_depth.timestampSys == 0) {
                ALOGD("localmap: depth wrong");
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

            // clear history 
            m_p_local_mapping->clearMap();
            // m_p_local_mapping->setMapValue(raw_odometry.twist.pose.x, raw_odometry.twist.pose.y, 254, 1.5);

            ninebot_tf::tf_message tf_msg = resList[0];
            ninebot_tf::tf_message tf_msg2 = resList[1];
            if (tf_msg.err==0 && tf_msg2.err==0) {
                StampedPose odom_pos = tfmsgTo2DPose(tf_msg);
                Eigen::Isometry3f depth_pose = PoseToCamcoor(tfmsgToPose(tf_msg2));
                // process depth
                m_p_local_mapping->processDepthFrame(local_depth, odom_pos, depth_pose);
            }
            else {
                ALOGE("localmap: Could not find tf pose with specified depth ts, error code: %d, %d", tf_msg.err, tf_msg2.err);
            }

            // StampedFloat ultrasonic;
            mRawDataInterface->retrieveUltrasonic(raw_ultrasonic);
            m_ultrasonic_buffer.push_back(raw_ultrasonic.value);
            if (m_ultrasonic_buffer.size() > 3) {
                m_ultrasonic_buffer.pop_front();
            }
            float ultrasonic_sum = 0;
            for (auto ultrasonic_element : m_ultrasonic_buffer) {
                ultrasonic_sum += ultrasonic_element;
            }
            m_ultrasonic_average = ultrasonic_sum / m_ultrasonic_buffer.size(); 

            m_p_local_mapping->getDepthMapWithFrontUltrasonic(m_local_map, false, tfmsgTo2DPose(resList[0]), m_ultrasonic_average);

            return true;
        }

        void AlgoFollow::trackHead(const float target_theta_head) {
            float w = 0.0f;
            float head_pitch_speed = 0.0f;
            float head_yaw_speed = 0.0f;
            mRawDataInterface->retrieveHeadPos(raw_headpos);
            
            // yaw 
            if (m_p_head_yaw_tracker){
                if (m_is_detected)
                    head_yaw_speed = m_p_head_yaw_tracker->calculate(target_theta_head, raw_headpos.yaw);
                else
                    head_yaw_speed = m_p_head_yaw_tracker->calculate(raw_headpos.yaw, raw_headpos.yaw);
            }

            // yaw 
            if (m_p_head_pitch_tracker)
                head_pitch_speed = m_p_head_pitch_tracker->calculate(0.5f, raw_headpos.pitch);

            // // pitch 
            // const int HEIGHT_COLOR_IMG = 480;
            // if (m_roi_color.y < HEIGHT_COLOR_IMG / 6){
            //     if (raw_headpos.pitch > 1.0f){
            //         head_pitch_speed = -0.1f;
            //     } 
            //     else {
            //         head_pitch_speed = 0.3f;
            //     }
            // } else if (m_roi_color.y > ( HEIGHT_COLOR_IMG / 4)){
            //     if (raw_headpos.pitch < 0.05f){
            //         head_pitch_speed = 0.1f;
            //     } 
            //     else {
            //         head_pitch_speed = -0.3f;
            //     }
            // } 
            // else {
            //     head_pitch_speed = 0.0f;
            // }

            mRawDataInterface->ExecuteHeadMode(1);
            if (m_is_track_head)
                mRawDataInterface->ExecuteHeadSpeed(head_yaw_speed,head_pitch_speed,0);
            else
                mRawDataInterface->ExecuteHeadSpeed(0.0f,head_pitch_speed,0);

            ALOGD("trackHead: target_theta_head = %f, yaw_speed = %f, pitch_speed = %f", target_theta_head, head_yaw_speed, head_pitch_speed);           
        }

        void AlgoFollow::trackVehicle(const float target_distance, const float target_theta) {
            float vehicle_linear_velocity, vehicle_angular_velocity;
            if (m_is_track_vehicle) {
                if (m_is_detected && target_distance > 0) {
                    if (target_distance > 1.5) {
                        vehicle_linear_velocity = m_p_vehicle_tracker->calculate(target_distance - 1.2f, 0.0);
                        vehicle_angular_velocity = vehicle_linear_velocity / fmin(2.0f,target_distance) * target_theta * 2.0f;                    
                    }
                    else {
                        vehicle_linear_velocity = m_p_vehicle_tracker->calculate(0.0, 0.0);
                        vehicle_linear_velocity = 0.0f;
                        if (abs(target_theta) > 0.2) {
                            vehicle_angular_velocity = target_theta * 1.0f;
                        }
                        else {
                            vehicle_angular_velocity = 0.0f;
                        }
                    }
                }
                else {
                    vehicle_linear_velocity = m_p_vehicle_tracker->calculate(0.0, 0.0);
                    vehicle_linear_velocity = 0.0f;
                    vehicle_angular_velocity = 0.0f;
                }
                this->safeControl(vehicle_linear_velocity, vehicle_angular_velocity);
            }
            ALOGD("trackVehicle: target_distance = %.2f, target_theta = %.2f, vehicle_linear_velocity = %.2f, vehicle_angular_velocity=%.2f", target_distance, target_theta, vehicle_linear_velocity, vehicle_angular_velocity);                       
        }

        void AlgoFollow::ExtractTarget(float & target_distance, float & target_theta_wrt_head){
            
            mRawDataInterface->retrieveHeadPos(raw_headpos);
            m_p_depth_processor->setPitch(raw_headpos.pitch);
            ALOGD("ExtractTarget: setPitch = %f", raw_headpos.pitch);

            cv::Mat hist_image;
            bool is_dist_valid = m_p_depth_processor->process(raw_depth, m_roi_color, m_roi_depth, hist_image, target_distance, target_theta_wrt_head);
            // target_theta_wrt_head = -target_theta_wrt_head;

            ALOGD("ExtractTarget: is_dist_valid = %d", is_dist_valid);
            ALOGD("ExtractTarget: target_distance = %f", target_distance);
            ALOGD("ExtractTarget: raw_headpos.yaw = %.2f, target_theta_wrt_head = %.2f, target_theta_wrt_head (manual) = %.2f", raw_headpos.yaw, target_theta_wrt_head, (float) 0.5f - (m_roi_color.x + m_roi_color.width / 2.0f ) / 640.0f );

            // target_theta_wrt_head = (float) m_roi_color.x / 640.f - 0.5f;
        }

        void AlgoFollow::renderDisplay() {
            /*! Draw the result to canvas */
            canvas.setTo(240);
            if(imgstream_en){
                if(!raw_color.image.empty()){
                    cv::Mat show_color;
                    const float resize_show_color = 0.5;
                    cv::resize(raw_color.image, show_color, cv::Size(), resize_show_color, resize_show_color); 
                    if (m_is_detected)
                        cv::rectangle(show_color, cv::Rect(int(m_roi_color.x * resize_show_color), int(m_roi_color.y * resize_show_color), int(m_roi_color.width * resize_show_color), int(m_roi_color.height * resize_show_color)), Scalar(0, 0, 0), 10);
                    cv::Mat flip_color;               // dst must be a different Mat
                    cv::flip(show_color, flip_color, 1);     // because you can't flip in-place (leads to segfault)
                    cv::Mat ca1 = canvas(cv::Rect(0, 0, flip_color.cols, flip_color.rows));
                    flip_color.copyTo(ca1);
                }
                if(!raw_depth.image.empty()){
                    cv::Mat show_depth, rescale_depth;
                    const float resize_show_depth = 1.0;
                    cv::resize(raw_depth.image, rescale_depth, cv::Size(), resize_show_depth, resize_show_depth); 
                    rescale_depth /= 10;
                    rescale_depth.convertTo(show_depth, CV_8U);
                    applyColorMap(show_depth, show_depth, cv::COLORMAP_JET);
                    if (m_is_detected)
                        cv::rectangle(show_depth, cv::Rect(int(m_roi_depth.x * resize_show_depth), int(m_roi_depth.y * resize_show_depth), int(m_roi_depth.width * resize_show_depth), int(m_roi_depth.height * resize_show_depth)), Scalar(0, 0, 0), 10);
                    ALOGD("m_roi_depth: (%f,%f,%f,%f)", m_roi_depth.x, m_roi_depth.y, m_roi_depth.width, m_roi_depth.height);
                    cv::Mat flip_depth;                         // dst must be a different Mat
                    cv::flip(show_depth, flip_depth, 1);        // because you can't flip in-place (leads to segfault)
                    cv::Mat ca2 = canvas(cv::Rect(320, 0, flip_depth.cols, flip_depth.rows));
                    flip_depth.copyTo(ca2);
                } 

                string contents;
                if (m_is_track_head){
                    contents = "Head: On"; 
                }
                else {
                    contents = "Head: Off"; 
                }
                putText(canvas, contents, cv::Point(0, 300), CV_FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0,0,0), 2);

                if (m_is_track_vehicle){
                    contents = "Wheel: On"; 
                }
                else {
                    contents = "Wheel: Off"; 
                }
                putText(canvas, contents, cv::Point(0, 330), CV_FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0,0,0), 2);

                if (m_target_distance > 0) {
                    contents = "D: " + ToString(m_target_distance);
                    putText(canvas, contents, cv::Point(200, 300), CV_FONT_HERSHEY_COMPLEX, 1.5, cv::Scalar(0,0,0), 2);
                    contents = "A: " + ToString(m_target_theta/3.14*180.0);
                    putText(canvas, contents, cv::Point(400, 300), CV_FONT_HERSHEY_COMPLEX, 1.5, cv::Scalar(0,0,0), 2);
                }

            }
        }

        bool AlgoFollow::loadConfig(float & head_kp, float & head_ki, float & head_kd, float & vehicle_kp, float & vehicle_ki, float & vehicle_kd)
        {
            head_kp = 1.5f;
            head_ki = 0.10f;
            head_kd = 0.01f;
            vehicle_kp = 0.50f;
            vehicle_ki = 0.01f;
            vehicle_kd = 0.01f;
            m_down_scale = 8;
            PARAM_SETTING param_setting = (PARAM_SETTING)0;
            std::ifstream ifconfig;
            std::string line;
            std::string filename = "/sdcard/follow.cfg";
            ifconfig.open(filename);
            if (!ifconfig.is_open())
            {
                ALOGD("failed to open config file '%s'", filename.c_str());
                return false;
            }

            ALOGD("opened config file '%s'", filename.c_str());

            int idx_config = 0;
            while (std::getline(ifconfig, line)) {
                std::istringstream iss(line);
                std::string parameter;
                while (iss >> parameter)
                {
                    char c = parameter[0];
                    if (c == '#')
                        break;

                    const char *_Ptr = parameter.c_str();
                    char *_Eptr;
                    float param_value = strtod(_Ptr, &_Eptr);

                    switch (param_setting)
                    {
                    case HEAD_KP:
                        ALOGD("config loaded, HEAD_KP = %f", param_value);
                        head_kp = param_value;
                        break;
                    case HEAD_KI:
                        ALOGD("config loaded, HEAD_KI = %f", param_value);
                        head_ki = param_value;
                        break;
                    case HEAD_KD:
                        ALOGD("config loaded, HEAD_KD = %f", param_value);
                        head_kd = param_value;
                        break;
                    case VEHICLE_KP:
                        ALOGD("config loaded, VEHICLE_KD = %f", param_value);
                        vehicle_kp = param_value;
                        break;
                    case VEHICLE_KI:
                        ALOGD("config loaded, VEHICLE_KD = %f", param_value);
                        vehicle_ki = param_value;
                        break;
                    case VEHICLE_KD:
                        ALOGD("config loaded, VEHICLE_KD = %f", param_value);
                        vehicle_kd = param_value;
                        break;       
                    case IMG_DOWNSCALE:
                        ALOGD("config loaded, m_down_scale = %f", param_value);
                        m_down_scale = int(param_value);
                        break;
                    }
                }
                idx_config++;
                param_setting = (PARAM_SETTING)idx_config;
            }

            ALOGD("config: head_kp = %.2f, head_ki = %.2f, head_kd = %.2f", head_kp, head_ki, head_kd);

            return true;
        }

	} // namespace follow_algo
} // namespace ninebot_algo

