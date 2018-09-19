#include "AlgoSocket.h"
#include "ninebot_log.h"
#include "AlgoUtils.h"
#include "Quaternion.h"
#include "Vector3.h"

#include "SocketServer.h"
#define PI 3.1415

namespace ninebot_algo
{
	namespace socket_algo
	{
		using namespace std;
		using namespace cv;

		AlgoSocket::~AlgoSocket() {
            if(m_p_local_mapping) {
                delete m_p_local_mapping;
                m_p_local_mapping = NULL;
            }

            if(m_p_localmapping_rawdata != NULL) {
                delete m_p_localmapping_rawdata;
                m_p_localmapping_rawdata = NULL;
            }

            delete m_p_server;
		}

		bool AlgoSocket::init(test_params_t cParams)
		{
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

			return m_is_init_succed;
		}


		void AlgoSocket::UpdateAccel(MTPoint val)
		{
			raw_accel = val;
		}

		void AlgoSocket::UpdateGyro(MTPoint val)
		{
			raw_gyro = val;
		}

		void AlgoSocket::toggleImgStream()
		{
			imgstream_en = !imgstream_en;
		}

		void AlgoSocket::setVLSopen(bool en)
        {
            if(en)
                mRawDataInterface->startVLS(false);
            else
                mRawDataInterface->stopVLS();
        }

        void AlgoSocket::startPoseRecord(std::string save_folder, int64_t save_time)
        {
            pose_isRecording = true;
            nStep = 0;
            m_folder_socket = "/sdcard/socket/";
            createFolder(m_folder_socket);        
            m_state_file.open(m_folder_socket + "state.txt");    
        }

        void AlgoSocket::stopPoseRecord()
        {
            pose_isRecording = false;
            m_state_file.close();
        }

        void AlgoSocket::onEmergencyStop(bool is_stop)
        {
            m_is_stop = is_stop;
        }

        void AlgoSocket::onWheelSide(int event)
        {
            m_slide_event = event;
        }

        void AlgoSocket::startMotionTest()
        {
            motion_test = 1;
            motion_sign = -motion_sign;
        }

		bool AlgoSocket::step()
		{
			/*! Get start timestamp for calculating algorithm runtime */
			auto start = std::chrono::high_resolution_clock::now();

			if(!m_is_init_succed){
				ALOGD("AlgoSocket: init false");
				return false;
			}

            // if(motion_test>0 && ++motion_test<150){
            //     mRawDataInterface->ExecuteCmd(0, motion_sign*0.4, mRawDataInterface->getCurrentTimestampSys());
            // }
            // else{
            //     motion_test=0;
            //     mRawDataInterface->ExecuteCmd(0, 0, mRawDataInterface->getCurrentTimestampSys());
            // }

            if(imgstream_en){
                mRawDataInterface->retrieveDepth(raw_depth, false);
                if(raw_depth.timestampSys==0)
                {
                    ALOGD("AlgoSocket: depth wrong");
                    return false;
                }
                // check if depth is duplicate
                if(t_old==raw_depth.timestampSys)
                {
                    ALOGD("fisheye duplicate: %lld",raw_depth.timestampSys/1000);
                    return true;
                }
                else
                {
                    t_old=raw_depth.timestampSys;
                }         

                /*! Get raw sensor data from RawData instance */
                // mRawDataInterface->retrieveFisheye(raw_fisheye, false);
                // if(raw_fisheye.timestampSys==0)
                // {
                //     ALOGD("AlgoSocket: fisheye wrong");
                //     return false;
                // }

                // // check if fisheye is duplicate
                // if(t_old==raw_fisheye.timestampSys)
                // {
                //     ALOGD("fisheye duplicate: %lld",raw_fisheye.timestampSys/1000);
                //     return true;
                // }
                // else
                // {
                //     t_old=raw_fisheye.timestampSys;
                // }       
            }

            mRawDataInterface->retrieveOdometry(raw_odometry, -1);
            prepare_localmap_and_pose_for_controller_g1();
            detectPerson();

            /*! **********************************************************************
             * **** Computation on remote PC 
             * ********************************************************************* */         
            this->stepServer();


			/*! **********************************************************************
			 * **** Processing the algorithm with all input and sensor data **********
			 * ********************************************************************* */
			 string contents;

			 if(m_isRender)
			 {
				/*! Draw the result to canvas */
				cv::Mat tfisheyeS, tfisheye;
				canvas.setTo(240);

				if(imgstream_en){
                    // cv::Mat ca2 = canvas(Rect(160,0,160,120));
                    // resize(raw_fisheye.image,tfisheyeS,Size(),0.25,0.25);
                    // cvtColor(tfisheyeS,tfisheye,CV_GRAY2RGB);
                    // tfisheye.copyTo(ca2);

                    cv::Mat tdepth = raw_depth.image / 10;
                    cv::Mat tdepth8, tdepthup, tdepth8color;
                    tdepth.convertTo(tdepth8, CV_8U);
                    resize(tdepth8, tdepthup, Size(), 0.75, 0.75);
                    cv::Mat ca1 = canvas(Rect(0, 0, 240, 180));
                    applyColorMap(tdepthup, tdepth8color, cv::COLORMAP_JET);
                    tdepth8color.copyTo(ca1);

                    if(!m_local_map.empty()){
                        cv::Mat show = map_to_show(m_local_map);
                        for (auto person : m_persons) {
                            cv::rectangle(show, cv::Point2f(person.first - 5, person.second - 5), cv::Point2f(person.first + 5, person.second + 5), cv::Scalar(255, 0, 0));
                        }
                        cv::resize(show, show, cv::Size(360,360)); 
                        add_show_fov(raw_odometry.twist.pose.orientation, 1.0, show, 0.05f);
                        ALOGD("show: size =(%zu,%zu)", show.rows, show.cols);
                        cv::Mat ca2 = canvas(cv::Rect(240, 0, show.cols, show.rows));
                        show.copyTo(ca2);
                    }

                    int type = raw_depth.image.type();
                    string r;
                    {
                      uchar depth = type & CV_MAT_DEPTH_MASK;
                      uchar chans = 1 + (type >> CV_CN_SHIFT);
                      switch ( depth ) {
                        case CV_8U:  r = "8U"; break;
                        case CV_8S:  r = "8S"; break;
                        case CV_16U: r = "16U"; break;
                        case CV_16S: r = "16S"; break;
                        case CV_32S: r = "32S"; break;
                        case CV_32F: r = "32F"; break;
                        case CV_64F: r = "64F"; break;
                        default:     r = "User"; break;
                      }
                      r += "C";
                      r += (chans+'0');
                    }            

                    contents = "depth: " + ToString((raw_depth.timestampSys - m_timestamp_start) / 1e6) + "(s)";
                    putText(canvas, contents, cv::Point(1, 280), CV_FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0,0,255),1);
                }

                // mRawDataInterface->retrieveHeadPos(raw_headpos);
                // contents = "headpos:" + ToString(raw_headpos.timestampYaw/1000) + ", yaw:" + ToString(raw_headpos.yaw*180/3.14159) + ", pitch:" + ToString(raw_headpos.pitch*180/3.14159) + ", roll:" + ToString(raw_headpos.roll*180/3.14159);
                // putText(canvas, contents, cv::Point(1, 170), CV_FONT_HERSHEY_COMPLEX, 0.39, cv::Scalar(0,0,0),1);

				contents = "odom: " + ToString((raw_odometry.timestamp - m_timestamp_start)/1e6) + "(s), x: " + ToString(raw_odometry.twist.pose.x) + ", y: " + ToString(raw_odometry.twist.pose.y)+ ", o: " + ToString(raw_odometry.twist.pose.orientation);
				putText(canvas, contents, cv::Point(1, 320), CV_FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0,0,0),1);

				// StampedVelocity velocity;
    //             mRawDataInterface->retrieveBaseVelocity(velocity);
    //             ALOGD("odometry delta: v0:%f, v:%f, w0:%f, w:%f", velocity.vel.linear_velocity, raw_odometry.twist.velocity.linear_velocity, velocity.vel.angular_velocity, raw_odometry.twist.velocity.angular_velocity);

				/*! Copy internal canvas to intermediate buffer mDisplayIm */
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

        void AlgoSocket::stepServer() {

            if (raw_depth.image.empty()) {
                ALOGW("empty depth");
                return;
            }

            if (pose_isRecording) {
                string filename_depth = m_folder_socket + "depth_" + ToString(nStep) + ".png";
                cv::imwrite(filename_depth, raw_depth.image);
                string filename_map = m_folder_socket + "map_" + ToString(nStep) + ".png";
                cv::imwrite(filename_map, m_local_map);
                m_state_file << raw_odometry.twist.pose.x << " " << raw_odometry.twist.pose.y << " " << raw_odometry.twist.pose.orientation << std::endl;            
            }

            if (m_p_server->isStopped()) {
                ALOGW("server stopped");
                return;
            }

            if (!m_p_server->isConnected()) {
                ALOGW("server disconnected");
                return;
            }

            // Float 
            float floats_send[14];
            memset (floats_send,0,sizeof(floats_send));
            floats_send[0] = raw_odometry.twist.pose.x;
            floats_send[1] = raw_odometry.twist.pose.y;
            floats_send[2] = raw_odometry.twist.pose.orientation;
            floats_send[3] = (raw_odometry.timestamp - m_timestamp_start)/1e6;

            // m_persons
            ALOGD("send #%d",nStep);
            int num_person = m_persons.size();
            const float resolution = 0.05;
            for (int i = 0; i != num_person; i++){
                floats_send[4+i*2] = (m_persons[i].first - (float)m_local_map.cols) * resolution + raw_odometry.twist.pose.x;
                floats_send[5+i*2] = (m_persons[i].second - (float)m_local_map.rows) * resolution + raw_odometry.twist.pose.y;
                // ALOGD("send Person: (%f,%f)", m_persons[i].first, m_persons[i].second);
            }

            int send_info_floats = m_p_server->sendFloats(floats_send, 14);
            if (send_info_floats < 0) {
                ALOGW("server send floats failed");
                return;
            }
            else {
                ALOGD("server send floats: (%.2f,%.2f,%.2f,%.2f)", floats_send[0],floats_send[1],floats_send[2],floats_send[3]);
                ALOGD("server send floats: (%.0f,%.0f)", floats_send[4],floats_send[5]);
                ALOGD("server send floats: (%.0f,%.0f)", floats_send[6],floats_send[7]);
                ALOGD("server send floats: (%.0f,%.0f)", floats_send[8],floats_send[9]);
                ALOGD("server send floats: (%.0f,%.0f)", floats_send[10],floats_send[11]);
                ALOGD("server send floats: (%.0f,%.0f)\n", floats_send[12],floats_send[13]);
                nStep++;
            }

            // Image 
            // cv::Mat1w random_image(3,3);
            // cv::randu(random_image, cv::Scalar(200), cv::Scalar(400));      
            // int send_info_image = m_p_server->sendDepth(raw_depth.image);
            // if (send_info_image < 0) {
            //     ALOGW("send depth failed");
            //     return;
            // }
            // else {
            //     ALOGD("send depth %lld",raw_depth.timestampSys);
            // }

            const int length_recv = 2;
            float* floats_recv = new float[length_recv];
            int recv_info = m_p_server->recvFloats(floats_recv,length_recv);
            if (recv_info>0)
            {
                mRawDataInterface->ExecuteCmd(floats_recv[0], floats_recv[1], mRawDataInterface->getCurrentTimestampSys());
                ALOGD("server recv floats: %.2f, %.2f", floats_recv[0], floats_recv[1]);
            }
            else {
                ALOGW("server failed to receive signal");
                return;
            }            
        }

		float AlgoSocket::runTime()
		{
			std::lock_guard<std::mutex> lock(mMutexTimer);
			return m_ptime;
		}

		bool AlgoSocket::showScreen(void* pixels) // canvas 640x360, RGBA format
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

		void AlgoSocket::setDisplayData()
		{
			std::lock_guard<std::mutex> lock(mMutexDisplay);
			mDisplayIm = canvas.clone();
		}

        /* tf_test_type
           0: world odom -> base 2d
           1: massive test
           2: world odom -> base pose
           3: world odom -> neck center
           4: world odom -> fisheye
           5: world odom -> depth
           6: world odom -> platform
        */
		void AlgoSocket::tf_test(int tf_test_type, cv::Mat& canvas_show)
		{
            string contents;
            switch(tf_test_type){
                case 0:{
                    auto start = std::chrono::high_resolution_clock::now();
                    ninebot_tf::tf_message tf_msg = mRawDataInterface->GetTfBewteenFrames("base_center_ground_frame", "world_odom_frame", -1, 500);
                    auto end = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<double, std::milli> tf_duration = end-start;
                    ALOGD("TFcost %lf,\ttimestamp is %lld", static_cast<double>(tf_duration.count()), (long long int)tf_msg.header.timestamp);

                    float yaw;
                    StampedPose odom_pos;
                    if(tf_msg.err==0){
                        odom_pos = tfmsgTo2DPose(tf_msg);
                        yaw = odom_pos.pose.orientation;
                    }
                    else
                        ALOGE("tfmsg error code: %d", tf_msg.err);

                    contents = ToString(tf_test_type) + "base 2d TF R:" + ToString(tf_msg.header.timestamp/1000) + ", rx:" + ToString(tf_msg.tf_data.rotation.x) + ", ry:" + ToString(tf_msg.tf_data.rotation.y) + ", rz:" + ToString(tf_msg.tf_data.rotation.z) + ", rw:" + ToString(tf_msg.tf_data.rotation.w)+ ", theta:" + ToString(yaw);
                    putText(canvas_show, contents, cv::Point(1, 315), CV_FONT_HERSHEY_COMPLEX, 0.39, cv::Scalar(0,0,0),1);
                    contents = ToString(tf_test_type) + "  TF T: tx:" + ToString(tf_msg.tf_data.translation.x) + ", ty:" + ToString(tf_msg.tf_data.translation.y) + ", tz:" + ToString(tf_msg.tf_data.translation.z);
                    putText(canvas_show, contents, cv::Point(1, 330), CV_FONT_HERSHEY_COMPLEX, 0.39, cv::Scalar(0,0,0),1);

                    break;
                }
                case 1:{
                    auto start = std::chrono::high_resolution_clock::now();
                    ninebot_tf::vector_req_t reqList;
                    ninebot_tf::vector_tf_msg_t resList;
                    ninebot_tf::tf_request_message msg1("world_evio_frame", "base_center_wheel_axis_frame");
                    for(int i = 0; i<100 ; i++){
                        reqList.push_back(msg1);
                    }
                    mRawDataInterface->getMassiveTfData(&reqList,&resList);
                    auto end = std::chrono::high_resolution_clock::now();
                    for(auto tfmsg : resList){
                        ALOGD("%s %s %lld\t %f\t%f\t%f\t",tfmsg.frame_id.c_str(),tfmsg.header.parent_frame_id.c_str(), tfmsg.header.timestamp, tfmsg.tf_data.rotation.x, tfmsg.tf_data.rotation.z, tfmsg.tf_data.translation.x);
                    }
                    std::chrono::duration<double, std::milli> tf_duration = end-start;
                    auto start1 = std::chrono::high_resolution_clock::now();
                    ninebot_tf::tf_message tf_msg = mRawDataInterface->GetTfBewteenFrames("base_center_wheel_axis_frame", "world_odom_frame", -1, 500);
                    auto end1 = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<double, std::milli> tf_duration1 = end1-start1;
                    ALOGD("TFcost \t%lf\t%lf", static_cast<double>(tf_duration.count()), static_cast<double>(tf_duration1.count()));
                    break;
                }
                case 2:{
                    auto start = std::chrono::high_resolution_clock::now();
                    ninebot_tf::tf_message tf_msg = mRawDataInterface->GetTfBewteenFrames("base_center_wheel_axis_frame", "world_odom_frame", -1, 500);
                    auto end = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<double, std::milli> tf_duration = end-start;
                    ALOGD("TFcost %lf,\ttimestamp is %lld", static_cast<double>(tf_duration.count()), (long long int)tf_msg.header.timestamp);

                    float yaw;
                    StampedPose odom_pos;
                    if(tf_msg.err==0){
                       odom_pos = tfmsgTo2DPose(tf_msg);
                       yaw = odom_pos.pose.orientation;
                    }
                    else
                       ALOGE("tfmsg error code: %d", tf_msg.err);

                    contents = ToString(tf_test_type) + "base pose TF R:" + ToString(tf_msg.header.timestamp/1000) + ", rx:" + ToString(tf_msg.tf_data.rotation.x) + ", ry:" + ToString(tf_msg.tf_data.rotation.y) + ", rz:" + ToString(tf_msg.tf_data.rotation.z) + ", rw:" + ToString(tf_msg.tf_data.rotation.w)+ ", theta:" + ToString(yaw);
                    putText(canvas_show, contents, cv::Point(1, 315), CV_FONT_HERSHEY_COMPLEX, 0.39, cv::Scalar(0,0,0),1);
                    contents = "  TF T: tx:" + ToString(tf_msg.tf_data.translation.x) + ", ty:" + ToString(tf_msg.tf_data.translation.y) + ", tz:" + ToString(tf_msg.tf_data.translation.z);
                    putText(canvas_show, contents, cv::Point(1, 330), CV_FONT_HERSHEY_COMPLEX, 0.39, cv::Scalar(0,0,0),1);
                    break;
                }
                case 3:{
                    auto start = std::chrono::high_resolution_clock::now();
                    ninebot_tf::tf_message tf_msg = mRawDataInterface->GetTfBewteenFrames("neck_center_body_internal_frame", "world_odom_frame", -1, 500);
                    auto end = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<double, std::milli> tf_duration = end-start;
                    ALOGD("TFcost %lf,\ttimestamp is %lld", static_cast<double>(tf_duration.count()), (long long int)tf_msg.header.timestamp);

                    float yaw;
                    StampedPose odom_pos;
                    if(tf_msg.err==0){
                       odom_pos = tfmsgTo2DPose(tf_msg);
                       yaw = odom_pos.pose.orientation;
                    }
                    else
                       ALOGE("tfmsg error code: %d", tf_msg.err);

                    contents = ToString(tf_test_type) + "neck center TF R:" + ToString(tf_msg.header.timestamp/1000) + ", rx:" + ToString(tf_msg.tf_data.rotation.x) + ", ry:" + ToString(tf_msg.tf_data.rotation.y) + ", rz:" + ToString(tf_msg.tf_data.rotation.z) + ", rw:" + ToString(tf_msg.tf_data.rotation.w)+ ", theta:" + ToString(yaw);
                    putText(canvas_show, contents, cv::Point(1, 315), CV_FONT_HERSHEY_COMPLEX, 0.39, cv::Scalar(0,0,0),1);
                    contents = "  TF T: tx:" + ToString(tf_msg.tf_data.translation.x) + ", ty:" + ToString(tf_msg.tf_data.translation.y) + ", tz:" + ToString(tf_msg.tf_data.translation.z);
                    putText(canvas_show, contents, cv::Point(1, 330), CV_FONT_HERSHEY_COMPLEX, 0.39, cv::Scalar(0,0,0),1);
                    break;
                }
                case 4:{
                    auto start = std::chrono::high_resolution_clock::now();
                    ninebot_tf::tf_message tf_msg = mRawDataInterface->GetTfBewteenFrames("rsfisheye_center_neck_fix_frame", "world_odom_frame", -1, 500);
                    //ninebot_tf::tf_message tf_msg = mRawDataInterface->GetTfBewteenFrames("rsfisheye_center_neck_fix_frame", "world_evio_frame", -1, 500);
                    auto end = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<double, std::milli> tf_duration = end-start;
                    ALOGD("TFcost %lf,\ttimestamp is %lld", static_cast<double>(tf_duration.count()), (long long int)tf_msg.header.timestamp);

                    float yaw;
                    StampedPose odom_pos;
                    if(tf_msg.err==0){
                       odom_pos = tfmsgTo2DPose(tf_msg);
                       yaw = odom_pos.pose.orientation;
                    }
                    else
                       ALOGE("tfmsg error code: %d", tf_msg.err);

                    contents = ToString(tf_test_type) + "fisheye TF R:" + ToString(tf_msg.header.timestamp/1000) + ", rx:" + ToString(tf_msg.tf_data.rotation.x) + ", ry:" + ToString(tf_msg.tf_data.rotation.y) + ", rz:" + ToString(tf_msg.tf_data.rotation.z) + ", rw:" + ToString(tf_msg.tf_data.rotation.w)+ ", theta:" + ToString(yaw);
                    putText(canvas_show, contents, cv::Point(1, 315), CV_FONT_HERSHEY_COMPLEX, 0.39, cv::Scalar(0,0,0),1);
                    contents = "  TF T: tx:" + ToString(tf_msg.tf_data.translation.x) + ", ty:" + ToString(tf_msg.tf_data.translation.y) + ", tz:" + ToString(tf_msg.tf_data.translation.z);
                    putText(canvas_show, contents, cv::Point(1, 330), CV_FONT_HERSHEY_COMPLEX, 0.39, cv::Scalar(0,0,0),1);
                    break;
                }
                case 5:{
                    auto start = std::chrono::high_resolution_clock::now();
                    ninebot_tf::tf_message tf_msg = mRawDataInterface->GetTfBewteenFrames("rsdepth_center_neck_fix_frame", "world_odom_frame", -1, 500);
                    auto end = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<double, std::milli> tf_duration = end-start;
                    ALOGD("TFcost %lf,\ttimestamp is %lld", static_cast<double>(tf_duration.count()), (long long int)tf_msg.header.timestamp);

                    float yaw;
                    StampedPose odom_pos;
                    if(tf_msg.err==0){
                       odom_pos = tfmsgTo2DPose(tf_msg);
                       yaw = odom_pos.pose.orientation;
                    }
                    else
                       ALOGE("tfmsg error code: %d", tf_msg.err);

                    // odom coord
                    contents = ToString(tf_test_type) + "depth TF R:" + ToString(tf_msg.header.timestamp/1000) + ", rx:" + ToString(tf_msg.tf_data.rotation.x) + ", ry:" + ToString(tf_msg.tf_data.rotation.y) + ", rz:" + ToString(tf_msg.tf_data.rotation.z) + ", rw:" + ToString(tf_msg.tf_data.rotation.w)+ ", theta:" + ToString(yaw);
                    putText(canvas_show, contents, cv::Point(1, 315), CV_FONT_HERSHEY_COMPLEX, 0.39, cv::Scalar(0,0,0),1);
                    contents = "  TF T: tx:" + ToString(tf_msg.tf_data.translation.x) + ", ty:" + ToString(tf_msg.tf_data.translation.y) + ", tz:" + ToString(tf_msg.tf_data.translation.z);
                    putText(canvas_show, contents, cv::Point(1, 330), CV_FONT_HERSHEY_COMPLEX, 0.39, cv::Scalar(0,0,0),1);
                    break;
                 }
                case 6:{
                    auto start = std::chrono::high_resolution_clock::now();
                    ninebot_tf::tf_message tf_msg = mRawDataInterface->GetTfBewteenFrames("platform_center_head_fix_frame", "world_odom_frame", -1, 500);
                    auto end = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<double, std::milli> tf_duration = end-start;
                    ALOGD("TFcost %lf,\ttimestamp is %lld", static_cast<double>(tf_duration.count()), (long long int)tf_msg.header.timestamp);

                    float yaw;
                    StampedPose odom_pos;
                    if(tf_msg.err==0){
                       odom_pos = tfmsgTo2DPose(tf_msg);
                       yaw = odom_pos.pose.orientation;
                    }
                    else
                       ALOGE("tfmsg error code: %d", tf_msg.err);

                    contents = ToString(tf_test_type) + "platformcam TF R:" + ToString(tf_msg.header.timestamp/1000) + ", rx:" + ToString(tf_msg.tf_data.rotation.x) + ", ry:" + ToString(tf_msg.tf_data.rotation.y) + ", rz:" + ToString(tf_msg.tf_data.rotation.z) + ", rw:" + ToString(tf_msg.tf_data.rotation.w)+ ", theta:" + ToString(yaw);
                    putText(canvas_show, contents, cv::Point(1, 315), CV_FONT_HERSHEY_COMPLEX, 0.39, cv::Scalar(0,0,0),1);
                    contents = "  TF T: tx:" + ToString(tf_msg.tf_data.translation.x) + ", ty:" + ToString(tf_msg.tf_data.translation.y) + ", tz:" + ToString(tf_msg.tf_data.translation.z);
                    putText(canvas_show, contents, cv::Point(1, 330), CV_FONT_HERSHEY_COMPLEX, 0.39, cv::Scalar(0,0,0),1);
                    break;
                 }
                 default:
                    break;
            }
		}

        void AlgoSocket::changeTfTestMode(){
            if(RawData::retrieveRobotModel() < 2){
                if(tfTestMode > 6){
                    tfTestMode = 0;
                } else {
                    tfTestMode++;
                }
             } else{
                if(tfTestMode > 2){
                    tfTestMode = 0;
                } else {
                    tfTestMode++;
                }
             }
        }

        void AlgoSocket::createFolder(std::string new_folder_name)
        {
            // std::string cmd_str_rm = "rm -rf \"" + new_folder_name + "\"";
            // system(cmd_str_rm.c_str());  
            // ALOGD("Command %s was executed. ", cmd_str_rm.c_str());          
            std::string cmd_str_mk = "mkdir \"" + new_folder_name + "\"";
            system(cmd_str_mk.c_str());
            ALOGD("Command %s was executed. ", cmd_str_mk.c_str());
        }


        bool AlgoSocket::initLocalMapping()
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

        bool AlgoSocket::prepare_localmap_and_pose_for_controller_g1() {
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
            if(tf_msg.err==0 && tf_msg2.err==0){
                StampedPose odom_pos = tfmsgTo2DPose(tf_msg);
                Eigen::Isometry3f depth_pose = PoseToCamcoor(tfmsgToPose(tf_msg2));
                // process depth
                m_p_local_mapping->processDepthFrame(local_depth, odom_pos, depth_pose);
            }
            else{
                ALOGE("localmap: Could not find tf pose with specified depth ts, error code: %d, %d", tf_msg.err, tf_msg2.err);
            }
            m_p_local_mapping->getDepthMap(m_local_map, false, tfmsgTo2DPose(resList[0]));

            return true;
        }

        bool AlgoSocket::detectPerson() {
            m_persons_map = m_local_map.clone();
            int erosion_size = 1;
            cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                cv::Point(erosion_size, erosion_size));
            cv::dilate(m_persons_map, m_persons_map, element);
            cv::dilate(m_persons_map, m_persons_map, element);
            cv::erode(m_persons_map, m_persons_map, element);
            cv::dilate(m_persons_map, m_persons_map, element);
            cv::dilate(m_persons_map, m_persons_map, element);
            cv::erode(m_persons_map, m_persons_map, element);
            cv::erode(m_persons_map, m_persons_map, element);

            cv::Mat labels;
            cv::Mat stats;
            cv::Mat centroids;
            cv::connectedComponentsWithStats(m_persons_map, labels, stats, centroids, 8, CV_16U);

            // ALOGD("centroids vvv");
            // for (int i = 0; i < centroids.rows; i++)
            // {
            //     std::string line;
            //     for (int j = 0; j < centroids.cols; j++)
            //     {
            //         line += std::to_string((double)centroids.at<double>(i, j)) + " ";
            //     }
            //     ALOGD("centroids: %s",line.c_str());
            // }            
            // ALOGD("centroids ^^^");

            m_persons.clear();
            for (int i = 0; i < stats.rows; i++)
            {
                int x = stats.at<int>(cv::Point(0, i));
                int y = stats.at<int>(cv::Point(1, i));
                int w = stats.at<int>(cv::Point(2, i));
                int h = stats.at<int>(cv::Point(3, i));
                int a = stats.at<int>(cv::Point(4, i));
                if (a < 25 && w < 5 && h < 5) {
                    cv::Scalar color(0, 0, 0);
                    cv::rectangle(m_persons_map, cv::Point2f(x, y), cv::Point2f(x+w, y+h), color);
                }
                else if (a > 1000) {

                }
                else {
                    m_persons.push_back(std::make_pair(x + w/2, y + h/2));
                }
            }

            return true;
        }

        void AlgoSocket::add_show_fov(float cur_angle, float fov_angle, cv::Mat &show_img, float radius_per_meter)
        {
            cv::Point2f center = cv::Point2f(show_img.cols / 2, show_img.rows / 2);
            cv::Point2f left, right;
            float range = 50;
            if (cur_angle > PI)
                cur_angle -= 2 * PI;
            if (cur_angle < -PI)
                cur_angle += 2 * PI;
            left.x = center.x + range * cos(cur_angle - fov_angle / 2);
            left.y = center.y + range * sin(cur_angle - fov_angle / 2);

            right.x = center.x + range * cos(cur_angle + fov_angle / 2);
            right.y = center.y + range * sin(cur_angle + fov_angle / 2);

            cv::line(show_img, center, left, cv::Scalar(0, 0, 255), 1, cv::LINE_4);
            cv::line(show_img, center, right, cv::Scalar(0, 0, 255), 1, cv::LINE_4);
            
            if (radius_per_meter > 0) {
                cv::circle(show_img, center, 1 * radius_per_meter, cv::Scalar(255, 160, 160), 1.5);
                cv::circle(show_img, center, 2 * radius_per_meter, cv::Scalar(255, 160, 160), 1.5);
                cv::circle(show_img, center, 3 * radius_per_meter, cv::Scalar(255, 160, 160), 1.5);
                cv::line(show_img, cv::Point2f(show_img.cols / 2, 0), cv::Point2f(show_img.cols / 2, show_img.rows), cv::Scalar(255, 160, 160), 1.5);
                cv::line(show_img, cv::Point2f(0, show_img.rows / 2), cv::Point2f(show_img.cols, show_img.rows / 2), cv::Scalar(255, 160, 160), 1.5);
            }

            cv::Point2f loomo_left_front, loomo_right_front, loomo_left_rear, loomo_right_rear;
            loomo_left_front.x = center.x + 0.2*cos(cur_angle)*radius_per_meter - 0.3*sin(cur_angle)*radius_per_meter;
            loomo_left_front.y = center.y + 0.2*sin(cur_angle)*radius_per_meter + 0.3*cos(cur_angle)*radius_per_meter;
            loomo_right_front.x = center.x + 0.2*cos(cur_angle)*radius_per_meter + 0.3*sin(cur_angle)*radius_per_meter;
            loomo_right_front.y = center.y + 0.2*sin(cur_angle)*radius_per_meter - 0.3*cos(cur_angle)*radius_per_meter;
            loomo_left_rear.x = center.x - 0.7*cos(cur_angle)*radius_per_meter - 0.2*sin(cur_angle)*radius_per_meter;
            loomo_left_rear.y = center.y - 0.7*sin(cur_angle)*radius_per_meter + 0.2*cos(cur_angle)*radius_per_meter;
            loomo_right_rear.x = center.x - 0.7*cos(cur_angle)*radius_per_meter + 0.2*sin(cur_angle)*radius_per_meter;
            loomo_right_rear.y = center.y - 0.7*sin(cur_angle)*radius_per_meter - 0.2*cos(cur_angle)*radius_per_meter;
            
            cv::line(show_img, loomo_left_front, loomo_right_front, cv::Scalar(100, 200, 0), 2, cv::LINE_4);
            cv::line(show_img, loomo_left_front, loomo_left_rear, cv::Scalar(100, 200, 0), 2, cv::LINE_4);
            cv::line(show_img, loomo_right_rear, loomo_right_front, cv::Scalar(100, 200, 0), 2, cv::LINE_4);
            cv::line(show_img, loomo_left_rear, loomo_right_rear, cv::Scalar(100, 200, 0), 2, cv::LINE_4);
            cv::circle(show_img, loomo_right_front, 1, cv::Scalar(0, 0, 255), 3);

            cv::Point poly[4];
            poly[0] = loomo_left_front;
            poly[1] = loomo_right_front;
            poly[2] = loomo_right_rear;
            poly[3] = loomo_left_rear;
        }        

        cv::Mat AlgoSocket::map_to_show(const cv::Mat & map) {
            cv::Mat show = cv::Mat::zeros(map.rows, map.cols, CV_8UC3);
            for (int i = 0; i < map.rows; i++)
            {
                for (int j = 0; j < map.cols; j++)
                {
                    uchar map_value = map.at<uchar>(i, j);
                    if (map_value == 255)
                    {
                        show.at<cv::Vec3b>(i, j) = cv::Vec3b(127, 255, 127);
                    }
                    else
                    {
                        show.at<cv::Vec3b>(i, j) = cv::Vec3b(255 - 2.5 * map_value, 255 - 2.5 * map_value, 255 - 2.5 * map_value);
                    }
                }
            } 
            return show.clone();   
        }

	} // namespace socket_algo
} // namespace ninebot_algo
