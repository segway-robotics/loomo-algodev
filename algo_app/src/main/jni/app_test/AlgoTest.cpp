#include "AlgoTest.h"
#include "ninebot_log.h"
#include "AlgoUtils.h"
#include "Quaternion.h"
#include "Vector3.h"

namespace ninebot_algo
{
	namespace test_algo
	{
		using namespace std;
		using namespace cv;

		AlgoTest::~AlgoTest() {
		}

		bool AlgoTest::init(test_params_t cParams)
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

			return m_is_init_succed;
		}


		void AlgoTest::UpdateAccel(MTPoint val)
		{
			raw_accel = val;
		}

		void AlgoTest::UpdateGyro(MTPoint val)
		{
			raw_gyro = val;
		}

		void AlgoTest::toggleImgStream()
		{
			imgstream_en = !imgstream_en;
		}

		void AlgoTest::setVLSopen(bool en)
        {
            if(en)
                mRawDataInterface->startVLS(false);
            else
                mRawDataInterface->stopVLS();
        }

        void AlgoTest::startPoseRecord(std::string save_folder, int64_t save_time)
        {
            pose_isRecording = true;
            nStep = 0;
        }

        void AlgoTest::stopPoseRecord()
        {
            pose_isRecording = false;
        }

        void AlgoTest::onEmergencyStop(bool is_stop)
        {
            m_is_stop = is_stop;
        }

        void AlgoTest::onWheelSide(int event)
        {
            m_slide_event = event;
        }

        void AlgoTest::startMotionTest()
        {
            motion_test = 1;
            motion_sign = -motion_sign;
        }

		bool AlgoTest::step()
		{
			/*! Get start timestamp for calculating algorithm runtime */
			auto start = std::chrono::high_resolution_clock::now();

			if(!m_is_init_succed){
				ALOGD("algotest: init false");
				return false;
			}

            if(motion_test>0 && ++motion_test<150){
                mRawDataInterface->ExecuteCmd(0, motion_sign*0.4, mRawDataInterface->getCurrentTimestampSys());
            }
            else{
                motion_test=0;
                mRawDataInterface->ExecuteCmd(0, 0, mRawDataInterface->getCurrentTimestampSys());
            }

            if(imgstream_en){
                /*! Get raw sensor data from RawData instance */
                mRawDataInterface->retrieveFisheye(raw_fisheye, false);
                if(raw_fisheye.timestampSys==0)
                {
                    ALOGD("algotest: fisheye wrong");
                    return false;
                }

                // check if fisheye is duplicate
                if(t_old==raw_fisheye.timestampSys)
                {
                    ALOGD("fisheye duplicate: %lld",raw_fisheye.timestampSys/1000);
                    return true;
                }
                else
                {
                    t_old=raw_fisheye.timestampSys;
                }

                mRawDataInterface->retrieveDepth(raw_depth, false);
                if(raw_depth.timestampSys==0)
                {
                    ALOGD("algotest: depth wrong");
                    return false;
                }
                mRawDataInterface->retrieveDS4Color(raw_colords4, false);
                if(raw_colords4.timestampSys==0)
                {
                    ALOGD("algotest: ds4 color wrong");
                    //return false;
                }

                mRawDataInterface->retrieveColor(raw_color, false);
                if(raw_color.timestampSys==0)
                {
                    ALOGD("algotest: color wrong");
                    return false;
                }
            }

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
                    cv::Mat ca2 = canvas(Rect(160,0,160,120));
                    resize(raw_fisheye.image,tfisheyeS,Size(),0.25,0.25);
                    cvtColor(tfisheyeS,tfisheye,CV_GRAY2RGB);
                    tfisheye.copyTo(ca2);

                    if(raw_colords4.timestampSys!=0){
                        cv::Mat tcolords4;
                        cv::Mat cacolords4 = canvas(Rect(320, 0, 160, 120));
                        if(raw_colords4.image.rows==240)
                           resize(raw_colords4.image, tcolords4, Size(), 0.5, 0.5);
                        else
                           resize(raw_colords4.image, tcolords4, Size(), 0.25, 0.25);
                        tcolords4.copyTo(cacolords4);
                    }
                    contents = "depth:" + ToString(raw_depth.timestampSys/1000) + ", fisheye:" + ToString(raw_fisheye.timestampSys/1000) + ", color:" + ToString(raw_color.timestampSys/1000) + ", ds4color:" + ToString(raw_colords4.timestampSys/1000);
                    putText(canvas, contents, cv::Point(1, 140), CV_FONT_HERSHEY_COMPLEX, 0.39, cv::Scalar(0,0,255),1);

                    cv::Mat tcolor;
                    cv::Mat cacolor = canvas(Rect(480,0,160,120));
                    resize(raw_color.image,tcolor,Size(),0.25,0.25);
                    tcolor.copyTo(cacolor);

                    cv::Mat tdepth = raw_depth.image / 10;
                    cv::Mat tdepth8, tdepthup, tdepth8color;
                    tdepth.convertTo(tdepth8, CV_8U);
                    resize(tdepth8, tdepthup, Size(), 0.5, 0.5);
                    cv::Mat ca1 = canvas(Rect(0, 0, 160, 120));
                    applyColorMap(tdepthup, tdepth8color, cv::COLORMAP_JET);
                    tdepth8color.copyTo(ca1);
                }

                mRawDataInterface->retrieveGOrientation(raw_orientation, raw_depth.timestampSys);
                contents = "orient:" + ToString(raw_orientation.timestamp/1000) + ", roll:" + ToString(raw_orientation.orientation.roll*180/3.14159) + ", pitch:" + ToString(raw_orientation.orientation.pitch*180/3.14159) + ", yaw:" + ToString(raw_orientation.orientation.yaw*180/3.14159);
                putText(canvas, contents, cv::Point(1, 155), CV_FONT_HERSHEY_COMPLEX, 0.39, cv::Scalar(0,0,255),1);

                mRawDataInterface->retrieveHeadPos(raw_headpos);
                contents = "headpos:" + ToString(raw_headpos.timestampYaw/1000) + ", yaw:" + ToString(raw_headpos.yaw*180/3.14159) + ", pitch:" + ToString(raw_headpos.pitch*180/3.14159) + ", roll:" + ToString(raw_headpos.roll*180/3.14159);
                putText(canvas, contents, cv::Point(1, 170), CV_FONT_HERSHEY_COMPLEX, 0.39, cv::Scalar(0,0,0),1);

                 mRawDataInterface->retrieveMaincamPos(raw_maincampos);
                 contents = "maincampos:" + ToString(raw_maincampos.timestampYaw/1000) + ", yaw:" + ToString(raw_maincampos.yaw*180/3.14159) + ", pitch:" + ToString(raw_maincampos.pitch*180/3.14159) + ", roll:" + ToString(raw_maincampos.roll*180/3.14159);
                 putText(canvas, contents, cv::Point(1, 185), CV_FONT_HERSHEY_COMPLEX, 0.39, cv::Scalar(0,0,0),1);

                 //tf test related sample.
                 tf_test(tfTestMode, canvas);


				 mRawDataInterface->retrieveIRsensor(raw_ir);
				 mRawDataInterface->retrieveBaseWheelInfo(raw_wheel_info);
				 contents =  "bodyir:" + ToString(raw_ir.timestamp/1000) + ", L:" + ToString(raw_ir.leftir,0) + ", R:" + ToString(raw_ir.rightir,0) + ", wheelsp:" + ToString(raw_ir.timestamp/1000) + ", L:" + ToString(raw_wheel_info.leftSpeed) + ", R:" + ToString(raw_wheel_info.rightSpeed);
				 if(raw_ir.leftir>2000 || raw_ir.rightir>2000 || raw_ir.leftir==0 || raw_ir.rightir==0)
                     putText(canvas, contents, cv::Point(1, 215), CV_FONT_HERSHEY_COMPLEX, 0.39, cv::Scalar(0,0,255),1);
                 else
                     putText(canvas, contents, cv::Point(1, 215), CV_FONT_HERSHEY_COMPLEX, 0.39, cv::Scalar(0,0,0),1);

				contents = "cur systime:" + ToString(mRawDataInterface->getCurrentTimestampSys()/1000) + ", errorcode:" + ToString(mRawDataInterface->retrieveDeviceErrorCode()) + ", mileage:" + ToString(mRawDataInterface->getCartMile()) + ", stop:" + ToString(m_is_stop) + ", slide:" + ToString(m_slide_event);
				putText(canvas, contents, cv::Point(1, 232), CV_FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,0,0),1);

				mRawDataInterface->retrieveOdometry(raw_odometry, -1);
				contents = "odom:" + ToString(raw_odometry.timestamp/1000) + ", x:" + ToString(raw_odometry.twist.pose.x) + ", y:" + ToString(raw_odometry.twist.pose.y)+ ", o:" + ToString(raw_odometry.twist.pose.orientation) + ", lin:" + ToString(raw_odometry.twist.velocity.linear_velocity) + ", ang:" + ToString(raw_odometry.twist.velocity.angular_velocity)+ ", L:" + ToString(raw_odometry.twist.left_ticks) + ", R:" + ToString(raw_odometry.twist.right_ticks);
				putText(canvas, contents, cv::Point(1, 250), CV_FONT_HERSHEY_COMPLEX, 0.39, cv::Scalar(0,0,0),1);

				StampedVelocity velocity;
                mRawDataInterface->retrieveBaseVelocity(velocity);
                ALOGD("odometry delta: v0:%f, v:%f, w0:%f, w:%f", velocity.vel.linear_velocity, raw_odometry.twist.velocity.linear_velocity, velocity.vel.angular_velocity, raw_odometry.twist.velocity.angular_velocity);

                mRawDataInterface->retrieveBasePos(raw_basepos);
				contents = "basepos:" + ToString(raw_basepos.timestamp/1000) + ", pitch:" + ToString(raw_basepos.pitch*180/3.14159) + ", roll:" + ToString(raw_basepos.roll*180/3.14159)+ ", yaw:" + ToString(raw_basepos.yaw*180/3.14159);
				putText(canvas, contents, cv::Point(1, 265), CV_FONT_HERSHEY_COMPLEX, 0.39, cv::Scalar(0,0,0),1);
				mRawDataInterface->retrieveUltrasonic(raw_ultrasonic);
				contents = "exposure fisheye: " + ToString(raw_fisheye.exposure) + ", depth:" + ToString(raw_depth.exposure) + ", imu temp:" + ToString(mRawDataInterface->retrieveIMUTemperature()) + ", ultras:" + ToString(raw_ultrasonic.timestamp/1000) + ", v:" + ToString(raw_ultrasonic.value,0);
				putText(canvas, contents, cv::Point(1, 280), CV_FONT_HERSHEY_COMPLEX, 0.39, cv::Scalar(0,0,0),1);
				contents = "gyro:" + ToString(raw_gyro.ts/1000) + ", x:" + ToString(raw_gyro.x) + ", y:" + ToString(raw_gyro.y) + ", z:" + ToString(raw_gyro.z) + "; accel:" + ToString(raw_accel.ts/1000) + ", x:" + ToString(raw_accel.x) + ", y:" + ToString(raw_accel.y) + ", z:" + ToString(raw_accel.z);
				putText(canvas, contents, cv::Point(1, 295), CV_FONT_HERSHEY_COMPLEX, 0.39, cv::Scalar(0,0,0),1);

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

		float AlgoTest::runTime()
		{
			std::lock_guard<std::mutex> lock(mMutexTimer);
			return m_ptime;
		}

		bool AlgoTest::showScreen(void* pixels) // canvas 640x360, RGBA format
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

		void AlgoTest::setDisplayData()
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
		void AlgoTest::tf_test(int tf_test_type, cv::Mat& canvas_show)
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

        void AlgoTest::changeTfTestMode(){
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
	} // namespace test_algo
} // namespace ninebot_algo
