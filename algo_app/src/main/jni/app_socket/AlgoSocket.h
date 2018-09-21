#ifndef NINEBOT_ALGO_SOCKET_H
#define NINEBOT_ALGO_SOCKET_H

#include <fstream>
#include "AlgoBase.h"
#include "SocketServer.h"
#include "LocalMapping.h"

namespace ninebot_algo
{
	/*! namespace of this algorithm  */
	namespace socket_algo
	{

		/*! Define the configurable parameters for this algorithm  */
		typedef struct test_params
		{
			//char* test_config_filename;
			// ........................................

		} test_params_t;

		/*! class of this algorithm, derived from base class AlgoBase */
		class AlgoSocket : public AlgoBase {
		public:
            /*! Constructor of AlgoSocket
                 * @param rawInterface The pointer to the RawData object instantiated in main()
                 * @param run_sleep_ms The sleep time in millisecond for AlgoBase::run() - AlgoBase::step() is executed following a sleep in run()
            * @param isRender The bool switch to select whether or not do rendering work in step()
        	*/
			AlgoSocket(RawData* rawInterface, int run_sleep_ms, bool isRender=true):
				AlgoBase(rawInterface,run_sleep_ms,true){
				m_is_init_succed = false;
				m_ptime = 10;
				m_isRender = isRender;
                pose_isRecording = false;
				canvas = cv::Mat::zeros( cv::Size(640, 360), CV_8UC3 );
				tfTestMode = 0;
				m_is_stop = false;
				m_slide_event = 0;
				step_count = 0;
				motion_test = 0;
				motion_sign = 1;
				scan_round = 0;
                m_p_local_mapping = NULL;
			}
			~AlgoSocket();


			/*! Implement a complete flow of this algorithm, takes all inputs, run once, and output */
			virtual bool step();

			/*! Initialize current algorithm with configurable parameters */
			bool init(test_params_t cParams);
			/*! Return the runtime of step() in milliseconds, shall implement the calculation of this number in step(). This function is called by main(). */
			float runTime();
			/*! Copy the internal drawing content to the external display buffer. This function is called by main(), the internal canvas is maintained and updated in step() according to user's demands */
			bool showScreen(void* pixels);

            void startPoseRecord(std::string save_folder, int64_t save_time);
            void stopPoseRecord();

            void onEmergencyStop(bool is_stop);
            void onWheelSide(int event);

			// imu callback
			virtual void UpdateAccel(MTPoint val);
			virtual void UpdateGyro(MTPoint val);
			/*! enable sync test*/
			void toggleImgStream();
			void startMotionTest();
            // VLS test
            void setVLSopen(bool en);
            void changeTfTestMode();
            int tfTestMode;
		private:
			/*! Copy internal canvas to intermediate buffer mDisplayIm */
			void setDisplayData();

            int step_count;
			int motion_test;
			int motion_sign;
            bool m_is_stop;
            int m_slide_event;

            bool pose_isRecording;
            int nStep;
			bool m_is_init_succed;
			// class variables
			StampedMat raw_fisheye, raw_depth, raw_color, raw_colords4;
			StampedOrientation raw_orientation;
			StampedIr raw_ir;
			StampedHeadPos raw_headpos;
			StampedHeadPos raw_maincampos;
			StampedBasePos raw_basepos;
			StampedFloat raw_ultrasonic;
			StampedTwist raw_odometry;
			StampedBaseWheelInfo raw_wheel_info;
			RawCameraIntrinsics raw_camerapara;
			CalibrationInfoDS4T calib;
			MTPoint raw_gyro;
			MTPoint raw_accel;
			int64_t t_old;
			bool m_isRender;
			std::mutex mMutexDisplay;
			cv::Mat canvas, mDisplayIm, mDisplayData;
			std::mutex mMutexTimer;
			float m_ptime;
			bool imgstream_en;
			int64_t m_timestamp_start;
			// algo parameters
          
			SocketServer* m_p_server; //(true, true, 8081);
			void stepServer();
			std::string m_folder_socket;
			std::ofstream m_state_file;
        	void createFolder(std::string new_folder_name);

        	bool initLocalMapping();
        	bool prepare_localmap_and_pose_for_controller_g1();
			RawData* m_p_localmapping_rawdata;
            int m_map_width, m_map_height;
            ninebot_algo::local_mapping::LocalMapping *m_p_local_mapping;
            cv::Mat m_local_map, m_persons_map;
            std::vector<std::pair<float, float>> m_persons;
            bool detectPerson();
            void add_show_fov(float cur_angle, float fov_angle, cv::Mat &show_img, float radius_per_meter);
			cv::Mat map_to_show(const cv::Mat & map);
			bool findFront(const cv::Mat & map, const std::pair<int,int> & proposal, std::pair<int,int> & result, int region = 3);
		
			void scanHead(float pitch, float yaw_limit_degree);
			int scan_round;
		};

	} // namespace socket_algo
} // namespace ninebot_algo

#endif