#ifndef NINEBOT_ALGO_SOCKET_H
#define NINEBOT_ALGO_SOCKET_H

#include <fstream>
#include "AlgoBase.h"
#include "SocketServer.h"
#include "LocalMapping.h"
#include "PID.h"
#include "DepthPreprocess.h"

namespace ninebot_algo
{
	/*! namespace of this algorithm  */
	namespace follow_algo
	{

		/*! Define the configurable parameters for this algorithm  */
		typedef struct test_params
		{
			//char* test_config_filename;
			// ........................................

		} test_params_t;

		/*! class of this algorithm, derived from base class AlgoBase */
		class AlgoFollow : public AlgoBase {
		public:
            /*! Constructor of AlgoFollow
                 * @param rawInterface The pointer to the RawData object instantiated in main()
                 * @param run_sleep_ms The sleep time in millisecond for AlgoBase::run() - AlgoBase::step() is executed following a sleep in run()
            * @param isRender The bool switch to select whether or not do rendering work in step()
        	*/
			AlgoFollow(RawData* rawInterface, int run_sleep_ms, bool isRender=true);
			~AlgoFollow();


			/*! Implement a complete flow of this algorithm, takes all inputs, run once, and output */
			virtual bool step();

			/*! Initialize current algorithm with configurable parameters */
			bool init();
			/*! Return the runtime of step() in milliseconds, shall implement the calculation of this number in step(). This function is called by main(). */
			float runTime();
			/*! Copy the internal drawing content to the external display buffer. This function is called by main(), the internal canvas is maintained and updated in step() according to user's demands */
			bool showScreen(void* pixels);
			std::string getDebugString();

            void startPoseRecord(std::string save_folder, int64_t save_time);
            void stopPoseRecord();

            void onEmergencyStop(bool is_stop);

			// imu callback
			virtual void UpdateAccel(MTPoint val);
			virtual void UpdateGyro(MTPoint val);
			
			/*! enable sync test*/
			void toggleImgStream();

            // VLS test
            void setVLSopen(bool en);
            
			void switchHeadTracker();
			void switchVehicleTracker();

		private:
			/*! Copy internal canvas to intermediate buffer mDisplayIm */
			void setDisplayData();
			void renderDisplay();

            bool pose_isRecording;
            int nStep;
			bool m_is_init_succed;
			// class variables
			StampedMat raw_fisheye, raw_depth, raw_color, raw_colords4;
			StampedOrientation raw_orientation;
			StampedIr raw_ir;
			StampedHeadPos raw_headpos;
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

			SocketServer* m_p_server; //(true, true, 8081);
			void stepServer();
			std::string m_folder_socket;
			std::ofstream m_state_file;
        	void createFolder(std::string new_folder_name);

        	// local map
        	bool initLocalMapping();
        	bool prepare_localmap_and_pose_for_controller_g1();
			RawData* m_p_localmapping_rawdata;
            int m_map_width, m_map_height;
            ninebot_algo::local_mapping::LocalMapping *m_p_local_mapping;
            cv::Mat m_local_map;

			// safety verification
			void safeControl(float v, float w);
			bool m_safety_control; 
			bool m_is_track_head;
			bool m_is_track_vehicle;
			std::deque<float> m_ultrasonic_buffer;
			float m_ultrasonic_average;

			// tracking
			void trackHead(const float target_theta_head);
            void trackVehicle(const float target_distance, const float target_theta);
			PID *m_p_head_yaw_tracker;
			PID *m_p_head_pitch_tracker;
			PID *m_p_vehicle_tracker;
			int m_direction_head_test;

            DepthPreprocess *m_p_depth_processor;
        	void ExtractTarget(float & target_distance, float & target_theta_wrt_head);
        	cv::Rect m_roi_color;
        	cv::Rect m_roi_depth;        	
        	bool m_is_detected;
        	float* bounding_box;
        	float m_target_distance;
        	float m_target_theta;

			// configuration
			int m_down_scale;
        	bool loadConfig(float & head_kp, float & head_ki, float & head_kd, float & vehicle_kp, float & vehicle_ki, float & vehicle_kd);
			enum PARAM_SETTING
			{
				HEAD_KP,
				HEAD_KI,
				HEAD_KD,
				VEHICLE_KP,
				VEHICLE_KI,
				VEHICLE_KD,
				IMG_DOWNSCALE
			};

		};

	} // namespace follow_algo
} // namespace ninebot_algo

#endif