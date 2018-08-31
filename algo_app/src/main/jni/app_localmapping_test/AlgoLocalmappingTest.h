#ifndef NINEBOT_ALGO_LOCALMAPPING_TEST_H
#define NINEBOT_ALGO_LOCALMAPPING_TEST_H

#include "AlgoBase.h"
#include <queue>
#include "LocalMapping.h"

#include "fcntl.h"
#include "termios.h"

namespace ninebot_algo {
	namespace localmapping_test {
		
		/*! class of this algorithm, derived from base class AlgoBase */
		class AlgoLocalmappingTest : public AlgoBase {
		public:

			AlgoLocalmappingTest(RawData* rawInterface, int run_sleep_ms, bool isRender = true):
                    AlgoBase(rawInterface, run_sleep_ms) {
				m_is_render = isRender;
				m_canvas = cv::Mat::zeros(cv::Size(CANVAS_X, CANVAS_Y), CV_8UC3);
				m_canvas.setTo(cv::Scalar(0, 0, 0));
				m_origin_point.x = 0.0;
				m_origin_point.y = 0.0;
				m_pixel_per_meter = MIN_PIXEL_PER_METER;

				scan_round = 0;
                m_p_local_mapping = NULL;
                m_pause_ultrasonic = false;
				m_head_speed = 3.5;

				block_cnt = 0;
				// init
				init();
				initLocalMapping();

			}
			~AlgoLocalmappingTest();

			/*! Implement a complete flow of this algorithm, takes all inputs, run once, and output */
			virtual bool step();

			/*! Return the runtime of step() in milliseconds, shall implement the calculation of this number in step(). This function is called by main(). */
			float runTime();
			/*! Copy the internal drawing content to the external display buffer. This function is called by main(), the internal canvas is maintained and updated in step() according to user's demands */
			bool showScreen(void* pixels);

			void clearMap(int sel);
			std::string getDebugString();

            bool initLocalMapping();
            bool prepare_localmap_and_pose_for_controller_g1();

		private:
			/*! Copy internal canvas to intermediate buffer mDisplayIm */
			void setDisplayData();
			void drawCanvas();
            void drawLocalMapping();
            void add_show_fov(float cur_angle, float fov_angle, cv::Mat &show_img, float radius_per_meter);
            void detect_and_draw_elevators(cv::Mat& show);

			/*! Initialize current algorithm with configurable parameters */
			bool init();

			void resetHeadMode(int smooth_factor);
			void scanHead(float pitch, float yaw_limit_degree);

        private:
			static constexpr int CANVAS_X = 640;
			static constexpr int CANVAS_Y = 360;
			static constexpr int BLANK_PIXEL = 10;
			static constexpr float MIN_PIXEL_PER_METER = 100;
			static constexpr int RAWDATA_BUFFER_SIZE = 10000;
			static constexpr int ROBOT_SIZE = 20;

            int robot_model;

            Eigen::Vector3f m_cur_pose;
            Eigen::Vector2f m_cur_vel;
            int block_cnt;

			float m_head_speed;
			// drawing
			bool m_is_render;
			cv::Point m_prePt, m_curPt;
			std::mutex m_mutex_display;
			cv::Mat m_canvas, m_display_image, m_display_data;
			float m_pixel_per_meter;
			cv::Point2f m_origin_point;
            std::mutex m_mutex_step;
			std::deque<cv::Point3f> m_trace;
            int scan_round;
			// class variables
			std::mutex mMutexTimer;
			float m_ptime;

			// sensor fusion

            StampedMat raw_fisheye, raw_depth, raw_color;
            // localization information
            std::mutex m_mutex_current;
            cv::Mat local_map_for_controller;

            int64_t last_timestamp_globalmap;

            // local mapping
            StampedMat m_raw_depth;
            ninebot_algo::local_mapping::LocalMapping *m_p_local_mapping;
            float m_map_resolution;
            int m_map_width, m_map_height;
			cv::Mat icon;
            float robot_orientation;

            // ultrasonic 
            std::deque<float> m_buf_ultrasonic;
            static const int m_sz_buf_ultrasonic = 3;
            static constexpr float m_thres_ultrasonic = 800;
            bool m_pause_ultrasonic;

			bool last_step_have_obstacle;


            cv::Mat m_show;
			RawData* m_p_localmapping_rawdata;
		};

	} // namespace localmapping_test
} // namespace ninebot_algo

#endif
