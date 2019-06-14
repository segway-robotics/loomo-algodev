//
// Created by bob on 2016/6/30.
//

#ifndef DEPTH_PREPROCESS_H
#define DEPTH_PREPROCESS_H

//#include "../interface/RawDataUtil.h"
#include "RawData.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <fstream>
#include <list>
using namespace std;
/*
 * this class implementation the iteration algorithms to
 * estimate the depth of the rect drawn on the color camera image.
 * basic idea: initially suppose a depth value, then iterate to depth image to get the real depth.
 *
 * */

namespace ninebot_algo {
      namespace follow_algo {

            #define DEGREE_STEP 2.5
            #define MAX_DEGREE 47.5
            #define MAT_SIZE 9
            #define TABLE_SIZE int(MAT_SIZE * MAX_DEGREE / DEGREE_STEP)

            class DepthPreprocess
            {
            public:
                  /*!Construction */
                  DepthPreprocess();
                  /*!Deconstruction */
                  ~DepthPreprocess();

                  /*! according to the rect in color image to calculate distance and angle
                   * @param stperson_rect, input,the rect from tracker
                   * @param raw_depth, input,the raw depth image
                   * @param depth_rect, output,the rect on depth image
                   * @param histImage, output, his image for display
                   * @param distance, output,distance to target
                   * @param theta, output,theta to target
                  */

                  //edit cp:if dist is valid, return true;else return false;
                  //void process(const StampedMat &raw_depth,const cv::Rect &stperson_rect,cv::Rect &depth_rect,cv::Mat &histImage,float &distance, float &theta);
                  bool process(const StampedMat &raw_depth, const cv::Rect &stperson_rect, cv::Rect &depth_rect, cv::Mat &histImage, float &distance, float &theta);

                  float pitch_offset;
                  float  _curPitch;
                  list<float> _lastSomeFrameDist;

                  void setPitch(float pitchAngle)
                  {
                        _curPitch = pitchAngle;
                  };

            private:

                  cv::Mat _depth;

                  /*! set the input color rect
                   * @param stperson_rect, input,the rect from tracker
                  */
                  void setColorRect(const cv::Rect &stperson_rect);
                  /*! calculate the hist of the roi of the depth image
                   * @param raw_depth, input
                   * @param depth_rect,input, the rect on depth image
                   * @param hist,output
                   * @param histImage,output, the image to show
                   * @param target_val,output, the distance of the target roi
                  */
                  bool CalHist(const StampedMat &raw_depth, const cv::Rect &depth_rect, cv::Mat &hist, cv::Mat &histImage, float &target_val);
                  /*! calculate target roi of the depth image
                   * @param roi, input
                   * @param channel,input, the desired channel
                   * @param res, output
                   */
                  bool CalHistTarget(const cv::Mat &roi, const int channel, float &res);
                  /*! calculate target theta
                  * @param img_size, input, depth image size
                  * @param rect,input, roi rect
                  * @param dist, input, distance to the roi
                  * @param theta, output, angle to the roi
                  */
                  void CalTargetTheta(const cv::Size &img_size, const cv::Rect &rect, const float dist, float& theta);
                  struct RawCameraIntrinsics {
                        float focalLengthX;
                        float focalLengthY;
                        float pricipalPointX;
                        float pricipalPointY;
                        float distortion[5];
                  };

                  std::vector<cv::Point3d> point_depthcamera_; /*< point depth camera vector*/
                  RawCameraIntrinsics camerapara_depth_; /*< depth camera intrinsic*/
                  RawCameraIntrinsics camerapara_color_;/*< color camera intrinsic*/
                  cv::Point2f leftup_corner_;/*< left up conrner for the detected person rect*/
                  cv::Point2f rightdown_corner_;/*< right down conrner for the detected person rect*/

                  float dist;/*< distance assumed to be when project the rect from color camera to depth camera*/

                  void ProjectToDepth(cv::Rect roi_rgb, cv::Rect & roi_depth, float pitch);
                  cv::Mat GetHomographyMat(const float pitchRadian);
                  cv::Mat HomographyMat;

                  double HOMOGRAPHY_TABLE_PREVIEW[TABLE_SIZE] = {
                        0.972319, 0.00881748, -159.855, 0.0240053, 0.944766, -96.6235, 8.33475e-05, 0.000131503, 1,
                        1.00403, 0.0271512, -169.211, 0.0263737, 0.986216, -118.362, 8.80487e-05, 0.000267247, 1,
                        1.05028, 0.0534764, -183.118, 0.0277063, 1.04504, -148.015, 9.9142e-05, 0.000452873, 1,
                        1.09108, 0.077477, -196.029, 0.02855, 1.09542, -174.497, 9.45748e-05, 0.000624641, 1,
                        1.13386, 0.0992243, -209.048, 0.0302932, 1.14404, -198.48, 9.81148e-05, 0.000783115, 1,
                        1.16716, 0.115513, -219.165, 0.031608, 1.18055, -216.622, 0.000103164, 0.000900003, 1,
                        1.216, 0.144233, -235.318, 0.0327407, 1.23482, -246.638, 8.93684e-05, 0.00109398, 1,
                        1.28072, 0.175952, -255.534, 0.0347635, 1.29982, -281.528, 9.68791e-05, 0.00131178, 1,
                        1.3276, 0.202237, -271.428, 0.0365503, 1.34418, -310.911, 8.93989e-05, 0.00147642, 1,
                        1.35639, 0.224973, -283.451, 0.037692, 1.36936, -338.156, 1.6379e-05, 0.00164706, 1,
                        1.51153, 0.287603, -330.299, 0.0476686, 1.51706, -400.792, 0.00011324, 0.0020279, 1,
                        1.66361, 0.356558, -380.115, 0.0562004, 1.64853, -476.516, 0.000120553, 0.00247145, 1,
                        1.78857, 0.409373, -419.207, 0.0595834, 1.76281, -530.655, 0.000113836, 0.00285747, 1,
                        1.96044, 0.477887, -473.3, 0.0660156, 1.91161, -603.763, 0.000163691, 0.00330693, 1,
                        2.11068, 0.536891, -519.609, 0.0672026, 2.03188, -672.573, 0.00015582, 0.00375214, 1,
                        2.20737, 0.589085, -556.669, 0.0688749, 2.09994, -736.612, 6.24976e-05, 0.00409893, 1,
                        2.33048, 0.628112, -592.895, 0.0707721, 2.17405, -795.431, 0.000101482, 0.00438706, 1,
                        3.08276, 0.890229, -819.729, 0.096082, 2.73923, -1106.04, 0.00014916, 0.0063851, 1,
                        3.25153, 0.942422, -867.573, 0.102366, 2.86762, -1165.95, 0.00018924, 0.00681092, 1
                  };

                  float _lastDis;
            };

      }
}


#endif //CS_VISION_CA_DEPTHITERESTIMATE_H
