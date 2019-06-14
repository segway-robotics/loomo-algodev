#include "DepthPreprocess.h"
#include "ninebot_log.h"
#include <math.h>
#include <cmath>

namespace ninebot_algo {
    namespace follow_algo {

        ///***********************public method
        DepthPreprocess::DepthPreprocess()
        {
            camerapara_depth_.focalLengthX = 314.14313;
            camerapara_depth_.focalLengthY = 314.14313;
            camerapara_depth_.pricipalPointX = 159.5;
            camerapara_depth_.pricipalPointY = 117.866;

            camerapara_color_.focalLengthX = 359.5382;
            camerapara_color_.focalLengthY = 358.7130;
            camerapara_color_.pricipalPointX = 327.4261;
            camerapara_color_.pricipalPointY = 251.9224;

            dist = 3.0;//asume the rect is 3.0 meter away...arbitrary
            _curPitch = 0.0f;
            pitch_offset = 0.0f;//0.3f;

            HomographyMat = GetHomographyMat(0);

            _lastDis = 0.2f;
        }

        DepthPreprocess::~DepthPreprocess()
        {

        }

        bool DepthPreprocess::process(const StampedMat &raw_depth, const cv::Rect &stperson_rect,
            cv::Rect &depth_rect, cv::Mat &histImage, float &distance, float &theta)
        {
            cv::Mat hist;
            raw_depth.image.copyTo(_depth);

            ALOGD("raw_depth: cols = %d, rows = %d", raw_depth.image.cols, raw_depth.image.rows);

            ProjectToDepth(stperson_rect, depth_rect, _curPitch);
            depth_rect &= cv::Rect(0, 0, 320, 240);

            ALOGD("depth_rect = (%f, %f)", depth_rect.x, depth_rect.y);
            float factor = 1.4f;
            cv::Rect depth_rect_pad;
            int centerX = depth_rect.x + depth_rect.width * 0.5f;
            int centerY = depth_rect.y + depth_rect.height * 0.5f;
            depth_rect_pad.width = depth_rect.width * factor;
            depth_rect_pad.height = depth_rect.height * factor;
            depth_rect_pad.x = centerX - depth_rect_pad.width * 0.5f;
            depth_rect_pad.y = centerY - depth_rect_pad.height * 0.5f;
            depth_rect_pad &= cv::Rect(0, 0, 320, 240);
            bool is_hist_valid = CalHist(raw_depth, depth_rect_pad, hist, histImage, distance);

            ALOGD("is_hist_valid = %d", is_hist_valid);

            if (is_hist_valid) {
                CalTargetTheta(raw_depth.image.size(), depth_rect, distance, theta);


                //peripheral area process
                if(depth_rect.x < 50  || depth_rect.x + depth_rect.width > 320 - 50)
                {
                    if(distance - _lastDis > 0.02)
                    {
                        distance = _lastDis + 0.02;
                    }
                }
                _lastDis = distance;
                return true;
            }
            else {
                distance = 0;
                theta = 0;
                return false;
            }

        }

        void DepthPreprocess::setColorRect(const cv::Rect &stperson_rect)
        {
            cv::Point2f leftup_corner = cv::Point2f(stperson_rect.x, stperson_rect.y);
            cv::Point2f rightdown_coner = cv::Point2f(stperson_rect.x + stperson_rect.width,
                stperson_rect.y + stperson_rect.height);
            this->leftup_corner_ = leftup_corner;
            this->rightdown_corner_ = rightdown_coner;
        }

        bool DepthPreprocess::CalHist(const StampedMat &raw_depth, const cv::Rect &depth_rect, cv::Mat &hist, cv::Mat &histImage, float &target_val)
        {
            histImage.setTo(cv::Scalar(255, 255, 255));
            cv::Mat roi = raw_depth.image(depth_rect);
            if (roi.size() == cv::Size())
                return false;

            int histSize = 15;
            float range[] = { 0, 6000 };
            const float* histRange = { range };
            bool uniform = true; bool accumulate = false;
            cv::calcHist(&roi, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);
            if (hist.size() == cv::Size())
                return false;

            int scale = 20;
            double minVal = 0;
            double maxVal = 0;
            int target_channel = 1;
            float target_freq = 0;

            cv::minMaxLoc(hist, &minVal, &maxVal, 0, 0);
            int hpt = cv::saturate_cast<int>(20 * histSize);
            for (int i = 0; i < histSize; i++)
            {
                float value = hist.at<float>(i);
                if (target_freq < value && i != 0)
                {
                    target_freq = value;
                    target_channel = i;
                }

                int realValue = cv::saturate_cast<int>(value * hpt / maxVal);
                rectangle(histImage, cv::Point(i*scale, scale*histSize - 1), cv::Point((i + 1)*scale - 1, scale*histSize - realValue), cv::Scalar(255, 0, 0), -1);
            }

            if (CalHistTarget(roi, target_channel, target_val))
                return true;
            else
                return false;
        }

        bool DepthPreprocess::CalHistTarget(const cv::Mat &roi, const int channel, float &res) {
            float start_peroid = (channel) * 6.0f / 15.0f;
            float end_period = (channel + 1) * 6.0f / 15.0f;
            //   ALOGD("DepthPreprocess CalHistTarget start:%f end:%f",start_peroid, end_period);
            int width = roi.cols;
            int height = roi.rows;
            const uint16_t max_depth_mm = 6000;
            float sum = 0.0f;
            int sample_count = 1;
            for (int y = 0; y < height; y += 1) {
                const uint16_t *data = roi.ptr<ushort>(y);
                for (int x = 0; x < width; x += 1) {
                    uint16_t z = data[x];
                    if (z <= 0 || z > max_depth_mm) //this can filter most noise in the rect
                        continue;

                    float p_z = (z) * 0.001f;
                    if (p_z > start_peroid && p_z < end_period)//statistic the valid pixels in the rect, and compute mean represents
                    {
                        sample_count++;
                        sum += p_z;
                        
                        if (x > int(width *1.5 / 4) && x < int(width * 2.5 / 4) && y > 0 &&
                            y < int(height * 1 / 4)) {
                            sample_count += 4;
                            sum += p_z * 4;
                        }
                    }
                }
            }

            if (sample_count > 20)
            {
                res = sum / sample_count;
                return true;
            }
            else
            {
                res = 0;
                return false;
            }
        }

        void DepthPreprocess::CalTargetTheta(const cv::Size &img_size, const cv::Rect &rect, const float dist, float& theta)
        {
            float InvFx = 1.0f / camerapara_depth_.focalLengthX;
            float projection_ratio = dist * InvFx;
            int center_x = rect.x + rect.width / 2;
            int center_y = rect.y + rect.height / 2;

            float dx = ((center_x - camerapara_depth_.pricipalPointX) * projection_ratio);
            float dy = ((center_y - camerapara_depth_.pricipalPointY) * projection_ratio);
            theta = atan2(dist, dx) - 3.1415f / 2.0;
        }

        void DepthPreprocess::ProjectToDepth(cv::Rect roi_rgb, cv::Rect & roi_depth, float pitch)
        {
            vector<cv::Point2f> ptSrc;
            cv::Point src_leftDown, src_rightDown;

            cv::Point temp;
            temp.x = roi_rgb.x;
            temp.y = roi_rgb.y;
            ptSrc.push_back(temp);

            temp.x = roi_rgb.x;
            temp.y = roi_rgb.y + roi_rgb.height;
            ptSrc.push_back(temp);

            temp.x = roi_rgb.x + roi_rgb.width;
            temp.y = roi_rgb.y;
            ptSrc.push_back(temp);

            temp.x = roi_rgb.x + roi_rgb.width;
            temp.y = roi_rgb.y + roi_rgb.height;
            ptSrc.push_back(temp);

            vector<cv::Point2f> ptDst(4);
            HomographyMat = GetHomographyMat(pitch);

            ALOGD("pitch = %f",pitch);

            cv::perspectiveTransform(ptSrc, ptDst, HomographyMat);
            roi_depth.x = floor(max(ptDst.at(0).x, ptDst.at(1).x));
            roi_depth.y = floor(max(ptDst.at(0).y, ptDst.at(2).y));
            roi_depth.width = ceil(min(ptDst.at(2).x, ptDst.at(3).x) - roi_depth.x);
            roi_depth.height = ceil(min(ptDst.at(1).y, ptDst.at(3).y) - roi_depth.y);

            roi_depth.height += roi_depth.height*0.5f;// TODO

            // Vertical constratins on depth bounding boux 
            if (roi_depth.y < 0) 
                // roi_depth.y = -roi_depth.height/2;
                roi_depth.y = 0;
            // double overlapRate = 0.3046 * 1.5;
	        // roi_depth.height = overlapRate * 240;
            ALOGD("roi_depth = (%d, %d, %d, %d)",roi_depth.x, roi_depth.y, roi_depth.width, roi_depth.height);

        }

        cv::Mat DepthPreprocess::GetHomographyMat(const float pitchRadian) {
            double* homo_table = NULL;
            homo_table = HOMOGRAPHY_TABLE_PREVIEW;
            cv::Mat homography(3, 3, CV_32F, 0.0);
            float pitch_degree = pitchRadian * 180.0f / 3.1415f;
            pitch_degree = pitch_degree < 0 ? 0 : pitch_degree;
            pitch_degree = pitch_degree >= MAX_DEGREE ? MAX_DEGREE - 1 : pitch_degree;
            int homo_table_colomn_index = pitch_degree / DEGREE_STEP;
            int start_id = homo_table_colomn_index * MAT_SIZE;
            int width = sqrt(MAT_SIZE);

            for (int i = 0; i < width; i++)
            {
                int x_off = i * width;
                for (int j = 0; j < width; j++)
                    homography.at<float>(i, j) = homo_table[start_id + x_off + j];
            }
            return homography;
        }
    }
}