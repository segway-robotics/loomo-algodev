//
// Created by tangchu on 2017/9/12.
//

#ifndef CS_VISION_ALGO_ALGOTENSORFLOWSAMPLE_H
#define CS_VISION_ALGO_ALGOTENSORFLOWSAMPLE_H
#include "AlgoBase.h"
#include "tensorflow/core/framework/graph.pb.h"
#include "tensorflow/core/framework/node_def.pb.h"
#include "tensorflow/core/framework/tensor.h"
#include "tensorflow/core/graph/graph.h"
#include "tensorflow/core/graph/graph_constructor.h"
#include "tensorflow/core/platform/env.h"
#include "tensorflow/core/platform/platform.h"
#include "tensorflow/core/platform/types.h"
#include "tensorflow/core/public/session.h"
#include <fstream>
#include <algorithm>

#define IMG_C 3
#define IMG_W 299//299
#define IMG_H 299//299

#define R_MEAN 122.67891434f
#define G_MEAN 116.66876762f
#define B_MEAN 104.00698793f

#define IMG_VALUE_NORMALIZATION 1
#define IMG_ZERO_MEAN 0

namespace ninebot_algo {
    namespace ninebot_tensorflow{
        template <typename T>
        std::string ToString(T val)
        {
            std::stringstream stream;
            stream << std::fixed << std::setprecision(2) << val;
            return stream.str();
        }
        class AlgoTensorflowSample : public AlgoBase {
        public:
            AlgoTensorflowSample(RawData *rawInterface, int run_sleep_ms, bool is_render);

            ~AlgoTensorflowSample();

            virtual bool step();    // run algorithm once
            RawData *main_RawData;

            float runTime();    // return the runtime of algorithm in ms, stat this number in step()
            bool showScreen(void *pixels);    // output the content inside a char array *data, the memory is allocated outside

        private:
            /*
             * convert image to tensor that is feed to predictor
             */
            bool convert_image_to_feed_data(const cv::Mat srcImg, tensorflow::Tensor* imgTensor);

            /*
             * draw classification result on the image
             */
            void draw_result(std::vector<tensorflow::Tensor>& output_vector, cv::Mat& input_image);

            bool m_isRender;
            std::mutex mMutexTimer;
            std::mutex _mutex_display;
            float m_ptime;
            StampedMat raw_color;
            cv::Mat canvas;

            tensorflow::SessionOptions options;
            tensorflow::Session* session;
            tensorflow::GraphDef* graph_definition;
            tensorflow::Tensor* img_tensor;

        };
    }
}



#endif //CS_VISION_ALGO_ALGOTENSORFLOWSAMPLE_H
