//
// Created by tangchu on 2017/9/12.
//

#include "AlgoTensorflowSample.h"
#include "classes.h"
using namespace ninebot_algo;
using namespace ninebot_tensorflow;

//using namespace tensorflow;

AlgoTensorflowSample::AlgoTensorflowSample(RawData *rawInterface, int run_sleep_ms, bool is_render)
        :AlgoBase(rawInterface,run_sleep_ms)
{
    ALOGD("tensorflow: start init");
    canvas = cv::Mat::zeros(cv::Size(640, 360), CV_8UC3);
    main_RawData = mRawDataInterface;
    this->m_isRender = is_render;

    std::string graph_path = "/sdcard/tensorflow/inception_v3_2016_08_28_frozen.pb";
    tensorflow::ConfigProto& config = options.config;
    //set to single thread
    config.set_intra_op_parallelism_threads(1);

    session = NewSession(options);
    graph_definition = new tensorflow::GraphDef();
    tensorflow::Status s = tensorflow::ReadBinaryProto(tensorflow::Env::Default(), graph_path, graph_definition);
    if (!s.ok()) {
        ALOGD("tensorflow: Failed to create load Graph from path: %s", graph_path.c_str());
        return;
    }
    if(session != NULL)
    {
        s = session->Create(*graph_definition);
    }
    else
    {
        ALOGD("tensorflow: session == NULL");
        return;
    }

    if (!s.ok()) {
        ALOGD("tensorflow: Could not create TensorFlow Session");
        return;
    }

    //initialize input tensor
    img_tensor = new tensorflow::Tensor(tensorflow::DT_FLOAT, tensorflow::TensorShape({1, IMG_W, IMG_H, 3}));
}

AlgoTensorflowSample::~AlgoTensorflowSample()
{
    delete img_tensor;
    img_tensor = NULL;
    delete graph_definition;
    graph_definition = NULL;
    delete session;
    session = NULL;
}

bool AlgoTensorflowSample::convert_image_to_feed_data(const cv::Mat srcImg, tensorflow::Tensor* imgTensor)
{
    if(srcImg.empty() || imgTensor == NULL || srcImg.channels() != IMG_C)
        return false;

    cv::Mat resizedImg;
    cv::resize(srcImg, resizedImg, cv::Size(IMG_W, IMG_H));
    cv::Mat float_img;
    resizedImg.convertTo(float_img, CV_32F);

    if(IMG_VALUE_NORMALIZATION)
    {
        float_img = float_img * 0.003922;//1 / 255
    }

    std::vector<cv::Mat> splited_mat;
    cv::split(float_img, splited_mat);
    cv::Mat r_channel = splited_mat[2].clone();
    cv::Mat g_channel = splited_mat[1].clone();
    cv::Mat b_channel = splited_mat[0].clone();

    if(IMG_ZERO_MEAN)
    {
        if(IMG_VALUE_NORMALIZATION)
        {
            r_channel = r_channel - R_MEAN * 0.003922;// 1 / 255
            g_channel = g_channel - G_MEAN * 0.003922;
            b_channel = b_channel - B_MEAN * 0.003922;
        }
        else
        {
            r_channel = r_channel - R_MEAN;
            g_channel = g_channel - G_MEAN;
            b_channel = b_channel - B_MEAN;
        }
    }
    float* data = const_cast<float *>(reinterpret_cast<const float *>(img_tensor->tensor_data().data()));

    for(int i = 0; i < IMG_H; i ++)
    {
        for(int j = 0; j < IMG_W; j ++)
        {
            data[(i * IMG_W + j) * 3] = r_channel.at<float>(i, j);
            data[(i * IMG_W + j) * 3 + 1] = g_channel.at<float>(i, j);
            data[(i * IMG_W + j) * 3 + 2] = b_channel.at<float>(i, j);
        }
    }
//    std::memcpy(data, (float*)b_channel.data, IMG_H * IMG_W * sizeof(float));
//    std::memcpy(&(data[IMG_W * IMG_H]), (float*)g_channel.data, IMG_H * IMG_W * sizeof(float));
//    std::memcpy(&(data[IMG_W * IMG_H * 2]), (float*)r_channel.data, IMG_H * IMG_W * sizeof(float));
    return true;
}


bool AlgoTensorflowSample::step()
{
    ALOGD("tensorflow: start step");
    auto start = std::chrono::high_resolution_clock::now();
    main_RawData->retrieveColor(raw_color);
    //main_RawData->retrieveDS4Color(raw_color, false);
    if (raw_color.timestampSys == 0)
    {
        return false;
    }
    if(raw_color.image.channels() != 3)
    {
        ALOGD("tensorflow: not supported!");
        return false;
    }
    cv::Mat tcolor = raw_color.image.clone();
    cv::resize(tcolor, tcolor, cv::Size(640,480));

    vector<pair<string, tensorflow::Tensor>> inputs = {{"input", *img_tensor}};
    std::vector<tensorflow::Tensor> outputs;

    convert_image_to_feed_data(tcolor, img_tensor);
    tensorflow::Status s = session->Run(inputs, {"InceptionV3/Predictions/Reshape_1"}, {}, &outputs);
    if (!s.ok()){
        ALOGD("tensorflow: Failed to run session: %s", s.error_message().c_str());
        return -1;
    }

    auto end = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> elapsed = end-start;
    {
        std::lock_guard<std::mutex> lock(mMutexTimer);
        m_ptime = elapsed.count()*0.5 + m_ptime*0.5;
    }

    //show result
    cv::Mat show_mat;
    cv::resize(tcolor,show_mat,cv::Size(480,360));
    cv::Mat draw_mat = canvas(cv::Rect(0,0,480,360));
    show_mat.copyTo(draw_mat);
    draw_result(outputs, draw_mat);

    return true;
}

float AlgoTensorflowSample::runTime()
{
    std::lock_guard<std::mutex> lock(mMutexTimer);
    return m_ptime;
}

bool AlgoTensorflowSample::showScreen(void* pixels)
{
    if(m_isRender)
    {
        cv::Mat display_img;
        {
            std::lock_guard <std::mutex> lock(_mutex_display);
            if (canvas.empty())
                return false;
            cv::cvtColor(canvas, display_img, CV_BGR2RGBA);
        }
        memcpy(pixels, (void *)display_img.data, display_img.cols * display_img.rows * 4);
    }
    return true;
}

void AlgoTensorflowSample::draw_result(std::vector<tensorflow::Tensor>& output_vector, cv::Mat& input_image)
{
    int top_k = 5;
    //sort by probability
    auto classify_vec = output_vector[0].matrix<float>();
    vector<size_t> index(classify_vec.size());
    for (size_t i = 0; i < index.size(); ++i){
        index[i] = i;
    }
    sort(index.begin(), index.end(), [classify_vec](size_t lhs, size_t rhs){
        return classify_vec(0, lhs) > classify_vec(0, rhs);
    });

    ALOGD("tensorflow: the max score label is %lu, the score is %lf\n", index[0], classify_vec(index[0]));

    int x_offset = 10;
    int y_offset = 25;
    for(int i = 0; i < top_k; i++)
    {
        //this model (inception_v3_2016_08_28_frozen.pb) was trained with a dummy class in its
        //ground truth label map,
        //therefore it totally outputs 1001 labels,
        //while the imagenet class label map has actually only 1000 labels,
        //hence the index needs to be adjusted.
        int actual_idx = index[i] - 1 > 0 ? index[i] - 1 : 0;
        std::string name = imagenet_classes[actual_idx];
        double score = classify_vec(index[i]);
        string class_info = name + ": " + ToString(score);
        putText(input_image, class_info, cv::Point(x_offset, y_offset + 25 * i), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255));
    }
}
