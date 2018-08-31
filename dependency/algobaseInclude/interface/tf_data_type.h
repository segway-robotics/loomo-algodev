#ifndef TF2_MESSAGE_TYPE_H
#define TF2_MESSAGE_TYPE_H

#include <stdint.h>
#include <string>
#include <vector>
// which should equal to geometry_msgs::TransformStamped
/// Storage for the rotation

/**
 * all the frame is named by patten with 
 * Frame position + connect to position + frame type + postfix "frame"
 * all the frame named below, further infor please check the wiki.
 * std::string world_odom_origin = "world_odom_frame";
 * std::string world_evio_origin = "world_evio_frame";
 * std::string base_odom_frame = "base_center_ground_frame";
 * std::string base_pose_frame = "base_center_wheel_axis_frame";
 * std::string neck_pose_frame = "neck_center_body_internal_frame";
 * std::string head_pose_y_frame = "neck_center_body_yaw_frame";
 * std::string head_pose_p_frame = "head_center_neck_internal_frame";
 * std::string rs_depth_frame = "rsdepth_center_neck_fix_frame";
 * std::string rs_fe_frame = "rsfisheye_center_neck_fix_frame";
 * std::string rs_color_frame = "rscolor_center_neck_fix_frame";
 * std::string head_pose_p_r_frame = "head_center_neck_pitch_frame";
 * const std::string tf_loomo::platform_cam_frame = "platform_center_head_fix_frame";
*/

namespace ninebot_tf {
typedef uint32_t CompactFrameID;
typedef long long unsigned int llui;
typedef long long int lli;

struct rotation_data_t {
    double x, y, z, w;
    rotation_data_t() : x(0), y(0), z(0), w(1) {}
    rotation_data_t(double _x, double _y, double _z, double _w)
        : x(_x), y(_y), z(_z), w(_w) {}
};

struct translation_data_t {
    double x, y, z;
    translation_data_t() : x(0), y(0), z(0) {}
    translation_data_t(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
};

struct header_t {
    std::string parent_frame_id;
    int64_t timestamp;
    uint32_t seq_increase_id;
};

// TODO convert all the  Transform func to tf_data_t
struct tf_data_t {                   // tf_data_t  == geometry_msgs::Transform
    rotation_data_t rotation;        // xyzw
    translation_data_t translation;  // xyz
    tf_data_t() {}
    tf_data_t(rotation_data_t r, translation_data_t t)
        : rotation(r), translation(t) {}
    tf_data_t(double r_x, double r_y, double r_z, double r_w, double t_x,
              double t_y, double t_z)
        : rotation(r_x, r_y, r_z, r_w), translation(t_x, t_y, t_z) {}
};

namespace TFError {
const int NO_ERROR = 0;
const int OUT_OF_TRESHOLD = -1;
const int LOOKUP_ERROR = -2;
// const int CONNECTIVITY_ERROR = 2;
// const int EXTRAPOLATION_ERROR = 3;
// const int INVALID_ARGUMENT_ERROR = 4;
// const int TIMEOUT_ERROR = 5;
// const int TRANSFORM_ERROR = 6;

enum TF_ERROR_CODE {
    NO_ERR = 0,
    OUT_OF_TRESHOLD_ERR = -1,
    LOOKUP_ERR = -2,
    FRAME_INVALID_ERR = -3,
    LOOKUP_TIME_ERR= -15,
    FRAME_NOT_FOUND_ERR = -16,
    STH_ERR = 12345
};
}  // namespace  TFError

class tf_request_message{
    public:
    std::string src;
    std::string tgt;
    int64_t timestamp;
    int time_trh;
    tf_request_message(std::string tgt_str, std::string src_str,
    int64_t ts,
    int trh){
        src = src_str;
        tgt = tgt_str;
        timestamp = ts;
        time_trh = trh;
    }

    tf_request_message(std::string tgt_str, std::string src_str){
        src = src_str;
        tgt = tgt_str;
        timestamp = -1;
        time_trh = 200;
    }
    
    tf_request_message(){
        src = "";
        tgt = "";
        timestamp = -1;
        time_trh = 200;
    }
};

class tf_message {  // tf_message == geometry_msgs::Transformstamped
   public:
    tf_message(){
    };
    tf_data_t tf_data;  // transform
    std::string frame_id;
    header_t header;
    TFError::TF_ERROR_CODE err;
    int init(){
        header.timestamp = 0;
        header.parent_frame_id = " ";
        frame_id = " ";
        tf_data.rotation.x = 0;
        tf_data.rotation.y = 0;
        tf_data.rotation.z = 0;
        tf_data.rotation.w = 1;
        tf_data.translation.x = 0;
        tf_data.translation.y = 0;
        tf_data.translation.z = 0;
		return 0;
    }
};

typedef std::vector<ninebot_tf::tf_request_message> vector_req_t;

typedef std::vector<ninebot_tf::tf_message> vector_tf_msg_t;

}
#endif  // TF2_MESSAGE_TYPE_H
