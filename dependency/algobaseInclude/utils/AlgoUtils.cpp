/*! define basic utility for AlgoUtils.cpp
 *
 * Filename: AlgoUtils.cpp
 * Version: 0.30
 * Algo team, Ninebot Inc., 2017
 */
#include "AlgoUtils.h"

namespace ninebot_algo {
	StampedPose tfmsgTo2DPose(const ninebot_tf::tf_message & tf_msg){
		double yaw;
		double q0, q1, q2, q3;
		q0 = tf_msg.tf_data.rotation.w;
		q1 = tf_msg.tf_data.rotation.x;
		q2 = tf_msg.tf_data.rotation.y;
		q3 = tf_msg.tf_data.rotation.z;
		yaw = atan2(2.0*(q0*q3 + q1*q2),
					1 - 2.0*(q2*q2 + q3*q3));
		StampedPose pos;
		pos.pose.x = tf_msg.tf_data.translation.x;
		pos.pose.y = tf_msg.tf_data.translation.y;
		pos.pose.orientation = yaw;
		pos.timestamp = tf_msg.header.timestamp;

		return pos;
	}

	Eigen::Isometry3f tfmsgToPose(const ninebot_tf::tf_message& tf_msg) {
        double q0, q1, q2, q3;
        q0 = tf_msg.tf_data.rotation.w;
		q1 = tf_msg.tf_data.rotation.x;
		q2 = tf_msg.tf_data.rotation.y;
		q3 = tf_msg.tf_data.rotation.z;
		Eigen::Quaternionf q(q0,q1,q2,q3);
		Eigen::Isometry3f pose;
		pose = q.toRotationMatrix();
		Eigen::Vector3f trans;
		trans[0] = tf_msg.tf_data.translation.x;
		trans[1] = tf_msg.tf_data.translation.y;
		trans[2] = tf_msg.tf_data.translation.z;
		pose.translation() = trans;

		return pose;
	}

	Eigen::Isometry3d tfmsgToIsometry3d(const ninebot_tf::tf_message& tf_msg) {
		Eigen::Quaterniond q(
				tf_msg.tf_data.rotation.w, tf_msg.tf_data.rotation.x,
				tf_msg.tf_data.rotation.y, tf_msg.tf_data.rotation.z);
		Eigen::Vector3d t(
				tf_msg.tf_data.translation.x, tf_msg.tf_data.translation.y,
				tf_msg.tf_data.translation.z);

		Eigen::Isometry3d pose;

		pose.setIdentity();
		pose.matrix().topLeftCorner(3, 3) = q.toRotationMatrix();
		pose.matrix().topRightCorner(3, 1) = t;

		return pose;
	}

    ninebot_tf::tf_message packageTfData(std::string tgt, std::string src, int64_t ts, float qx, float qy, float qz, float qw, float tx, float ty, float tz){
        ninebot_tf::tf_message tf;
        tf.init();
        tf.tf_data.translation = ninebot_tf::translation_data_t(tx,ty,tz);
        tf.tf_data.rotation = ninebot_tf::rotation_data_t(qx, qy, qz, qw);
        tf.header.timestamp = ts;
        tf.header.parent_frame_id = tgt;
        tf.frame_id = src;
        tf.err = ninebot_tf::TFError::TF_ERROR_CODE(0);
        return tf;
    }

	Eigen::Isometry3f PoseToCamcoor(const Eigen::Isometry3f & pose_odom) {
		Eigen::Matrix3f rs_rot = pose_odom.matrix().block(0, 0, 3, 3);
		Eigen::Quaternionf q(rs_rot);
		// convert to realsense coordinate
		Eigen::Quaternionf q_c_o = getQuatCamOdo();
		Eigen::Quaternionf t;
		t.w() = 0;
		t.vec() = pose_odom.translation();
		// t
		t = q_c_o.inverse() * t * q_c_o;
		// r
		q = q_c_o.inverse() * q * q_c_o;

		Eigen::Isometry3f pose;
		pose = q.toRotationMatrix();
		pose.translation() = t.vec();

		return pose;
	}

	Eigen::Vector3f quatToEuler(const Eigen::Quaternionf & quat) {
		float yaw, pitch, roll;
		float q0, q1, q2, q3;
		q0 = quat.w();
		q1 = quat.x();
		q2 = quat.y();
		q3 = quat.z();

		roll = (atan2(2.0*(q0*q1 + q2*q3),
					  1 - 2.0*(q1*q1 + q2*q2)));

		pitch = asin(2.0*(q0*q2 - q3*q1));

		yaw = atan2(2.0*(q0*q3 + q1*q2),
					1 - 2.0*(q2*q2 + q3*q3));

		Eigen::Vector3f angle;
		angle[1] = pitch;
		angle[0] = roll;
		angle[2] = yaw;

		return angle;
	}

	Eigen::Vector3d quatToEuler(const Eigen::Quaterniond & quat) {
		double yaw, pitch, roll;
		double q0, q1, q2, q3;
		q0 = quat.w();
		q1 = quat.x();
		q2 = quat.y();
		q3 = quat.z();

		roll = (atan2(2.0*(q0*q1 + q2*q3),
					  1 - 2.0*(q1*q1 + q2*q2)));

		pitch = asin(2.0*(q0*q2 - q3*q1));

		yaw = atan2(2.0*(q0*q3 + q1*q2),
					1 - 2.0*(q2*q2 + q3*q3));

		Eigen::Vector3d angle;
		angle[1] = pitch;
		angle[0] = roll;
		angle[2] = yaw;

		return angle;
	}

    Eigen::Quaternionf getQuatCamOdo() {
        Eigen::AngleAxisf roll(-M_PI_2, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitch(0.0, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yaw(-M_PI_2, Eigen::Vector3f::UnitZ());
        Eigen::Quaternionf q = yaw*pitch*roll;

        return q;
    }

    Eigen::Quaternionf getQuatOdoCam() {
        Eigen::AngleAxisf roll(M_PI_2, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitch(-M_PI_2, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yaw(0.0, Eigen::Vector3f::UnitZ());
        Eigen::Quaternionf q = yaw*pitch*roll;

        return q;
    }

	Eigen::Isometry3f getBasePose(const StampedTwist raw_odometry, const StampedBasePos raw_basepos) {
		// Odometry
		float x = raw_odometry.twist.pose.x;
		float y = raw_odometry.twist.pose.y;
		float odo_yaw = raw_odometry.twist.pose.orientation;

		// Basepos
		float base_roll = raw_basepos.roll;
		float base_pitch = raw_basepos.pitch;
		float base_yaw = raw_basepos.yaw;

		Eigen::AngleAxisf roll_axis(base_roll, Eigen::Vector3f::UnitX());
		Eigen::AngleAxisf pitch_axis(base_pitch, Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf yaw_axis(odo_yaw, Eigen::Vector3f::UnitZ());
		Eigen::Quaternionf q = yaw_axis*pitch_axis*roll_axis;

		Eigen::Vector3f trans;
		trans[0] = x;
		trans[1] = y;
		trans[2] = 0.0;

		Eigen::Isometry3f pose;
		pose = q.toRotationMatrix();
		pose.translation() = trans;

		return pose;
	}

	Eigen::Isometry3f getUltrasonicPose(const StampedTwist raw_odometry, const StampedBasePos raw_basepos) {
		Eigen::Isometry3f base_pose = getBasePose(raw_odometry, raw_basepos);
		Eigen::Matrix3f rot = base_pose.matrix().block(0, 0, 3, 3);
		Eigen::Vector3f trans = base_pose.translation();

		Eigen::Vector3f ultrasonic_center(ULTRASONIC_WIDTH, 0, ULTRASONIC_HEIGHT);
		ultrasonic_center = rot*ultrasonic_center;

		Eigen::Isometry3f pose;
		pose = rot;
		pose.translation() = trans + ultrasonic_center;

		return pose;
	}

	Eigen::Isometry3f getNeckPose(const StampedTwist raw_odometry, const StampedBasePos raw_basepos) {
		Eigen::Isometry3f base_pose = getBasePose(raw_odometry, raw_basepos);
		Eigen::Matrix3f rot = base_pose.matrix().block(0, 0, 3, 3);
		Eigen::Vector3f trans = base_pose.translation();
		// the origin point is the center of robot on fisheye coordinate.
		Eigen::Vector3f neck_center(NECK_WIDTH, 0, NECK_HEIGHT);
		neck_center = rot*neck_center;

		Eigen::Isometry3f pose;
		pose = rot;
		pose.translation() = trans + neck_center;

		return pose;
	}

	/* Get robot Pose from realsense camera, both on odometry coordinate */
	Eigen::Isometry3f getBasePose(const Eigen::Isometry3f realsense_pose, const StampedHeadPos raw_headpos,
		RealsenseSensorType realsense_sensor) {
		Eigen::Matrix3f rs_rot = realsense_pose.matrix().block(0, 0, 3, 3);
		Eigen::Quaternionf rs_q(rs_rot);
		Eigen::Vector3f rs_t = realsense_pose.translation();
		
		// translation
		Eigen::Vector3f rs_center;
		rs_center[0] = REALSENSE_WIDTH;
		rs_center[2] = REALSENSE_HEIGHT;
		switch (realsense_sensor)
		{
		case Fisheye:
			rs_center[1] = FISHEYE_LENGTH;
			break;
		case Depth:
			rs_center[1] = DEPTH_LENGTH;
			break;
		case DS4Color:
			rs_center[1] = DS4COLOR_LENGTH;
			break;
		default:
			break;
		}
		Eigen::Quaternionf rs_center_q;
		rs_center_q.w() = 0;
		rs_center_q.vec() = rs_center;
		rs_center_q = rs_q * rs_center_q * rs_q.inverse();

		float head_yaw = raw_headpos.yaw;
		Eigen::AngleAxisf head_roll_axis(0.0, Eigen::Vector3f::UnitX());
		Eigen::AngleAxisf head_pitch_axis(0.0, Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf head_yaw_axis(head_yaw, Eigen::Vector3f::UnitZ());
		Eigen::Quaternionf head_q = head_yaw_axis*head_pitch_axis*head_roll_axis;

		Eigen::Quaternionf neck_q = rs_q*head_q.inverse();
		Eigen::Vector3f neck_t = rs_t - rs_center_q.vec();

		Eigen::Vector3f neck_center(NECK_WIDTH, 0.0, NECK_HEIGHT);
		Eigen::Quaternionf neck_center_q;
		neck_center_q.w() = 0;
		neck_center_q.vec() = neck_center;
		neck_center_q = neck_q*neck_center_q*neck_q.inverse();

		Eigen::Quaternionf base_q = neck_q;
		Eigen::Vector3f base_t = neck_t - neck_center_q.vec();

		Eigen::Isometry3f pose;
		pose = base_q.toRotationMatrix();
		pose.translation() = base_t;

		return pose;
	}

	/* Get robot base pose on odometry coordinate from realsense camera with camera coordinate */
	Eigen::Isometry3f getBasePoseFromCamcoor(const Eigen::Isometry3f realsense_pose, const StampedHeadPos raw_headpos, 
		RealsenseSensorType realsense_sensor) {
        Eigen::Matrix3f rot = realsense_pose.matrix().block(0,0,3,3);
        Eigen::Vector3f translation = realsense_pose.translation();

		Eigen::Quaternionf q_c_o = getQuatCamOdo();
		// translation
		Eigen::Vector3f center;
		center[0] = REALSENSE_WIDTH;
		center[2] = REALSENSE_HEIGHT;
		switch (realsense_sensor)
		{
		case Fisheye:
			center[1] = FISHEYE_LENGTH;
			break;
		case Depth:
			center[1] = DEPTH_LENGTH;
			break;
		case DS4Color:
			center[1] = DS4COLOR_LENGTH;
			break;
		default:
			break;
		}
		Eigen::Quaternionf center_q;
		center_q.w() = 0;
		center_q.vec() = center;
		center_q = q_c_o.inverse() * center_q * q_c_o;
		center = rot*center_q.vec();

		float head_yaw = raw_headpos.yaw;
		Eigen::AngleAxisf head_roll_axis(0.0, Eigen::Vector3f::UnitX());
		Eigen::AngleAxisf head_pitch_axis(0.0, Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf head_yaw_axis(head_yaw, Eigen::Vector3f::UnitZ());
		Eigen::Quaternionf head_q = head_yaw_axis*head_pitch_axis*head_roll_axis;
		head_q = q_c_o.inverse()*head_q*q_c_o;

		Eigen::Quaternionf neck_q = Eigen::Quaternionf(rot)*head_q.inverse();
		Eigen::Vector3f neck_t = translation - center;

		Eigen::Vector3f neck_center(NECK_WIDTH, 0.0, NECK_HEIGHT);
		Eigen::Quaternionf neck_center_q;
		neck_center_q.w() = 0;
		neck_center_q.vec() = neck_center;
		neck_center_q = q_c_o.inverse() * neck_center_q * q_c_o;
		neck_center = neck_q.toRotationMatrix()*neck_center_q.vec();

		Eigen::Quaternionf base_q = neck_q;
		Eigen::Vector3f base_t = neck_t - neck_center;

		// convert to odometry coordinate
		Eigen::Quaternionf q_o_c = getQuatOdoCam();
		Eigen::Quaternionf t;
		t.w() = 0;
		t.vec() = base_t;
		// t
		t = q_o_c.inverse() * t * q_o_c;
		// r
		base_q = q_o_c.inverse() * base_q * q_o_c;

		Eigen::Isometry3f pose;
		pose = base_q.toRotationMatrix();
		pose.translation() = t.vec();

		return pose;
    }

	/* Get robot base pose from realsense camera, both on camera coordinate */
	Eigen::Isometry3f getBasePoseFromCamcoorToCamcoor(const Eigen::Isometry3f realsense_pose, const StampedHeadPos raw_headpos,
		RealsenseSensorType realsense_sensor) {
		Eigen::Matrix3f rot = realsense_pose.matrix().block(0, 0, 3, 3);
		Eigen::Vector3f translation = realsense_pose.translation();

		Eigen::Quaternionf q_c_o = getQuatCamOdo();
		// translation
		Eigen::Vector3f center;
		center[0] = REALSENSE_WIDTH;
		center[2] = REALSENSE_HEIGHT;
		switch (realsense_sensor)
		{
		case Fisheye:
			center[1] = FISHEYE_LENGTH;
			break;
		case Depth:
			center[1] = DEPTH_LENGTH;
			break;
		case DS4Color:
			center[1] = DS4COLOR_LENGTH;
			break;
		default:
			break;
		}
		Eigen::Quaternionf center_q;
		center_q.w() = 0;
		center_q.vec() = center;
		center_q = q_c_o.inverse() * center_q * q_c_o;
		center = rot*center_q.vec();

		float head_yaw = raw_headpos.yaw;
		Eigen::AngleAxisf head_roll_axis(0.0, Eigen::Vector3f::UnitX());
		Eigen::AngleAxisf head_pitch_axis(0.0, Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf head_yaw_axis(head_yaw, Eigen::Vector3f::UnitZ());
		Eigen::Quaternionf head_q = head_yaw_axis*head_pitch_axis*head_roll_axis;
		head_q = q_c_o.inverse()*head_q*q_c_o;

		Eigen::Quaternionf neck_q = Eigen::Quaternionf(rot)*head_q.inverse();
		Eigen::Vector3f neck_t = translation - center;

		Eigen::Vector3f neck_center(NECK_WIDTH, 0.0, NECK_HEIGHT);
		Eigen::Quaternionf neck_center_q;
		neck_center_q.w() = 0;
		neck_center_q.vec() = neck_center;
		neck_center_q = q_c_o.inverse() * neck_center_q * q_c_o;
		neck_center = neck_q.toRotationMatrix()*neck_center_q.vec();

		Eigen::Quaternionf base_q = neck_q;
		Eigen::Vector3f base_t = neck_t - neck_center;

		Eigen::Isometry3f pose;
		pose = base_q.toRotationMatrix();
		pose.translation() = base_t;		

		return pose;
	}

	/* Get realsense camera pose on odometry coordinate from robot raw data*/
	Eigen::Isometry3f getRsPose(const StampedTwist raw_odometry, const StampedHeadPos raw_headpos,
		const StampedBasePos raw_basepos, RealsenseSensorType realsense_sensor) {
		// get neck pose
		Eigen::Isometry3f neck_pose = getNeckPose(raw_odometry, raw_basepos);
		Eigen::Matrix3f neck_rot = neck_pose.matrix().block(0, 0, 3, 3);
		Eigen::Vector3f neck_trans = neck_pose.translation();

		// Headpos yaw
		float head_yaw = raw_headpos.yaw;
		Eigen::AngleAxisf head_roll_axis(0.0, Eigen::Vector3f::UnitX());
		Eigen::AngleAxisf head_pitch_axis(0.0, Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf head_yaw_axis(head_yaw, Eigen::Vector3f::UnitZ());
		Eigen::Quaternionf head_q = head_yaw_axis*head_pitch_axis*head_roll_axis;

		// quaternion
		Eigen::Quaternionf q = Eigen::Quaternionf(neck_rot)*head_q;
		// translation
		Eigen::Vector3f center;
		center[0] = REALSENSE_WIDTH;
		center[2] = REALSENSE_HEIGHT;
		switch (realsense_sensor)
		{
		case Fisheye:
			center[1] = FISHEYE_LENGTH;
			break;
		case Depth:
			center[1] = DEPTH_LENGTH;
			break;
		case DS4Color:
			center[1] = DS4COLOR_LENGTH;
			break;
		default:
			break;
		}
		Eigen::Vector3f translation = neck_trans + q.toRotationMatrix()*center;

		Eigen::Isometry3f pose;
		pose = q.toRotationMatrix();
		pose.translation() = translation;

		return pose;
	}

    /* Get realsense camera pose on realsense coordinate from robot raw data*/
    Eigen::Isometry3f getRsPoseToCamcoor(const StampedTwist raw_odometry, const StampedHeadPos raw_headpos,
		const StampedBasePos raw_basepos, RealsenseSensorType realsense_sensor) {
		Eigen::Isometry3f rs_pose = getRsPose(raw_odometry, raw_headpos, raw_basepos, realsense_sensor);
		Eigen::Matrix3f rs_rot = rs_pose.matrix().block(0, 0, 3, 3);
		Eigen::Quaternionf q(rs_rot);
		// convert to realsense coordinate
		Eigen::Quaternionf q_c_o = getQuatCamOdo();
		Eigen::Quaternionf t;
		t.w() = 0;
		t.vec() = rs_pose.translation();
		// t
		t = q_c_o.inverse() * t * q_c_o;
		// r
		q = q_c_o.inverse() * q * q_c_o;

        Eigen::Isometry3f pose;
        pose = q.toRotationMatrix();
        pose.translation() = t.vec();

        return pose;
    }

	/* Get platform center pose on odometry coordinate from robot raw data*/
	Eigen::Isometry3f getPlatformCenterPose(const StampedTwist raw_odometry, const StampedHeadPos raw_headpos,
											const StampedBasePos raw_basepos)
	{
		// get neck pose
		Eigen::Isometry3f neck_pose = getNeckPose(raw_odometry, raw_basepos);
		Eigen::Matrix3f neck_rot = neck_pose.matrix().block(0, 0, 3, 3);
		Eigen::Vector3f neck_trans = neck_pose.translation();

		// Headpos yaw
		float head_yaw = raw_headpos.yaw;
		Eigen::AngleAxisf head_roll_axis(0.0, Eigen::Vector3f::UnitX());
		Eigen::AngleAxisf head_pitch_axis(0.0, Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf head_yaw_axis(head_yaw, Eigen::Vector3f::UnitZ());
		Eigen::Quaternionf head_q = head_yaw_axis*head_pitch_axis*head_roll_axis;

		// quaternion
		Eigen::Quaternionf q = Eigen::Quaternionf(neck_rot)*head_q;
		// translation
		Eigen::Vector3f center;
		center[0] = 0.0;
		center[1] = 0.0;
		center[2] = PLATFORM_CENTER_HEIGHT;
		Eigen::Vector3f translation = neck_trans + q.toRotationMatrix()*center;

		Eigen::Isometry3f pose;
		pose = q.toRotationMatrix();
		pose.translation() = translation;

		return pose;
	}

	/* Get platform center pose on camera coordinate (same as realsense) from robot raw data*/
	Eigen::Isometry3f getPlatformCenterPoseToCamcoor(const StampedTwist raw_odometry, const StampedHeadPos raw_headpos,
													 const StampedBasePos raw_basepos)
	{
		Eigen::Isometry3f platform_center_pose = getPlatformCenterPose(raw_odometry, raw_headpos, raw_basepos);
		Eigen::Matrix3f pc_rot = platform_center_pose.matrix().block(0, 0, 3, 3);
		Eigen::Quaternionf q(pc_rot);
		// convert to camera coordinate
		Eigen::Quaternionf q_c_o = getQuatCamOdo();
		Eigen::Quaternionf t;
		t.w() = 0;
		t.vec() = platform_center_pose.translation();
		// t
		t = q_c_o.inverse() * t * q_c_o;
		// r
		q = q_c_o.inverse() * q * q_c_o;

		Eigen::Isometry3f pose;
		pose = q.toRotationMatrix();
		pose.translation() = t.vec();

		return pose;
	}

	/* Get platform color camera pose on odometry coordinate from robot raw data*/
	Eigen::Isometry3f getPlatformColorPose(const StampedTwist raw_odometry, const StampedHeadPos raw_headpos,
										   const StampedBasePos raw_basepos)
	{
		// get platform center pose
		Eigen::Isometry3f pc_pose = getPlatformCenterPose(raw_odometry, raw_headpos, raw_basepos);
		Eigen::Matrix3f pc_rot = pc_pose.matrix().block(0, 0, 3, 3);
		Eigen::Vector3f pc_trans = pc_pose.translation();

		// Headpos pitch
		float head_pitch = raw_headpos.pitch;
		Eigen::AngleAxisf head_roll_axis(0.0, Eigen::Vector3f::UnitX());
		Eigen::AngleAxisf head_pitch_axis(-head_pitch, Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf head_yaw_axis(0.0, Eigen::Vector3f::UnitZ());
		Eigen::Quaternionf head_q = head_yaw_axis*head_pitch_axis*head_roll_axis;

		// quaternion
		Eigen::Quaternionf q = Eigen::Quaternionf(pc_rot)*head_q;
		// translation
		Eigen::Vector3f center;
		center[0] = PLATFORM_COLOR_WIDTH;
		center[1] = PLATFORM_COLOR_LENGTH;
		center[2] = PLATFORM_COLOR_HEIGHT;
		Eigen::Vector3f translation = pc_trans + q.toRotationMatrix()*center;

		Eigen::Isometry3f pose;
		pose = q.toRotationMatrix();
		pose.translation() = translation;

		return pose;
	}

	/* Get platform color camera pose on camera coordinate (same as realsense) from robot raw data*/
	Eigen::Isometry3f getPlatformColorPoseToCamcoor(const StampedTwist raw_odometry, const StampedHeadPos raw_headpos,
													const StampedBasePos raw_basepos)
	{
		Eigen::Isometry3f platform_color_pose = getPlatformColorPose(raw_odometry, raw_headpos, raw_basepos);
		Eigen::Matrix3f pc_rot = platform_color_pose.matrix().block(0, 0, 3, 3);
		Eigen::Quaternionf q(pc_rot);
		// convert to camera coordinate
		Eigen::Quaternionf q_c_o = getQuatCamOdo();
		Eigen::Quaternionf t;
		t.w() = 0;
		t.vec() = platform_color_pose.translation();
		// t
		t = q_c_o.inverse() * t * q_c_o;
		// r
		q = q_c_o.inverse() * q * q_c_o;

		Eigen::Isometry3f pose;
		pose = q.toRotationMatrix();
		pose.translation() = t.vec();

		return pose;
	}
}
