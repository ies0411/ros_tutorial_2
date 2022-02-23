/**
 * @file tiers_pose.h
 * @author EunsooLim
 * @brief get a estimated pose
 * @version 0.1
 * @date 2022-02-01
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef __TIERS_POSE__
#define __TIERS_POSE__

/*
2. Write a simple node to control a virtual aerial robot. Assume that the full 6D
pose of the robot is received through a given topic in PoseStamped messages.


The node should publish TwistStamped messages to a given topic, in order to
follow a predefined 3D trajectory. The trajectory is given in the form of an array
as a parameter. For each point in the trajectory, there is a given orientation
(yaw). Include all parameters (topic names, trajectory, speed) in a roslaunch
file. If a PoseStamped message is not received within 200ms, the robot should
log an error message.

For this second node, you can write a simple solution that assumes the robot is
able to fly at constant speed even while turning (instant acceleration and
deceleration in any direction). Therefore, there is no need to consider any
controller for the speed other than having the proper direction (to follow the
trajectory) and orientation (at each point of the trajectory).
*/
#include "tiers_common.h"

class GetPose {
   private:
    // ros variable
    ros::NodeHandle pram_handler_;
    ros::NodeHandle nh_;
    ros::Time pose_time_check_;

    std::string gps_topic_name_, imu_topic_name_;

    // get a fusioned robot pose , callback function
    void EKFFilter(const geometry_msgs::PoseStamped::ConstPtr &imu, const geometry_msgs::PoseStamped::ConstPtr &gps);

    void rosparamHandler() {
        pram_handler_.param<std::string>("GPS_TOPIC_NAME", gps_topic_name_, "tiers2/gps");
        pram_handler_.param<std::string>("IMU_TOPIC_NAME", imu_topic_name_, "tiers2/imu");
    }

   public:
    Eigen::Vector4d current_pose_;

    // checking sensor callback timetic
    ros::Time getTime() { return pose_time_check_; }

    GetPose();
    ~GetPose();
};

GetPose::GetPose() {
    message_filters::Subscriber<geometry_msgs::PoseStamped> gps_sub_(nh_, "tiers2/gps", 1);
    message_filters::Subscriber<geometry_msgs::PoseStamped> imu_sub_(nh_, "tiers2/imu", 1);
    message_filters::TimeSynchronizer<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> sync(gps_sub_, imu_sub_, 1);
    sync.registerCallback(boost::bind(&GetPose::EKFFilter, this, _1, _2));
}
GetPose::~GetPose() {
    ROS_INFO("terminate Getpose class");
}

void GetPose::EKFFilter(const geometry_msgs::PoseStamped::ConstPtr &imu, const geometry_msgs::PoseStamped::ConstPtr &gps) {
    // acquiring roll,pitch,yaw from imu
    pose_time_check_ = ros::Time::now();
    // convert quaternion to rpy
    tf::Quaternion q(
        imu->pose.orientation.x,
        imu->pose.orientation.y,
        imu->pose.orientation.z,
        imu->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw, temp;
    m.getRPY(roll, pitch, yaw);
    // estimation x,y,z through Filterng imu and gps
    current_pose_[InfoDirType::YAW] = yaw;
    current_pose_[InfoDirType::X] = gps->pose.position.x;
    current_pose_[InfoDirType::Y] = gps->pose.position.y;
    current_pose_[InfoDirType::Z] = gps->pose.position.z;
}
#endif
