/**
 * @file tiers_move.h
 * @author EunsooLim
 * @brief robot movement control class
 * @version 0.1
 * @date 2022-02-01
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef __TIER2_MOVE__
#define __TIER2_MOVE__

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

#include "PID.h"
#include "tiers_common.h"
#include "tiers_pose.h"
class Move {
   private:
    // ros variable
    ros::NodeHandle pram_handler_;
    ros::NodeHandle n_;
    ros::Publisher velocity_pub;
    geometry_msgs::Twist input_twist_;

    std::string velocity_address;
    Eigen::Vector4d goal_pose_, input_velocity;
    // PID coefficient
    double_t pid_dt_ = 0.01, pid_Kp_ = 0.5, pid_Kd_ = 0.1, pid_Ki_ = 0.0, pid_max_, pid_min_;
    double_t pid_z_dt_ = 0.01, pid_z_max_ = 0.7, pid_z_min_ = 0.0, pid_z_Kp_ = 1.0, pid_z_Kd_ = 0.1, pid_z_Ki_ = 0.0;
    double_t pid_th_dt_ = 0.01, pid_th_max_ = 3.0, pid_th_min_ = 0.0, pid_th_Kp_ = 1.0, pid_th_Kd_ = 0.1, pid_th_Ki_ = 0.0;
    std::vector<double_t> points_contain_;
    // movement function point to point
    void moveOnePoint();
    // confused PID and linear velocity
    double_t linearFlightAlgorithm(const double_t &&goal_yaw, const std::shared_ptr<PID> &linear_pid, const std::shared_ptr<PID> &linear_z_pid, const std::shared_ptr<PID> &th_w_pid);
    // confused PID and rotation velocity
    bool RotateAlgorithm(const double_t &goal_yaw, const double_t &yaw, const std::shared_ptr<PID> &w_pid);
    // get param from config file
    void rosparamHandler() {
        pram_handler_.param<double_t>("MAX_SPEED", pid_max_, 1.5);
        pram_handler_.param<double_t>("MIN_SPEED", pid_min_, 0);
        pram_handler_.param<std::string>("VELOCITY_TOPIC_NAME", velocity_address, "tiers2/imu");
        pram_handler_.param<std::vector<double_t>>("POSITION_YAW_LIST", points_contain_);

        Waypoint temp;
        for (uint8_t index = 1; index < points_contain_.size(); index++) {
            if (index % 4 == 1) {
                temp.x_goal = points_contain_[index];
            } else if (index % 4 == 2) {
                temp.y_goal = points_contain_[index];
            } else if (index % 4 == 3) {
                temp.z_goal = points_contain_[index];
            } else if (index % 4 == 0) {
                temp.yaw_goal = DEG_TO_RAD(points_contain_[index]);

                TransRadToPiToPi(temp.yaw_goal);
                waypoints_vector.push_back(temp);
            }
        }
    }

   public:
    std::vector<Waypoint> waypoints_vector;
    bool moveWaypoint(const std::vector<Waypoint> &waypoints);

    std::shared_ptr<GetPose> tier2_pose_ = std::make_shared<GetPose>();

    Move();
    ~Move();
};

Move::Move() {
    rosparamHandler();
    velocity_pub = n_.advertise<geometry_msgs::Twist>(velocity_address, 10);
}
Move::~Move() {
    ROS_INFO("terminate");
}

#endif
