/**
 * @file tiers_common.h
 * @author EunsooLim
 * @brief common variable and function
 * @version 0.1
 * @date 2022-02-01
 *
 * @copyright BSD
 *
 */
#ifndef __COMMON_H__
#define __COMMON_H__

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <tf/tf.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#define RAD_TO_DEG(RAD) (RAD) * (180.f) / (M_PI)
#define DEG_TO_RAD(DEG) (DEG) * (M_PI) / (180.f)
#define RAD_360 (2 * M_PI)

#define THRESHOLD_YAW DEG_TO_RAD(5)
#define THRESHOLD_DISTANCE 0.5

enum InfoDirType {
    X,
    Y,
    Z,
    YAW,
};

enum BasicMoveType {
    ROTATE,
    LINEAR_MOVE,
    FINAL_ROTATE,
};

struct Waypoint {
    double_t x_goal, y_goal, z_goal, yaw_goal;
};

/**calculation distance between two points**/
void ThreePointDistance(const std::vector<std::pair<double_t, double_t>> &point, double_t &distance) {
    distance = sqrt(pow(point[0].first - point[0].second, 2) + pow(point[1].first - point[1].second, 2));
}

/**calculation between current position and goal position**/
void TwoPointDistance(const std::vector<std::pair<double_t, double_t>> &point, double_t &distance) {
    distance = sqrt(pow(point[0].first - point[0].second, 2) + pow(point[1].first - point[1].second, 2) + pow(point[2].first - point[2].second, 2));
}

void TransRadToPiToPi(double_t &before_rad) {
    while (true) {
        if (before_rad > M_PI) {
            before_rad -= (M_PI * 2.0);

        } else if (before_rad < (-1) * M_PI) {
            before_rad += (M_PI * 2.0);
        } else {
            return;
        }
    }
}

#endif
