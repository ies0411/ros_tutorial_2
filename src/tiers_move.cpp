#include "tiers_move.h"

void Move::moveOnePoint() {
    ros::Rate move_rate(100);
    // generate PID instance
    std::shared_ptr<PID> linear_speed_pid = std::make_shared<PID>();
    std::shared_ptr<PID> z_linear_speed_pid = std::make_shared<PID>();
    std::shared_ptr<PID> th_w_pid = std::make_shared<PID>();
    // PID set
    linear_speed_pid->PID_set(pid_dt_, pid_max_, pid_min_, pid_Kp_, pid_Kd_, pid_Ki_);
    z_linear_speed_pid->PID_set(pid_th_dt_, pid_z_max_, pid_z_min_, pid_z_Kp_, pid_z_Kd_, pid_z_Ki_);
    th_w_pid->PID_set(pid_th_dt_, pid_th_max_, pid_th_min_, pid_th_Kp_, pid_th_Kd_, pid_th_Ki_);

    uint8_t seq = 0;
    ros::Duration duration;
    while (ros::ok()) {
        ros::spinOnce();
        move_rate.sleep();
        // senor time checking
        duration = ros::Time::now() - tier2_pose_->getTime();
        if (duration.toSec() > 0.2) {
            ROS_ERROR_STREAM("Hz is slow than 0.2ms");
            input_twist_.linear.x = 0;
            input_twist_.linear.y = 0;
            input_twist_.linear.z = 0;
            input_twist_.angular.z = 0;
            return;
        }
        // RTR Flight  1.heading change 2.movement 3.heading change
        switch (seq) {
            // heading chage align to goal pose
            case BasicMoveType::ROTATE: {
                double_t diff_x = goal_pose_[InfoDirType::X] - tier2_pose_->current_pose_[InfoDirType::X];
                double_t diff_y = goal_pose_[InfoDirType::Y] - tier2_pose_->current_pose_[InfoDirType::Y];
                double_t tmp_yaw_goal = atan2(diff_y, diff_x);
                if (RotateAlgorithm(tmp_yaw_goal, tier2_pose_->current_pose_[InfoDirType::YAW], th_w_pid)) {
                    seq++;
                }
                // just input yawrate value
                input_twist_.linear.x = 0;
                input_twist_.linear.y = 0;
                input_twist_.linear.z = 0;
                input_twist_.angular.z = input_velocity[InfoDirType::YAW];
                break;
            }
            // movement
            case BasicMoveType::LINEAR_MOVE: {
                double_t diff_x = goal_pose_[InfoDirType::X] - tier2_pose_->current_pose_[InfoDirType::X];
                double_t diff_y = goal_pose_[InfoDirType::Y] - tier2_pose_->current_pose_[InfoDirType::Y];
                double_t tmp_yaw_goal = atan2(diff_y, diff_x);
                double_t distance = linearFlightAlgorithm(std::move(tmp_yaw_goal), linear_speed_pid, z_linear_speed_pid, th_w_pid);
                if (distance < THRESHOLD_DISTANCE) {
                    seq++;
                }
                input_twist_.linear.x = input_velocity[InfoDirType::X];
                input_twist_.linear.y = input_velocity[InfoDirType::Y];
                input_twist_.linear.z = input_velocity[InfoDirType::Z];
                input_twist_.angular.z = input_velocity[InfoDirType::YAW];

                break;
            }
            // ailgn to goal value of yaw
            case BasicMoveType::FINAL_ROTATE: {
                if (RotateAlgorithm(goal_pose_[InfoDirType::YAW], tier2_pose_->current_pose_[InfoDirType::YAW], th_w_pid)) {
                    return;
                }
                // just input yawrate value
                input_twist_.linear.x = 0;
                input_twist_.linear.y = 0;
                input_twist_.linear.z = 0;
                input_twist_.angular.z = input_velocity[InfoDirType::YAW];
                ;
                break;
            }
        }
        velocity_pub.publish(input_twist_);
    }
}

double_t Move::linearFlightAlgorithm(const double_t &&goal_yaw, const std::shared_ptr<PID> &linear_pid, const std::shared_ptr<PID> &linear_z_pid, const std::shared_ptr<PID> &th_w_pid) {
    // coordinate convert
    Eigen::MatrixXf Tr(2, 2), pre_pose(2, 1), Tr_pose(2, 1), pre_goal_pose(2, 1), Tr_goal_pose(2, 1);
    double_t th = -tier2_pose_->current_pose_[InfoDirType::YAW];
    Tr << cos(th), -sin(th),
        sin(th), cos(th);
    pre_pose << tier2_pose_->current_pose_[InfoDirType::X], tier2_pose_->current_pose_[InfoDirType::Y];
    Tr_pose = Tr * pre_pose;

    double_t transfer_pose_x = Tr_pose(0);
    double_t transfer_pose_y = Tr_pose(1);

    pre_goal_pose << goal_pose_[InfoDirType::X], goal_pose_[InfoDirType::Y];
    Tr_goal_pose = Tr * pre_goal_pose;

    double_t transfer_pose_x_goal = Tr_goal_pose(0);
    double_t transfer_pose_y_goal = Tr_goal_pose(1);

    std::vector<std::pair<double_t, double_t>> point;

    point.emplace_back(std::make_pair(transfer_pose_x, transfer_pose_x_goal));
    point.emplace_back(std::make_pair(transfer_pose_y, transfer_pose_y_goal));
    point.emplace_back(std::make_pair(tier2_pose_->current_pose_[InfoDirType::Z], goal_pose_[InfoDirType::Z]));

    double_t distance;
    ThreePointDistance(point, distance);
    double_t x_y_distance;
    TwoPointDistance(point, x_y_distance);

    double_t diff_pose_x = transfer_pose_x_goal - transfer_pose_x;
    double_t diff_pose_y = transfer_pose_y_goal - transfer_pose_y;

    double_t diff_pose_z = goal_pose_[InfoDirType::Z] - tier2_pose_->current_pose_[InfoDirType::Z];

    double_t tan_degree = atan2(diff_pose_y, diff_pose_x);

    // PID, input : distance , output : velocity
    // PID instance are divided by (x,y),(z),(yaw)
    double_t &&speed_pid = linear_pid->calculate(0.0, x_y_distance);
    double_t &&z_speed_pid = linear_z_pid->calculate(0.0, fabs(diff_pose_z));

    if (diff_pose_z > 0) {
        input_velocity[InfoDirType::Z] = z_speed_pid;
    } else if (diff_pose_z < 0) {
        input_velocity[InfoDirType::Z] = (-1) * z_speed_pid;
    }

    input_velocity[InfoDirType::Y] = (speed_pid)*sin(tan_degree);
    input_velocity[InfoDirType::X] = (speed_pid)*cos(tan_degree);

    RotateAlgorithm(goal_pose_[InfoDirType::YAW], tier2_pose_->current_pose_[InfoDirType::YAW], th_w_pid);

    return distance;
}

// trajectory(waypoint) function is formed by moveOnepoint function
bool Move::moveWaypoint(const std::vector<Waypoint> &waypoints) {
    for (uint8_t i = 0; i < waypoints.size(); i++) {
        goal_pose_[InfoDirType::X] = waypoints.at(i).x_goal;
        goal_pose_[InfoDirType::Y] = waypoints.at(i).y_goal;
        goal_pose_[InfoDirType::Z] = waypoints.at(i).z_goal;

        if (i == waypoints.size() - 1) {
            moveOnePoint();
        }
    }
    return true;
}

bool Move::RotateAlgorithm(const double_t &goal_yaw, const double_t &yaw, const std::shared_ptr<PID> &w_pid) {
    double_t fabs_yaw = fabs(goal_yaw - yaw);
    double_t diff_yaw = goal_yaw - yaw;
    // turn_speed_ =

    if (fabs_yaw > M_PI) {
        if (RAD_360 - fabs_yaw < THRESHOLD_YAW) {
            input_velocity[InfoDirType::YAW] = 0;

            return true;
        } else {
            if (diff_yaw > 0) {
                input_velocity[InfoDirType::YAW] = w_pid->calculate(0, fabs_yaw);
            } else {
                input_velocity[InfoDirType::YAW] = (-1 * w_pid->calculate(0, fabs_yaw));
            }
        }
    } else {
        if (fabs_yaw < THRESHOLD_YAW) {
            input_velocity[InfoDirType::YAW] = 0;

            return true;
        } else {
            if (diff_yaw > 0) {
                input_velocity[InfoDirType::YAW] = (-1 * w_pid->calculate(0, fabs_yaw));
            } else {
                input_velocity[InfoDirType::YAW] = w_pid->calculate(0, fabs_yaw);
            }
        }
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "tiers_2");
    Move move;

    ros::Rate rate(100);

    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();

        if (move.moveWaypoint(move.waypoints_vector)) return 1;
    }

    return 0;
}
