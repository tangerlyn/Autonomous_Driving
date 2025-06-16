#pragma once

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include <Eigen/Dense>
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/buffer.h"
#include <vector>
#include <string>
#include <memory>
#include <cmath>
#include <algorithm>
#include <fstream>

using std::placeholders::_1;

struct csvFileData {
    std::vector<double> s_m;
    std::vector<double> x_m;
    std::vector<double> y_m;
    std::vector<double> psi_rad;
    std::vector<double> kappa_radpm;
    std::vector<double> vx_mps;
    std::vector<double> ax_mps2;
};

struct SteeringState {
    double steering_angle;
    int index;
    double lateral_error;
    double yaw_error;
};

class LQRPID : public rclcpp::Node {
public:
    explicit LQRPID();

private:
    std::string odom_topic;
    std::string car_refFrame;
    std::string global_refFrame;
    std::string drive_topic;
    std::string rviz_nearest_point_topic;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void timer_callback();

    void load_waypoints();
    double pid_velocity(double x, double y, double yaw, double v);
    SteeringState lqr_steering(double x, double y, double yaw, double v);
    void publish_message(double x, double y, double yaw, double v);

    std::pair<int, double> find_nearest_point(const std::vector<double>& cx,
                                              const std::vector<double>& cy,
                                              const std::vector<double>& cyaw,
                                              double x, double y);
    double p2pdist(double x1, double x2, double y1, double y2) const;
    double pi2pi(double angle);

    std::ifstream csvFile_waypoints;
    std::string waypoints_path;
    csvFileData waypoints;
    int num_waypoints = 0;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_drive;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_vis_nearest_point;
    rclcpp::TimerBase::SharedPtr timer_;

    double pid_kp = 1.0;
    double pid_ki = 0.0;
    double pid_kd = 0.1;
    double velocity_error_sum = 0.0;
    double prev_velocity_error = 0.0;

    double prev_lateral_error_ = 0.0;
    double prev_yaw_error_ = 0.0;

    double dt = 0.1;
    double wheelbase;
    double vehicle_length;
    double vehicle_width;
    double max_steer_rad;
    double max_steer_vel;
    double max_accel;
    double max_speed;
    double min_speed;
    
    double dl;
    int n_ind_search;

    double lqr_q_y = 100.0;
    double lqr_q_yaw = 100.0;
    double lqr_r_steer = 1.0;
};

Eigen::Matrix4d solve_dare(const Eigen::Matrix4d& A,
                           const Eigen::Matrix<double, 4, 1>& B,
                           const Eigen::Matrix4d& Q,
                           const Eigen::Matrix<double, 1, 1>& R);

Eigen::Matrix<double, 1, 4> dlqr(const Eigen::Matrix4d& A,
                                 const Eigen::Matrix<double, 4, 1>& B,
                                 const Eigen::Matrix4d& Q,
                                 const Eigen::Matrix<double, 1, 1>& R);

