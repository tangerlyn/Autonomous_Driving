#include "lqr_pid/lqr_pid.hpp"

#include <math.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Eigen>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <limits>
#include <Eigen/Dense>
#include <functional>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <chrono>
using namespace std::chrono;
rclcpp::Time start_time_;
rclcpp::Time end_time_;
bool lap_started_ = false;
bool lap_finished_ = false;

using namespace std::chrono_literals;

LQRPID::LQRPID() : Node("lqr_pid_node") {
    // initialise parameters
    this->declare_parameter("waypoints_path", "/sim_ws/src/lqr_pid/racelines/basic.csv");
    this->declare_parameter("odom_topic", "/ego_racecar/odom");
    this->declare_parameter("car_refFrame", "ego_racecar/base_link");
    this->declare_parameter("drive_topic", "/sim/drive");
    //this->declare_parameter("rviz_lqr_traj_topic", "/lqr_traj");
    this->declare_parameter("rviz_nearest_point_topic", "/nearest_point");
    this->declare_parameter("global_refFrame", "map");

    this->declare_parameter("pid_kp", 1.0);
    this->declare_parameter("pid_ki", 0.0);
    this->declare_parameter("pid_kd", 0.1);

    //this->declare_parameter("lqr_preview_horizon", 6);
    this->declare_parameter("lqr_q_y", 300.0);
    this->declare_parameter("lqr_q_yaw", 300.0);
    this->declare_parameter("lqr_r_steer", 0.1);

    this->declare_parameter("wheelbase", 0.33);
    this->declare_parameter("vehicle_length", 0.58);
    this->declare_parameter("vehicle_width", 0.31);
    this->declare_parameter("max_steer_rad", 0.7);
    this->declare_parameter("max_steer_vel", 3.2);
    this->declare_parameter("max_accel", 3.0);
    this->declare_parameter("max_speed", 5.0);
    this->declare_parameter("min_speed", 0.0);

    this->declare_parameter("dt", 0.1);
    this->declare_parameter("dl", 0.03);
    this->declare_parameter("n_ind_search", 20);

    // Default Values
    waypoints_path = this->get_parameter("waypoints_path").as_string();
    odom_topic = this->get_parameter("odom_topic").as_string();
    car_refFrame = this->get_parameter("car_refFrame").as_string();
    drive_topic = this->get_parameter("drive_topic").as_string();
    //rviz_lqr_traj_topic = this->get_parameter("rviz_lqr_traj_topic").as_string();
    rviz_nearest_point_topic = this->get_parameter("rviz_nearest_point_topic").as_string();
    global_refFrame = this->get_parameter("global_refFrame").as_string();

    pid_kp = this->get_parameter("pid_kp").as_double();
    pid_ki = this->get_parameter("pid_ki").as_double();
    pid_kd = this->get_parameter("pid_kd").as_double();

    //lqr_preview_horizon = this->get_parameter("lqr_preview_horizon").as_double();
    lqr_q_y = this->get_parameter("lqr_q_y").as_double();
    lqr_q_yaw = this->get_parameter("lqr_q_yaw").as_double();
    lqr_r_steer = this->get_parameter("lqr_r_steer").as_double();

    wheelbase = this->get_parameter("wheelbase").as_double();
    vehicle_length = this->get_parameter("vehicle_length").as_double();
    vehicle_width = this->get_parameter("vehicle_width").as_double();
    max_steer_rad = this->get_parameter("max_steer_rad").as_double();
    max_accel = this->get_parameter("max_accel").as_double();
    max_speed = this->get_parameter("max_speed").as_double();
    min_speed = this->get_parameter("min_speed").as_double();

    dt = this->get_parameter("dt").as_double();
    dl = this->get_parameter("dl").as_double();
    n_ind_search = this->get_parameter("n_ind_search").as_int();

    // Publisher and Subscription
    sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 25, std::bind(&LQRPID::odom_callback, this, _1));
    timer_ = this->create_wall_timer(2000ms, std::bind(&LQRPID::timer_callback, this));

    pub_drive = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 25);
    //pub_vis_lqr_traj = this->create_publisher<visualization_msgs::msg::Marker>(rviz_lqr_traj_topic, 10);
    //pub_vis_nearest_point = this->create_publisher<visualization_msgs::msg::Marker>(rviz_nearest_point_topic, 10);

    // not use in world frame.. it's for pure pursuit
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    //for labtime
    lap_started_ = false;
    lap_finished_ = false;

    RCLCPP_INFO(this->get_logger(), "LQR-PID node has been launched");

    load_waypoints();
}

double LQRPID::p2pdist(double x1, double x2, double y1, double y2) const {
    double dist = sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
    return dist;
}

double LQRPID::pi2pi(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
}

void LQRPID::load_waypoints() {
    csvFile_waypoints.open(waypoints_path, std::ios::in);

    if (!csvFile_waypoints.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot Open CSV File: %s", waypoints_path.c_str());
        return;
    } else {
        RCLCPP_INFO(this->get_logger(), "CSV File Opened");
    }

    std::string line, word;

    while (std::getline(csvFile_waypoints, line)) {
        std::stringstream s(line);
        std::vector<std::string> tokens;
        while (std::getline(s, word, ';')) {
            tokens.push_back(word);
        }

        if (tokens.size() < 7) {
            RCLCPP_WARN(this->get_logger(), "Skipping line: not enough columns (%zu)", tokens.size());
            continue;
        }

        try {
            waypoints.s_m.push_back(std::stod(tokens[0]));
            waypoints.x_m.push_back(std::stod(tokens[1]));
            waypoints.y_m.push_back(std::stod(tokens[2]));
            waypoints.psi_rad.push_back(std::stod(tokens[3]));
            waypoints.kappa_radpm.push_back(std::stod(tokens[4]));
            waypoints.vx_mps.push_back(std::stod(tokens[5]));
            waypoints.ax_mps2.push_back(std::stod(tokens[6]));
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error parsing line: %s", e.what());
        }
    }

    csvFile_waypoints.close();
    num_waypoints = waypoints.x_m.size();

    RCLCPP_INFO(this->get_logger(), "Finished loading %d waypoints from %s", num_waypoints, waypoints_path.c_str());

    double average_dist_between_waypoints = 0.0;
    for (int i = 0; i < num_waypoints - 1; i++) {
        average_dist_between_waypoints += this->p2pdist(
            waypoints.x_m[i], waypoints.x_m[i + 1],
            waypoints.y_m[i], waypoints.y_m[i + 1]);
    }
    average_dist_between_waypoints /= (num_waypoints - 1);
    RCLCPP_INFO(this->get_logger(), "Average distance between waypoints: %f", average_dist_between_waypoints);
}

//void LQRPID::visualize_nearest_point()



std::pair<int, double> LQRPID::find_nearest_point(
    const std::vector<double>& cx,
    const std::vector<double>& cy,
    const std::vector<double>& cyaw,
    double x, double y)
{
    int nearest_ind = 0;
    double min_d2 = std::numeric_limits<double>::infinity();

    // 1. 가장 가까운 인덱스 찾기
    for (std::size_t i = 0; i < cx.size(); ++i) {
        double dx = cx[i] - x;
        double dy = cy[i] - y;
        double d2 = dx * dx + dy * dy;
        if (d2 < min_d2) {
            min_d2 = d2;
            nearest_ind = i;
        }
    }

    // 2. lateral error 계산 (최근접 기준)
    double e = std::sqrt(min_d2);
    double angle = pi2pi(cyaw[nearest_ind] - std::atan2(cy[nearest_ind] - y, cx[nearest_ind] - x));
    if (angle < 0.0) e = -e;

    // 3. preview horizon 적용
    int preview_horizon = 2;  // 실험적으로 조정: 10 ~ 20
    int target_ind = std::min(nearest_ind + preview_horizon, static_cast<int>(cx.size() - 1));

    return {target_ind, e};  // target point index와 최근접 기준 lateral error 반환
}



double LQRPID::pid_velocity(double x, double y, double yaw, double v) {
    (void)yaw;
    constexpr double epsilon = 1e-4;

    auto [idx, _] = find_nearest_point(waypoints.x_m, waypoints.y_m, waypoints.psi_rad, x, y);

    double kappa = std::abs(waypoints.kappa_radpm[idx]);
    double v_ref = waypoints.vx_mps[idx];

    double gain_scale = 1.0 / (kappa + epsilon);

    double error = v_ref - v;
    velocity_error_sum += error * dt;
    double d_error = (error - prev_velocity_error) / dt;

    double output = gain_scale * (
        pid_kp * error +
        pid_ki * velocity_error_sum +
        pid_kd * d_error
    );

    output = std::clamp(output, 0.0, max_speed);

    prev_velocity_error = error;

    return output;
}
Eigen::Matrix4d solve_dare(const Eigen::Matrix4d& A, const Eigen::Matrix<double,4,1>& B,
                    const Eigen::Matrix4d& Q, const Eigen::Matrix<double,1,1>& R) {
	Eigen::Matrix4d X = Q;
    for (int i = 0; i < 150; ++i) {
	    Eigen::Matrix<double,1,1> tmp = R + B.transpose() * X * B;
	    Eigen::Matrix4d Xn = A.transpose() * X * A - A.transpose() * X * B * tmp.inverse() * B.transpose() * X * A + Q;
        if ((Xn - X).cwiseAbs().maxCoeff() < 1e-2) {
            return Xn;
        }
        X = Xn;
    }
    return X;
}

Eigen::Matrix<double,1,4> dlqr(const Eigen::Matrix4d& A, const Eigen::Matrix<double,4,1>& B,
                        const Eigen::Matrix4d& Q, const Eigen::Matrix<double,1,1>& R) {
	Eigen::Matrix4d X = solve_dare(A, B, Q, R);
    return (B.transpose() * X * B + R).inverse() * (B.transpose() * X * A);
}





SteeringState LQRPID::lqr_steering(double x, double y, double yaw, double v) {
    // 1. 최근접 인덱스 + preview 적용
    auto [ind, e] = find_nearest_point(
        waypoints.x_m, waypoints.y_m, waypoints.psi_rad, x, y);

    // 2. 추종 대상 waypoint의 방향 및 곡률
    double psi = waypoints.psi_rad[ind];
    double k = waypoints.kappa_radpm[ind];
    double th_e = pi2pi(yaw - psi);  // heading error

    // 3. 속도 조건 검사
    if (std::abs(v) < 0.1) {
        SteeringState s;
        s.steering_angle = 0.0;
        s.index = ind;
        s.lateral_error = e;
        s.yaw_error = th_e;
        return s;
    }

    // 4. 상태-입력 행렬 정의
    double direction = (v >= 0) ? 1.0 : -1.0;

    Eigen::Matrix4d A = Eigen::Matrix4d::Zero();
    A(0,0) = 1.0; A(0,1) = dt;
    A(1,2) = std::abs(v);  // always positive
    A(2,2) = 1.0; A(2,3) = dt;
    A(3,3) = 1.0;

    Eigen::Matrix<double,4,1> B = Eigen::Matrix<double,4,1>::Zero();
    B(3,0) = direction * std::abs(v) / wheelbase;

    Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
    Q(0,0) = lqr_q_y;
    Q(1,1) = 1.0;
    Q(2,2) = lqr_q_yaw;
    Q(3,3) = 1.0;

    Eigen::Matrix<double,1,1> R;
    R(0,0) = lqr_r_steer;

    Eigen::Matrix<double,1,4> K = dlqr(A, B, Q, R);

    Eigen::Vector4d x_err;
    x_err << e,
             (e - prev_lateral_error_) / dt,
             th_e,
             (th_e - prev_yaw_error_) / dt;

    x_err *= direction;

    double ff = std::atan2(wheelbase * k, 1.0);
    double fb = pi2pi(- (K * x_err)(0));
    double delta = std::clamp(ff + fb, -max_steer_rad, max_steer_rad);

    prev_lateral_error_ = e;
    prev_yaw_error_ = th_e;

    // 디버깅 출력
    RCLCPP_INFO(this->get_logger(), "LQR idx=%d, e=%.3f, th_e=%.3f, psi=%.3f, yaw=%.3f, delta=%.3f",
                ind, e, th_e, psi, yaw, delta);

    SteeringState state;
    state.steering_angle = delta;
    state.index = ind;
    state.lateral_error = e;
    state.yaw_error = th_e;
    return state;
}



void LQRPID::publish_message(double x, double y, double yaw, double velocity) {
    SteeringState steer = lqr_steering(x, y, yaw, velocity);

    double target_velocity = pid_velocity(x, y, yaw, velocity);

    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = this->get_clock()->now();
    drive_msg.header.frame_id = "base_link";

    drive_msg.drive.steering_angle = steer.steering_angle;
    drive_msg.drive.speed = target_velocity;

    pub_drive->publish(drive_msg);
}




void LQRPID::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer_->lookupTransform(global_refFrame, msg->header.frame_id, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
        return;
    }

    // 위치 변환
    const auto& t = tf.transform.translation;
    const auto& r = tf.transform.rotation;
    tf2::Vector3 translation(t.x, t.y, t.z);
    tf2::Quaternion rotation(r.x, r.y, r.z, r.w);
    tf2::Transform tf2_transform(rotation, translation);

    tf2::Vector3 pos_orig(msg->pose.pose.position.x,
                          msg->pose.pose.position.y,
                          msg->pose.pose.position.z);
    tf2::Vector3 pos_transformed = tf2_transform * pos_orig;

    double x = pos_transformed.x();
    double y = pos_transformed.y();

    const auto& q = msg->pose.pose.orientation;
    tf2::Quaternion q_orig(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_orig).getRPY(roll, pitch, yaw);
        yaw -= M_PI_2;
        yaw = pi2pi(yaw);

   double velocity = msg->twist.twist.linear.x;

   // change
   // 랩 타임 측정 시작
    if (!lap_started_) {
        start_x_ = x;
        start_y_ = y;
        start_time_ = this->now();
        lap_started_ = true;
        RCLCPP_INFO(this->get_logger(), "Lap started at %.2f", start_time_.seconds());
    }

    // 랩 종료 조건: 시작점 근처에 도달했을 때 (반경 1m 이내)
    double dx = x - start_x_;
    double dy = y - start_y_;
    double dist = std::sqrt(dx * dx + dy * dy);


    //double elapsed_time = (this->now() - start_time_).seconds();
    if (lap_started_ && !lap_finished_ && dist < 1.0 && (this->now() - start_time_).seconds() > 1.0) {  // 시작 지점으로 돌아왔을 때
        end_time_ = this->now();
        lap_finished_ = true;

        // 랩 타임 계산
        double lap_time = (end_time_ - start_time_).seconds();
        RCLCPP_INFO(this->get_logger(), "Lap finished! Time: %.2f seconds", lap_time);
    }


    publish_message(x, y, yaw, velocity);
}

void LQRPID::timer_callback() {
    pid_kp = this->get_parameter("pid_kp").as_double();
    pid_ki = this->get_parameter("pid_ki").as_double();
    pid_kd = this->get_parameter("pid_kd").as_double();

    lqr_q_y = this->get_parameter("lqr_q_y").as_double();
    lqr_q_yaw = this->get_parameter("lqr_q_yaw").as_double();
    lqr_r_steer = this->get_parameter("lqr_r_steer").as_double();

    wheelbase = this->get_parameter("wheelbase").as_double();
    max_speed = this->get_parameter("max_speed").as_double();
    max_steer_rad = this->get_parameter("max_steer_rad").as_double();

    dt = this->get_parameter("dt").as_double();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node_ptr = std::make_shared<LQRPID>();
  rclcpp::spin(node_ptr);
  rclcpp::shutdown();
  return 0;
}
