#include <sstream>
#include <string>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
/// CHECK: include needed ROS msg type headers and libraries
#include "lqr.hpp"
#include "tf2/utils.h"
#include <fstream>
#include <iostream>

using namespace std;

struct TrajectoryPoint {
    double s;
    double x;
    double y;
    double psi;
    double kappa;
    double v;
    double a;
};

std::vector<TrajectoryPoint> loadTrajectoryFromCSV(const std::string& filepath) {
    std::vector<TrajectoryPoint> trajectory;
    std::ifstream file(filepath);

    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filepath << std::endl;
        return trajectory;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        std::vector<double> values;

        while (std::getline(ss, token, ';')) {
            try {
                values.push_back(std::stod(token));
            } catch (const std::invalid_argument& e) {
                std::cerr << "Invalid value: " << token << std::endl;
            }
        }

        if (values.size() == 7) {
            trajectory.push_back({
                values[0], values[1], values[2],
                values[3], values[4], values[5], values[6]
            });
        } else {
            std::cerr << "Invalid line with " << values.size() << " fields: " << line << std::endl;
        }
    }

    file.close();
    return trajectory;
}

class MPC : public rclcpp::Node
{
    // Implement MPC
    // This is just a template, you are free to implement your own node!

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    ackermann_msgs::msg::AckermannDriveStamped drive_msg_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    double current_speed_ = 0.0;

    std::vector<TrajectoryPoint> trajectory_;
    std::vector<double> cx_, cy_, cyaw_, ck_;
    double prev_e_ = 0.0;
    double prev_th_e_ = 0.0;

    double pid_integral_ = 0.0;
    double prev_v_error_ = 0.0;

public:
    MPC() : Node("mpc_node")
    {
        // TODO: create ROS subscribers and publishers
	// Load trajectory
	trajectory_ = loadTrajectoryFromCSV("src/mpc/racelines/traj_full.csv");
        for (const auto &pt : trajectory_) {
            cx_.push_back(pt.x);
            cy_.push_back(pt.y);
            cyaw_.push_back(pt.psi);
            ck_.push_back(pt.kappa);
        }

	// create subscriber & publisher
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
			"/pf/viz/inferred_pose", 10,
			std::bind(&MPC::pose_callback, this, std::placeholders::_1));

	odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
			"/sim/ego_racecar/odom", 10,
			std::bind(&MPC::odom_callback, this, std::placeholders::_1)
);

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
    }

    void compute_pid_gains(double kappa, double& Kp, double& Ki, double& Kd)
    {
	// more kappa, more sensitive - just hard cording
        // Kp = 1.0 + 4.0 * kappa;
	// Ki = 0.1 * (1.0 + kappa);
        // Kd = 0.5 * (1.0 + 2.0 * kappa);
	
	// PID based tanh
	double scale = 10.0; // can adjust, it is parameter
	double abs_kappa = std::abs(kappa);
	double response = std::tanh(scale * abs_kappa); // nomalization react to kappa

	Kp = 1.0 + 3.0 * response;   // 1.0 ~ 4.0
        Ki = 0.1 + 0.2 * response;   // 0.1 ~ 0.3
        Kd = 0.5 + 1.0 * response;   // 0.5 ~ 1.5

    }

    double compute_pid_speed(double ref_v, double current_v, double kappa, double dt)
{
        double Kp, Ki, Kd;
        compute_pid_gains(kappa, Kp, Ki, Kd);

        double v_error = ref_v - current_v;
        pid_integral_ += v_error * dt;
        double d_error = (v_error - prev_v_error_) / dt;

        double output = Kp * v_error + Ki * pid_integral_ + Kd * d_error;
        prev_v_error_ = v_error;

        //output = std::clamp(output, -2.0, 5.0);  // speed limit
        return output;
}

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
    {
        double x = pose_msg->pose.position.x;
        double y = pose_msg->pose.position.y;
        double yaw = tf2::getYaw(pose_msg->pose.orientation);
        double v = current_speed_; // current velocity

        // LQR calling
        double delta;
        int nearest_idx;
        double e, th_e;

        std::tie(delta, nearest_idx, e, th_e) = lqrSteeringControl(
            cx_, cy_, cyaw_, ck_, x, y, yaw, v, prev_e_, prev_th_e_);

        prev_e_ = e;
        prev_th_e_ = th_e;
	
	// PID calling
	double ref_v = trajectory_[nearest_idx].v;
	double kappa = trajectory_[nearest_idx].kappa;
	double dt = 0.1;

	double cmd_speed = compute_pid_speed(ref_v, v, kappa, dt);
        
	// Ackermann message publish
        drive_msg_.drive.steering_angle = delta;
        drive_msg_.drive.speed = cmd_speed;
        drive_pub_->publish(drive_msg_);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
	current_speed_ = msg->twist.twist.linear.x;
    }

    ~MPC() {}
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPC>());
    rclcpp::shutdown();
    return 0;
}
