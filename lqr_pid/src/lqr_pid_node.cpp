void LQRPID::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    geometry_msgs::msg::PoseStamped pose_in, pose_out;
    pose_in.header = msg->header;
    pose_in.pose = msg->pose.pose;

    try {
        tf_buffer_->transform(pose_in, pose_out, global_refFrame);  // 예: "map"
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
        return;
    }

    double x = pose_out.pose.position.x;
    double y = pose_out.pose.position.y;

    tf2::Quaternion q(
        pose_out.pose.orientation.x,
        pose_out.pose.orientation.y,
        pose_out.pose.orientation.z,
        pose_out.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    double velocity = msg->twist.twist.linear.x;

    publish_message(x, y, yaw, velocity);
}

// tf 초기화
tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
