#include "swarm_captain/swarm_captain.hpp"

SwarmCaptain::SwarmCaptain() : Node("swarm_captain") {
    k_avoidance_ = this->declare_parameter("k_avoidance", 1.0);
    k_cohesion_ = this->declare_parameter("k_cohesion", 1.0);
    k_descent_ = this->declare_parameter("k_descent", 1.0);

    avoidance_radius_ = this->declare_parameter("avoidance_radius", 1.0);
    cohesion_saturation_distance_ = this->declare_parameter("cohesion_saturation_distance", 1.0);
    cohesion_max_ = this->declare_parameter("cohesion_max", 1.0);
    eps_ = this->declare_parameter("eps", 1e-6);

    prev_scalar_ = 0.0;
    curr_scalar_ = 0.0;

    scalar_count_ = 0;

    peer_relative_positions_.reserve(boat::ids::BOAT_ID_MAX);
    peer_relative_positions_.clear();

    prev_odom_ = Eigen::Vector2d(0.0, 0.0);
    curr_odom_ = Eigen::Vector2d(0.0, 0.0);
    prev_scalar_acquired_odom_ = Eigen::Vector2d(0.0, 0.0);
    curr_scalar_acquired_odom_ = Eigen::Vector2d(0.0, 0.0);

    peer_detection_sub_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
        "peers/relative", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&SwarmCaptain::peer_detection_callback, this, std::placeholders::_1));
    scalar_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "scalar", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&SwarmCaptain::scalar_callback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "state/odom", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&SwarmCaptain::odom_callback, this, std::placeholders::_1));

    net_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("cmd/desired_vel", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());

    // for debugging
    avoidance_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("debug/avoidance", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    cohesion_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("debug/cohesion", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    descent_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("debug/descent", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
}

void SwarmCaptain::scalar_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    prev_scalar_ = curr_scalar_;
    curr_scalar_ = msg->data;
    prev_scalar_acquired_odom_ = curr_scalar_acquired_odom_;
    curr_scalar_acquired_odom_ = curr_odom_;
    scalar_count_++;
}

void SwarmCaptain::peer_detection_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
    peer_relative_positions_.clear();
    for (const auto& point : msg->polygon.points) {
        peer_relative_positions_.push_back(Eigen::Vector2d(point.x, point.y));
    }

    Eigen::Vector2d weighted_avoidance = k_avoidance_ * get_avoidance();
    Eigen::Vector2d weighted_cohesion = k_cohesion_ * get_cohesion();
    Eigen::Vector2d weighted_descent = k_descent_ * get_descent();
    
    Eigen::Vector2d net = weighted_avoidance + weighted_cohesion + weighted_descent;

    geometry_msgs::msg::Vector3Stamped net_msg;
    net_msg.header.stamp = this->now();
    net_msg.vector.x = net(0);
    net_msg.vector.y = net(1);
    net_msg.vector.z = 0.0;
    net_pub_->publish(net_msg);

    // for debugging
    geometry_msgs::msg::Vector3Stamped avoidance_msg;
    avoidance_msg.header.stamp = this->now();
    avoidance_msg.vector.x = weighted_avoidance(0);
    avoidance_msg.vector.y = weighted_avoidance(1);
    avoidance_msg.vector.z = 0.0;
    avoidance_pub_->publish(avoidance_msg);

    geometry_msgs::msg::Vector3Stamped cohesion_msg;
    cohesion_msg.header.stamp = this->now();
    cohesion_msg.vector.x = weighted_cohesion(0);
    cohesion_msg.vector.y = weighted_cohesion(1);
    cohesion_msg.vector.z = 0.0;
    cohesion_pub_->publish(cohesion_msg);

    geometry_msgs::msg::Vector3Stamped descent_msg;
    descent_msg.header.stamp = this->now();
    descent_msg.vector.x = weighted_descent(0);
    descent_msg.vector.y = weighted_descent(1);
    descent_msg.vector.z = 0.0;
    descent_pub_->publish(descent_msg);
}

void SwarmCaptain::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    prev_odom_ = curr_odom_;
    curr_odom_ = Eigen::Vector2d(msg->pose.pose.position.x, msg->pose.pose.position.y);
}

// vector math functions
Eigen::Vector2d SwarmCaptain::get_avoidance() {
    Eigen::Vector2d avoidance(0.0, 0.0);
    for (const auto& peer_pos : peer_relative_positions_) {
        double distance = peer_pos.norm();
        if (distance < avoidance_radius_) {
            avoidance -= peer_pos * (1.0 / (distance + eps_));
        }
    }
    return avoidance;
}

Eigen::Vector2d SwarmCaptain::get_cohesion() {
    Eigen::Vector2d cohesion(0.0, 0.0);
    for (const auto& peer_pos : peer_relative_positions_) {
        double distance = peer_pos.norm();
        double saturation = distance / (distance + cohesion_saturation_distance_);
        double magnitude = cohesion_max_ * saturation;
        cohesion += peer_pos.normalized() * magnitude;
    }
    return cohesion;
}

Eigen::Vector2d SwarmCaptain::get_descent() {
    Eigen::Vector2d descent(0.0, 0.0);
    if (scalar_count_ < 2) {
        return descent;
    }
    Eigen::Vector2d delta_odom = curr_scalar_acquired_odom_ - prev_scalar_acquired_odom_;
    double step_sq = delta_odom.dot(delta_odom);

    if (step_sq > eps_) {
        double delta_scalar = curr_scalar_ - prev_scalar_;
        double projectoin_scale = delta_scalar / (step_sq + eps_);
        Eigen::Vector2d projection_gradient = delta_odom * projectoin_scale;
        descent = -projection_gradient;
    }
    return descent;
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwarmCaptain>());
    rclcpp::shutdown();
    return 0;
}