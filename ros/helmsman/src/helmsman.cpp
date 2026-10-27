#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"

class Helmsman : public rclcpp::Node {
public:
    Helmsman() : Node("helmsman") {
        RCLCPP_INFO(this->get_logger(), "Helmsman node has been started.");

        attitude_sub_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
            "state/odom", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&Helmsman::attitude_callback, this, std::placeholders::_1));
        desired_vel_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "cmd/desired_vel", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&Helmsman::desired_vel_callback, this, std::placeholders::_1));
        cmd_vel_auto_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd/vel_auto", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    }
private:
    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr attitude_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr desired_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_auto_pub_;

    geometry_msgs::msg::QuaternionStamped current_attitude_;

    double current_yaw_ = 0.0;
    const double linear_threshold_deg_ = 10.0; // Threshold angle in degrees for
    const double heading_deadzone_eps_ = 0.01; // Deadzone for heading control
    const double heading_tiebreak_rate_ = 0.1; // Rate to apply

    geometry_msgs::msg::TwistStamped desired_to_cmd_vel(const geometry_msgs::msg::Vector3Stamped & desired_vel) {
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.stamp = this->now();
        cmd_vel.header.frame_id = "base_link";

        const double vx = desired_vel.vector.x, vy = desired_vel.vector.y;
        const double c = std::cos(current_yaw_), s = std::sin(current_yaw_);
        const double fx = vx * c + vy * s;
        const double fy = -vx * s + vy * c;
        const double magnitude = std::hypot(fx, fy);
        const double threshold_x = magnitude * std::cos(linear_threshold_deg_ * M_PI / 180.0);
        cmd_vel.twist.linear.x = fx > threshold_x ? fx : 0.0;
        if (std::abs(fy) < heading_deadzone_eps_ && fx < 0.0) {
            cmd_vel.twist.angular.z = heading_tiebreak_rate_;
        } else {
            cmd_vel.twist.angular.z = fy;
        }
        return cmd_vel;
    };

    void attitude_callback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg) {
        current_attitude_ = *msg;
        const auto& q = current_attitude_.quaternion;
        current_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    void desired_vel_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        geometry_msgs::msg::TwistStamped cmd_vel = desired_to_cmd_vel(*msg);
        cmd_vel_auto_pub_->publish(cmd_vel);
    }
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Helmsman>());
    rclcpp::shutdown();
    return 0;
}