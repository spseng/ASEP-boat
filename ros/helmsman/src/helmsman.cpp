#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"

class Helmsman : public rclcpp::Node {
public:
    Helmsman() : Node("helmsman") {
        attitude_sub_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
            "state/attitude", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&Helmsman::attitude_callback, this, std::placeholders::_1));
        desired_vel_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "cmd/desired_vel", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&Helmsman::desired_vel_callback, this, std::placeholders::_1));
        cmd_vel_auto_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_auto", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    }
private:
    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr attitude_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr desired_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_auto_pub_;

    geometry_msgs::msg::QuaternionStamped current_attitude_;

    geometry_msgs::msg::TwistStamped desired_to_cmd_vel(const geometry_msgs::msg::Vector3Stamped & desired_vel) {
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.stamp = this->now();
        cmd_vel.twist.linear.x = desired_vel.vector.x;
        cmd_vel.twist.linear.y = 0.0;
        cmd_vel.twist.linear.z = 0.0;
        cmd_vel.twist.angular.x = 0.0;
        cmd_vel.twist.angular.y = 0.0;
        //TODO implement
        cmd_vel.twist.angular.z = desired_vel.vector.y;
        return cmd_vel;
    };

    void attitude_callback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg) {
        current_attitude_ = *msg;
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