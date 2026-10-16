#include "mcu_bridge/mcu_bridge.hpp"

McuBridge::McuBridge() : Node("mcu_bridge") {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "state/odom", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&McuBridge::odom_callback, this, std::placeholders::_1));
    scalar_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "scalar", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&McuBridge::scalar_callback, this, std::placeholders::_1));
    mode_sub_ = this->create_subscription<msgs::msg::Mode>(
        "mode", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), std::bind(&McuBridge::mode_callback, this, std::placeholders::_1));
    heart_beat_sub_ = this->create_subscription<msgs::msg::HeartBeat>(
        "heart_beat", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&McuBridge::heart_beat_callback, this, std::placeholders::_1));
    motor_pwm_sub_ = this->create_subscription<msgs::msg::MotorPwm>(
        "cmd/motor_pwm", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&McuBridge::motor_pwm_callback, this, std::placeholders::_1));

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    command_frame_pub_ = this->create_publisher<msgs::msg::CommandFrame>("mcu/command_frame", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
    mcu_status_pub_ = this->create_publisher<msgs::msg::McuStatus>("mcu/status", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort());
    peer_table_pub_ = this->create_publisher<msgs::msg::PeerTable>("mcu/peer_table", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    gps_fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    gps_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("gps/vel", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<McuBridge>());
    rclcpp::shutdown();
    return 0;
}