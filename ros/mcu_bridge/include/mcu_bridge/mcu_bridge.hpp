#include "rclcpp/rclcpp.hpp"
#include <wirelink/wirelink.h>
#include "std_msgs/msg/float64.hpp"
#include "msgs/msg/mode.hpp"
#include "msgs/msg/motor_pwm.hpp"
#include "msgs/msg/heart_beat.hpp"
#include "msgs/msg/command_frame.hpp"
#include "msgs/msg/mcu_status.hpp"
#include "msgs/msg/peer_table.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

class McuBridge : public rclcpp::Node {
public:
    McuBridge() : Node("mcu_bridge"){} // TODO: Implement
private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr scalar_sub_;
    rclcpp::Subscription<msgs::msg::Mode>::SharedPtr mode_sub_;
    rclcpp::Subscription<msgs::msg::HeartBeat>::SharedPtr heart_beat_sub_;
    rclcpp::Subscription<msgs::msg::MotorPwm>::SharedPtr motor_pwm_sub_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<msgs::msg::CommandFrame>::SharedPtr command_frame_pub_;
    rclcpp::Publisher<msgs::msg::McuStatus>::SharedPtr mcu_status_pub_;
    rclcpp::Publisher<msgs::msg::PeerTable>::SharedPtr peer_table_pub_;
};