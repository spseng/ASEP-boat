#include "rclcpp/rclcpp.hpp"
#include <wirelink/wirelink.h>
#include <boat_defs/boat_defs.h>
#include "msgs/msg/command_frame.hpp"
#include "msgs/msg/mcu_status.hpp"
#include "msgs/msg/mode.hpp"
#include "msgs/msg/heart_beat.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class ModeManager : public rclcpp::Node {
public:
    ModeManager() : Node("mode_manager") {
        my_id_ = this->declare_parameter("my_id", 1);

        command_frame_sub_ = this->create_subscription<msgs::msg::CommandFrame>(
            "mcu/command_frame", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), std::bind(&ModeManager::command_frame_callback, this, std::placeholders::_1));
        mcu_status_sub_ = this->create_subscription<msgs::msg::McuStatus>(
            "mcu/status", rclcpp::QoS(rclcpp::KeepLast(5)).reliable(), std::bind(&ModeManager::mcu_status_callback, this, std::placeholders::_1));

        mode_pub_ = this->create_publisher<msgs::msg::Mode>("mode", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
        heart_beat_pub_ = this->create_publisher<msgs::msg::HeartBeat>("heart_beat", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd/vel", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    }



private:
    uint8_t my_id_;

    boat::mode::Mode current_mode_ = boat::mode::Mode::MANUAL;
    boat::mode::ArmedState armed_state_ = boat::mode::ArmedState::DISARMED;

    rclcpp::Subscription<msgs::msg::CommandFrame>::SharedPtr command_frame_sub_;
    rclcpp::Subscription<msgs::msg::McuStatus>::SharedPtr mcu_status_sub_;

    rclcpp::Publisher<msgs::msg::Mode>::SharedPtr mode_pub_;
    rclcpp::Publisher<msgs::msg::HeartBeat>::SharedPtr heart_beat_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;

    void command_frame_callback(const msgs::msg::CommandFrame::SharedPtr msg) {
        if (msg->payload.size() > wirelink::msg::lora::MAX_LORA_PAYLOAD) {
            RCLCPP_ERROR(this->get_logger(), "Received CommandFrame with payload size exceeding MAX_LORA_PAYLOAD");
            return;
        }
        wirelink::bytes::Payload payload;
        std::copy(msg->payload.begin(), msg->payload.end(), payload.data.begin());
        payload.len = msg->payload.size();

        switch (static_cast<wirelink::msg::lora::MsgType>(msg->type)) { // BAD!!!! THIS NEEDS TO BE FIXED WHEN AUTONOMOUS IS IMPLEMENTED
            case wirelink::msg::lora::MsgType::Command: {
                wirelink::msg::lora::Command wirelink_msg;
                if (!wirelink::codec::unpack_bytes(msg->payload.data(), msg->payload.size(), wirelink_msg)) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to unpack Command message");
                    return;
                }
                if (wirelink_msg.rx_id == my_id_) {
                    geometry_msgs::msg::TwistStamped cmd_vel_msg;
                    cmd_vel_msg.header.stamp = this->now();
                    cmd_vel_msg.twist.linear.x = wirelink_msg.lin_vel / 1000.0; // Convert mm/s to m/s
                    cmd_vel_msg.twist.angular.z = -wirelink_msg.ang_vel / 1000.0; // Convert mrad/s to rad/s, maybe flip because ENU
                    cmd_vel_pub_->publish(cmd_vel_msg);
                }
            } break;
        }
    }
    void mcu_status_callback(const msgs::msg::McuStatus::SharedPtr msg) {
        // Handle incoming MCU status messages
        RCLCPP_INFO(this->get_logger(), "Received McuStatus message");
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModeManager>());
    rclcpp::shutdown();
    return 0;
}