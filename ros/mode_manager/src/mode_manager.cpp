#include "rclcpp/rclcpp.hpp"
#include <wirelink/wirelink.h>
#include <boat_defs/boat_defs.h>
#include "msgs/msg/command_frame.hpp"
#include "msgs/msg/mcu_status.hpp"
#include "msgs/msg/mode.hpp"
#include "msgs/msg/heart_beat.hpp"
#include "msgs/msg/config.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class ModeManager : public rclcpp::Node {
public:
    ModeManager() : Node("mode_manager") {
        my_id_ = this->declare_parameter("my_id", 1);

        command_frame_sub_ = this->create_subscription<msgs::msg::CommandFrame>(
            "mcu/command_frame", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), std::bind(&ModeManager::command_frame_callback, this, std::placeholders::_1));
        mcu_status_sub_ = this->create_subscription<msgs::msg::McuStatus>(
            "mcu/status", rclcpp::QoS(rclcpp::KeepLast(5)).reliable(), std::bind(&ModeManager::mcu_status_callback, this, std::placeholders::_1));
        cmd_vel_auto_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "cmd/vel_auto", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&ModeManager::cmd_vel_auto_callback, this, std::placeholders::_1));

        mode_pub_ = this->create_publisher<msgs::msg::Mode>("mode", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
        heart_beat_pub_ = this->create_publisher<msgs::msg::HeartBeat>("heart_beat", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd/vel", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
        config_pub_ = this->create_publisher<msgs::msg::Config>("config", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

        pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {
            publish_mode();
            publish_config();
        });
    }

private:
    uint8_t my_id_;

    boat::mode::Mode current_mode_ = boat::mode::Mode::MANUAL;
    boat::mode::ArmedState armed_state_ = boat::mode::ArmedState::DISARMED;

    bool gate_reenable_requested_ = false;

    rclcpp::Subscription<msgs::msg::CommandFrame>::SharedPtr command_frame_sub_;
    rclcpp::Subscription<msgs::msg::McuStatus>::SharedPtr mcu_status_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_auto_sub_;

    rclcpp::Publisher<msgs::msg::Mode>::SharedPtr mode_pub_;
    rclcpp::Publisher<msgs::msg::HeartBeat>::SharedPtr heart_beat_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<msgs::msg::Config>::SharedPtr config_pub_;

    rclcpp::TimerBase::SharedPtr pub_timer_;

    double last_manual_linear_x_ = 0.0;
    double last_manual_angular_z_ = 0.0;
    double last_auto_linear_x_ = 0.0;
    double last_auto_angular_z_ = 0.0;

    void command_frame_callback(const msgs::msg::CommandFrame::SharedPtr msg) {
        if (msg->payload.size() > wirelink::msg::lora::MAX_LORA_PAYLOAD) {
            RCLCPP_ERROR(this->get_logger(), "Received CommandFrame with payload size exceeding MAX_LORA_PAYLOAD");
            return;
        }
        wirelink::bytes::Payload payload;
        std::copy(msg->payload.begin(), msg->payload.end(), payload.data.begin());
        payload.len = msg->payload.size();

        switch (static_cast<wirelink::msg::lora::MsgType>(msg->type)) {
            case wirelink::msg::lora::MsgType::Command: {
                wirelink::msg::lora::Command wirelink_msg;
                if (!wirelink::codec::unpack_bytes(msg->payload.data(), msg->payload.size(), wirelink_msg)) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to unpack Command message");
                    return;
                }
                if (wirelink_msg.rx_id == my_id_) {
                    last_manual_linear_x_ = wirelink_msg.lin_vel / 1000.0; // Convert mm/s to m/s
                    last_manual_angular_z_ = wirelink_msg.ang_vel / 1000.0; // Convert mrad/s to rad/s, no flip!!!
                    publish_cmd_vel();
                }
            } break;
            case wirelink::msg::lora::MsgType::Heartbeat: {} break;
            case wirelink::msg::lora::MsgType::Reenable: {
                wirelink::msg::lora::Reenable wirelink_msg;
                if (!wirelink::codec::unpack_bytes(msg->payload.data(), msg->payload.size(), wirelink_msg)) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to unpack Reenable message");
                    return;
                }
                if (wirelink_msg.rx_id != my_id_ && wirelink_msg.rx_id != boat::ids::BROADCAST_ID) {
                    RCLCPP_DEBUG(this->get_logger(), "Received Reenable message for a different boat ID: %d", wirelink_msg.rx_id);
                    return;
                }
                gate_reenable_requested_ = true;
            } break;
            case wirelink::msg::lora::MsgType::SetMode: {
                wirelink::msg::lora::SetMode wirelink_msg;
                if (!wirelink::codec::unpack_bytes(msg->payload.data(), msg->payload.size(), wirelink_msg)) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to unpack SetMode message");
                    return;
                }
                if (wirelink_msg.rx_id != my_id_ && wirelink_msg.rx_id != boat::ids::BROADCAST_ID) {
                    RCLCPP_DEBUG(this->get_logger(), "Received SetMode message for a different boat ID: %d", wirelink_msg.rx_id);
                    return;
                }
                current_mode_ = static_cast<boat::mode::Mode>(wirelink_msg.mode);
                armed_state_ = static_cast<boat::mode::ArmedState>(wirelink_msg.armed);

                last_manual_linear_x_ = 0.0;
                last_manual_angular_z_ = 0.0;
                last_auto_linear_x_ = 0.0;
                last_auto_angular_z_ = 0.0;
                publish_cmd_vel();
            } break;
        }
    }

    void mcu_status_callback(const msgs::msg::McuStatus::SharedPtr msg) {
        // Handle incoming MCU status messages
        RCLCPP_INFO(this->get_logger(), "Received McuStatus message");
    }

    void cmd_vel_auto_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        last_auto_linear_x_ = msg->twist.linear.x;
        last_auto_angular_z_ = msg->twist.angular.z;
        publish_cmd_vel();
    }

    void publish_config() {
        msgs::msg::Config config_msg;
        config_msg.boat_id = my_id_;
        config_msg.gate_enable = gate_reenable_requested_;
        gate_reenable_requested_ = false;
        config_pub_->publish(config_msg);
    }

    void publish_mode() {
        msgs::msg::Mode mode_msg;
        mode_msg.header.stamp = this->now();
        mode_msg.mode = static_cast<uint8_t>(current_mode_);
        mode_msg.armed = static_cast<uint8_t>(armed_state_);
        mode_pub_->publish(mode_msg);
    }

    void publish_cmd_vel() {
        geometry_msgs::msg::TwistStamped cmd_vel_msg;
        cmd_vel_msg.header.stamp = this->now();
        cmd_vel_msg.header.frame_id = "base_link";
        
        if (armed_state_ == boat::mode::ArmedState::ARMED) {
            if (current_mode_ == boat::mode::Mode::MANUAL) {
                cmd_vel_msg.twist.linear.x = last_manual_linear_x_;
                cmd_vel_msg.twist.angular.z = last_manual_angular_z_;
            }
            else if (current_mode_ == boat::mode::Mode::AUTONOMOUS) {
                // For autonomous mode, you would typically get the desired velocities from another source
                // Here we just set them to zero for demonstration purposes
                cmd_vel_msg.twist.linear.x = last_auto_linear_x_;
                cmd_vel_msg.twist.angular.z = last_auto_angular_z_;
            }
            else {
                cmd_vel_msg.twist.linear.x = 0.0;
                cmd_vel_msg.twist.angular.z = 0.0;
            }
        } else {
            cmd_vel_msg.twist.linear.x = 0.0;
            cmd_vel_msg.twist.angular.z = 0.0;
        }

        cmd_vel_pub_->publish(cmd_vel_msg);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModeManager>());
    rclcpp::shutdown();
    return 0;
}