#include "rclcpp/rclcpp.hpp"
#include <wirelink/wirelink.h>
#include <boat_defs/boat_defs.h>
#include "std_msgs/msg/float64.hpp"
#include "msgs/msg/mode.hpp"
#include "msgs/msg/motor_pwm.hpp"
#include "msgs/msg/heart_beat.hpp"
#include "msgs/msg/command_frame.hpp"
#include "msgs/msg/mcu_status.hpp"
#include "msgs/msg/peer_table.hpp"
#include "msgs/msg/config.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <cerrno>
#include <mutex>


class McuBridge : public rclcpp::Node {
public:
    McuBridge();

    ~McuBridge() override;
private:
    wirelink::Link link_;

    std::string serial_port_;
    int serial_fd_ = -1;
    std::mutex tx_mutex_;

    void init_serial();

    bool send_packet(wirelink::bytes::Packet& packet);

    void on_rx_tick();
    void dispatch(const wirelink::Frame& f);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr scalar_sub_;
    rclcpp::Subscription<msgs::msg::Mode>::SharedPtr mode_sub_;
    rclcpp::Subscription<msgs::msg::MotorPwm>::SharedPtr motor_pwm_sub_;
    rclcpp::Subscription<msgs::msg::Config>::SharedPtr config_sub_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<msgs::msg::CommandFrame>::SharedPtr command_frame_pub_;
    rclcpp::Publisher<msgs::msg::McuStatus>::SharedPtr mcu_status_pub_;
    rclcpp::Publisher<msgs::msg::PeerTable>::SharedPtr peer_table_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_fix_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr gps_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr raw_scalar_pub_;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scalar_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void mode_callback(const msgs::msg::Mode::SharedPtr msg);
    void motor_pwm_callback(const msgs::msg::MotorPwm::SharedPtr msg);
    void config_callback(const msgs::msg::Config::SharedPtr msg);

    uint8_t odom_seq_ = 0;
    uint8_t scalar_seq_ = 0;
    uint8_t mode_seq_ = 0;
    uint8_t heart_beat_seq_ = 0;
    uint8_t motor_pwm_seq_ = 0;
    uint8_t config_seq_ = 0;

    rclcpp::TimerBase::SharedPtr rx_timer_;
};

std::optional<speed_t> to_speed(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:     return std::nullopt;
    }
}