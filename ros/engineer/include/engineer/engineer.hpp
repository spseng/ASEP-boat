#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "msgs/msg/motor_pwm.hpp"
#include <boat_defs/boat_defs.h>

class Engineer : public rclcpp::Node {
public:
    Engineer();
    
private:
    double target_v_; // m/s
    double target_w_    ; // rad/s

    double current_v_; // m/s
    double current_w_; // rad/s

    double boat_mass_; // kg
    double boat_inertia_; // kg*m^2
    double boat_length_; // center to thruster, m

    double kp_linear_;
    double ki_linear_;
    double kd_linear_;

    double kp_angular_;
    double ki_angular_;
    double kd_angular_;

    boat::SimplePid linear_pid_;
    boat::SimplePid angular_pid_;

    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
    rclcpp::Time last_time_;
    bool first_ = true;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<msgs::msg::MotorPwm>::SharedPtr motor_pwm_pub_;

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void cmd_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    void update_control();

    static uint16_t force_to_pwm(double force);

    double get_port_force(double linear_acc, double angular_acc);
    double get_starboard_force(double linear_acc, double angular_acc);
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Engineer>());
    rclcpp::shutdown();
    return 0;
}