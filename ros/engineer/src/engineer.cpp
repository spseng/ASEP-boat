#include "engineer/engineer.hpp"

Engineer::Engineer() : Node("engineer") {
    RCLCPP_INFO(this->get_logger(), "Engineer node has been started.");

    target_v_ = 0.0;
    target_w_ = 0.0;
    current_v_ = 0.0;
    current_w_ = 0.0;

    boat_mass_ = this->declare_parameter("boat_mass", 5.0);
    boat_inertia_ = this->declare_parameter("boat_inertia", 1.5);
    boat_length_ = this->declare_parameter("boat_length", 0.5);

    kp_linear_ = this->declare_parameter("kp_linear", 1.0);
    ki_linear_ = this->declare_parameter("ki_linear", 0.0);
    kd_linear_ = this->declare_parameter("kd_linear", 0.0);

    kp_angular_ = this->declare_parameter("kp_angular", 6.0);
    ki_angular_ = this->declare_parameter("ki_angular", 0.0);
    kd_angular_ = this->declare_parameter("kd_angular", 0.0);

    linear_pid_ = boat::SimplePid(kp_linear_, ki_linear_, kd_linear_);
    angular_pid_ = boat::SimplePid(kp_angular_, ki_angular_, kd_angular_);

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "cmd/vel", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&Engineer::cmd_vel_callback, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&Engineer::imu_callback, this, std::placeholders::_1));
    motor_pwm_pub_ = this->create_publisher<msgs::msg::MotorPwm>("cmd/motor_pwm", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
}

void Engineer::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    current_v_ = msg->linear_acceleration.x;
    current_w_ = msg->angular_velocity.z;

    update_control();
}

void Engineer::cmd_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    target_v_ = msg->twist.linear.x;
    target_w_ = msg->twist.angular.z;
}

void Engineer::update_control() {
    auto now = steady_clock_.now();
    double dt = first_ ? 0.02 : (now - last_time_).seconds();
    last_time_ = now;
    first_ = false;

    double linear_acc = target_v_ * 7.8; // TODO: ONLY USE THIS WHILE TESTING MANUALLY
    double angular_acc = target_w_ * 6.5; // TODO: ONLY USE THIS WHILE TESTING MANUALLY

    double port_force = get_port_force(linear_acc, angular_acc);
    double starboard_force = get_starboard_force(linear_acc, angular_acc);

    double port_pwm = force_to_pwm(port_force);
    double starboard_pwm = force_to_pwm(starboard_force);

    msgs::msg::MotorPwm pwm_msg;
    pwm_msg.pwm_port = port_pwm;
    pwm_msg.pwm_starboard = starboard_pwm;
    motor_pwm_pub_->publish(pwm_msg);
}

uint16_t Engineer::force_to_pwm(double force) {
    double u = std::clamp(force / 19.6, -1.0, 1.0);   // F_MAX = 19.6 N, 2kg, TODO: do not hardcode when autonomous
    if (u >= 0.0) return boat::esc::PWM_NEUTRAL_US + u * (boat::esc::PWM_MAX_US - boat::esc::PWM_NEUTRAL_US);
    else          return boat::esc::PWM_NEUTRAL_US + u * (boat::esc::PWM_NEUTRAL_US - boat::esc::PWM_MIN_US);
}

double Engineer::get_port_force(double linear_acc, double angular_acc) {
    return (boat_mass_ * linear_acc - boat_inertia_ * angular_acc / boat_length_) / 2.0;
}

double Engineer::get_starboard_force(double linear_acc, double angular_acc) {
    return (boat_mass_ * linear_acc + boat_inertia_ * angular_acc / boat_length_) / 2.0;
}