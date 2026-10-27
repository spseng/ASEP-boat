#include "mcu_bridge/mcu_bridge.hpp"

McuBridge::McuBridge() : Node("mcu_bridge") {
    serial_port_ = this->declare_parameter("mcu_serial_port", "/dev/mcu");

    link_ = wirelink::Link();

    init_serial();

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "state/odom", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&McuBridge::odom_callback, this, std::placeholders::_1));
    scalar_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "scalar", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&McuBridge::scalar_callback, this, std::placeholders::_1));
    mode_sub_ = this->create_subscription<msgs::msg::Mode>(
        "mode", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), std::bind(&McuBridge::mode_callback, this, std::placeholders::_1));
    motor_pwm_sub_ = this->create_subscription<msgs::msg::MotorPwm>(
        "cmd/motor_pwm", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&McuBridge::motor_pwm_callback, this, std::placeholders::_1));
    config_sub_ = this->create_subscription<msgs::msg::Config>(
        "config", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), std::bind(&McuBridge::config_callback, this, std::placeholders::_1));

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    command_frame_pub_ = this->create_publisher<msgs::msg::CommandFrame>("mcu/command_frame", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    mcu_status_pub_ = this->create_publisher<msgs::msg::McuStatus>("mcu/status", rclcpp::QoS(rclcpp::KeepLast(5)).reliable());
    peer_table_pub_ = this->create_publisher<msgs::msg::PeerTable>("mcu/peer_table", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    gps_fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    gps_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("gps/vel", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    raw_scalar_pub_ = this->create_publisher<std_msgs::msg::Float64>("raw_scalar", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());

    rx_timer_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&McuBridge::on_rx_tick, this));
}

McuBridge::~McuBridge() {
    if (serial_fd_ >= 0) ::close(serial_fd_);
}

void McuBridge::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    return;
}

void McuBridge::scalar_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    return;
}

void McuBridge::mode_callback(const msgs::msg::Mode::SharedPtr msg) {
    wirelink::msg::serial::Heartbeat wirelink_msg;
    wirelink_msg.ros_mode = msg->mode;
    wirelink_msg.ros_armed = msg->armed;

    wirelink::Frame frame;
    if (!wirelink::codec::pack(wirelink_msg, frame)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to pack Heartbeat message");
        return;
    }
    auto packet = wirelink::framing::wrap(frame, mode_seq_);
    if (!packet) {
        RCLCPP_ERROR(this->get_logger(), "Failed to wrap Heartbeat frame");
        return;
    }
    if (!send_packet(*packet)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send Heartbeat packet");
        return;
    }
    mode_seq_++;
}

void McuBridge::config_callback(const msgs::msg::Config::SharedPtr msg) {
    wirelink::msg::serial::Config wirelink_msg;
    wirelink_msg.boat_id = msg->boat_id;
    wirelink_msg.gate_enable = msg->gate_enable;

    wirelink::Frame frame;
    if (!wirelink::codec::pack(wirelink_msg, frame)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to pack Config message");
        return;
    }
    auto packet = wirelink::framing::wrap(frame, config_seq_);
    if (!packet) {
        RCLCPP_ERROR(this->get_logger(), "Failed to wrap Config frame");
        return;
    }
    if (!send_packet(*packet)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send Config packet");
        return;
    }
    config_seq_++;
}

void McuBridge::motor_pwm_callback(const msgs::msg::MotorPwm::SharedPtr msg) {
    wirelink::msg::serial::MotorCommand wirelink_msg;
    wirelink_msg.pwm_port = msg->pwm_port;
    wirelink_msg.pwm_starboard = msg->pwm_starboard;
    wirelink::Frame frame;
    if (!wirelink::codec::pack(wirelink_msg, frame)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to pack MotorCommand message");
        return;
    }
    auto packet = wirelink::framing::wrap(frame, motor_pwm_seq_);
    if (!packet) {
        RCLCPP_ERROR(this->get_logger(), "Failed to wrap MotorCommand frame");
        return;
    }
    if (!send_packet(*packet)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send MotorCommand packet");
        return;
    }
    motor_pwm_seq_++;
    return;
}

void McuBridge::init_serial() {
    serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s: %s", serial_port_.c_str(), strerror(errno));
        return;
    }

    termios tio{};
    tcgetattr(serial_fd_, &tio);
    cfmakeraw(&tio);
    cfsetispeed(&tio, to_speed(boat::serial::BAUD_RATE).value_or(B115200));
    cfsetospeed(&tio, to_speed(boat::serial::BAUD_RATE).value_or(B115200));
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;
    tcsetattr(serial_fd_, TCSANOW, &tio);
    tcflush(serial_fd_, TCIOFLUSH);
}

bool McuBridge::send_packet(wirelink::bytes::Packet& packet) {
    std::lock_guard<std::mutex> lock(tx_mutex_);
    const uint8_t* data = packet.data.data();
    size_t len = packet.len;
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = ::write(serial_fd_, data + sent, len - sent);
        if (n > 0) { sent += n; continue; }
        if (n < 0 && errno == EAGAIN) {
            pollfd p{serial_fd_, POLLOUT, 0};
            poll(&p, 1, 10);
            continue;
        }
        return false;
    }
    return true;
}

void McuBridge::on_rx_tick() {
    uint8_t buf[256];
    ssize_t n;
    while ((n = ::read(serial_fd_, buf, sizeof(buf))) > 0) {
        for (ssize_t i = 0; i < n; ++i) {
            auto frame = link_.feed(buf[i]);
            if (frame) {
                dispatch(*frame);
            }
        }
    }
}

void McuBridge::dispatch(const wirelink::Frame& f) {
    switch (static_cast<wirelink::msg::serial::MsgType>(f.type)) {
        case wirelink::msg::serial::MsgType::Attitude: {
            wirelink::msg::serial::Attitude wirelink_msg;
            if (!wirelink::codec::unpack(f, wirelink_msg)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to unpack Attitude message");
                return;
            }
            sensor_msgs::msg::Imu msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = "base_link";

            msg.orientation_covariance[0] = msg.orientation_covariance[4] = msg.orientation_covariance[8] = 0.0076;
            msg.angular_velocity_covariance[0] = msg.angular_velocity_covariance[4] = msg.angular_velocity_covariance[8] = 1e-4;
            msg.linear_acceleration_covariance[0] = -1;

            const double c = yaw_rot_c_, s = yaw_rot_s_;
            const double w = wirelink_msg.quat_w, x = wirelink_msg.quat_x, y = wirelink_msg.quat_y, z = wirelink_msg.quat_z;
            msg.orientation.w = c * w - s * z;
            msg.orientation.x = c * x - s * y;
            msg.orientation.y = c * y + s * x;
            msg.orientation.z = c * z + s * w;

            msg.angular_velocity.x = wirelink_msg.ang_vel_x;
            msg.angular_velocity.y = wirelink_msg.ang_vel_y;
            msg.angular_velocity.z = wirelink_msg.ang_vel_z;
            msg.linear_acceleration.x = wirelink_msg.lin_acc_x;
            msg.linear_acceleration.y = wirelink_msg.lin_acc_y;
            msg.linear_acceleration.z = wirelink_msg.lin_acc_z;
            imu_pub_->publish(msg);
        } break;
        case wirelink::msg::serial::MsgType::GPS: {
            wirelink::msg::serial::GPS wirelink_msg;
            if (!wirelink::codec::unpack(f, wirelink_msg)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to unpack GPS message");
                return;
            }
            sensor_msgs::msg::NavSatFix fix_msg;
            fix_msg.header.stamp = this->now();
            fix_msg.header.frame_id = "base_link";
            fix_msg.latitude = static_cast<double>(wirelink_msg.lat) / 1e7;
            fix_msg.longitude = static_cast<double>(wirelink_msg.lon) / 1e7;
            fix_msg.status.status = wirelink_msg.fix_quality == 0 ? sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX : sensor_msgs::msg::NavSatStatus::STATUS_FIX;
            fix_msg.status.service = 1; // GPS

            constexpr double UERE_M = 4.0;
            const double sigma = wirelink_msg.hdop * UERE_M;
            const double var = sigma * sigma;
            fix_msg.position_covariance = {var, 0.0, 0.0,
                                            0.0, var, 0.0,
                                            0.0, 0.0, 4.0 * var};
            fix_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
            gps_fix_pub_->publish(fix_msg);

            geometry_msgs::msg::TwistStamped vel_msg;
            vel_msg.header.stamp = this->now();
            const double cog = wirelink_msg.course_over_ground * M_PI / 180.0;
            vel_msg.twist.linear.x = wirelink_msg.speed_over_ground * std::sin(cog); 
            vel_msg.twist.linear.y = wirelink_msg.speed_over_ground * std::cos(cog);
            gps_vel_pub_->publish(vel_msg);
        } break;
        case wirelink::msg::serial::MsgType::PeerTable: {
            wirelink::msg::serial::PeerTable wirelink_msg;
            if (!wirelink::codec::unpack(f, wirelink_msg)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to unpack PeerTable message");
                return;
            }
            msgs::msg::PeerTable msg;
            msg.header.stamp = this->now();
            for (size_t i = 0; i < wirelink_msg.peer_entries.count; ++i) {
                const auto& entry = wirelink_msg.peer_entries.items[i];
                msgs::msg::Peer peer_entry;
                peer_entry.lat = static_cast<double>(entry.lat) / 1e7;
                peer_entry.lon = static_cast<double>(entry.lon) / 1e7;

                msg.peers.push_back(peer_entry);
            }
            peer_table_pub_->publish(msg);
        }; break;
        case wirelink::msg::serial::MsgType::OwnScalar: {}; break; // temprature probe yet to be implemented
        case wirelink::msg::serial::MsgType::ReceivedCommand: {
            wirelink::msg::serial::ReceivedCommand wirelink_msg;
            if (!wirelink::codec::unpack(f, wirelink_msg)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to unpack ReceivedCommand message");
                return;
            }
            msgs::msg::CommandFrame msg;
            msg.header.stamp = this->now();
            msg.type = wirelink_msg.lora_type;
            msg.payload.assign(wirelink_msg.payload.data.begin(), wirelink_msg.payload.data.begin() + wirelink_msg.payload.len);
            command_frame_pub_->publish(msg);
        }; break;
        case wirelink::msg::serial::MsgType::Status: {}; break;
    }
    return;
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<McuBridge>());
    rclcpp::shutdown();
    return 0;
}