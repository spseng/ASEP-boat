#include "mcu_bridge/mcu_bridge.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<McuBridge>());
    rclcpp::shutdown();
    return 0;
}