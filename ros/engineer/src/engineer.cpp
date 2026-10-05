#include "rclcpp/rclcpp.hpp"
#include "msgs/msg/motor_pwm.hpp"

class Engineer : public rclcpp::Node {
public:
    Engineer() : Node("engineer") {
        RCLCPP_INFO(this->get_logger(), "Engineer node has been started.");
    }

private:
    
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Engineer>());
    rclcpp::shutdown();
    return 0;
}