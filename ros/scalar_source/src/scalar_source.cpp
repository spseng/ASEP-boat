#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"

#include <array>
#include <cmath>
#include <vector>

class ScalarSource : public rclcpp::Node{
public:
	ScalarSource() : Node("scalar_source") {
		const char* keys[4] = {"field_x", "field_y", "field_amp", "field_sigma"};

		for (size_t i = 0; i < 4; ++i) {
			simulated_peaks_[i] = this->declare_parameter<std::vector<double>>(keys[i], std::vector<double>{});
		}
		const size_t n = simulated_peaks_[0].size();
		bool ok = n > 0;
		for (const auto& v : simulated_peaks_) ok = ok && v.size() == n;
		if (!ok) {
			RCLCPP_ERROR(this->get_logger(), "field_x/y/amp/sigma must be non-empty and the same length");
			for (auto& v : simulated_peaks_) v.clear();
		} else {
			RCLCPP_INFO(this->get_logger(), "Simulated field: %zu Gaussians", n);
		}

		odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
			"state/odom", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&ScalarSource::odom_callback, this, std::placeholders::_1));
		scalar_pub_ = this->create_publisher<std_msgs::msg::Float64>("scalar", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
	}
private:

	std::array<std::vector<double>, 4> simulated_peaks_; // x, y, amp, sigma

	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr scalar_pub_;

	void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
		const auto& [fx, fy, famp, fsigma] = simulated_peaks_;
		if (famp.empty()) return;

		const double x = msg->pose.pose.position.x;
		const double y = msg->pose.pose.position.y;
		double value = 0.0;
		for (size_t i = 0; i < famp.size(); ++i) {
			const double dx = x - fx[i];
			const double dy = y - fy[i];
			value += famp[i] * std::exp(-(dx*dx + dy*dy) / (2.0 * fsigma[i] * fsigma[i]));
		}

		std_msgs::msg::Float64 scalar_msg;
		scalar_msg.data = value;
		scalar_pub_->publish(scalar_msg);
	}
};

int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ScalarSource>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
