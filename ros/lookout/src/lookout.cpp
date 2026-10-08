#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "msgs/msg/peer_table.hpp"
#include <GeographicLib/LocalCartesian.hpp>
#include "Eigen/Core"

class Lookout : public rclcpp::Node {
public:
    Lookout() : Node("lookout") {
        origin_lat_ = this->declare_parameter<double>("origin_lat", 0.0);
        origin_lon_ = this->declare_parameter<double>("origin_lon", 0.0);
        local_proj_ = GeographicLib::LocalCartesian(origin_lat_, origin_lon_, 0.0);

        current_position_ = Eigen::Vector2d(0.0, 0.0);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "state/odom", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&Lookout::odom_callback, this, std::placeholders::_1));
        peer_table_sub_ = this->create_subscription<msgs::msg::PeerTable>(
            "mcu/peer_table", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&Lookout::peer_table_callback, this, std::placeholders::_1));
        peer_detection_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("peers/relative", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    }

private:
    double origin_lat_;
    double origin_lon_;

    GeographicLib::LocalCartesian local_proj_;

    Eigen::Vector2d current_position_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<msgs::msg::PeerTable>::SharedPtr peer_table_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr peer_detection_pub_;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_position_ << msg->pose.pose.position.x, msg->pose.pose.position.y;
    }
    void peer_table_callback(const msgs::msg::PeerTable::SharedPtr msg) {
        geometry_msgs::msg::PolygonStamped peer_detections;
        peer_detections.header.stamp = this->now();
        for (const auto & peer : msg->peers) {
            double x, y, z;
            local_proj_.Forward(peer.lat, peer.lon, 0.0, x, y, z);
            Eigen::Vector2d peer_position(x, y);
            Eigen::Vector2d relative_position = peer_position - current_position_;
            geometry_msgs::msg::Point32 point;
            point.x = relative_position(0);
            point.y = relative_position(1);
            point.z = 0.0;
            peer_detections.polygon.points.push_back(point);
        }
        peer_detection_pub_->publish(peer_detections);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Lookout>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}