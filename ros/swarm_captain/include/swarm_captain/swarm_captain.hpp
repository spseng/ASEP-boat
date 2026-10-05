#include "rclcpp/rclcpp.hpp"
#include "Eigen/Core"
#include "boat_defs/boat_defs.h"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

class SwarmCaptain : public rclcpp::Node {
public:
    SwarmCaptain();

private:
    double k_avoidance_;
    double k_cohesion_;
    double k_descent_;

    double avoidance_radius_;
    double cohesion_saturation_distance_;
    double cohesion_max_;

    double eps_;

    double prev_scalar_;
    double curr_scalar_;

    int scalar_count_;

    std::vector<Eigen::Vector2d> peer_relative_positions_;

    Eigen::Vector2d prev_odom_;
    Eigen::Vector2d curr_odom_;
    Eigen::Vector2d prev_scalar_acquired_odom_;
    Eigen::Vector2d curr_scalar_acquired_odom_;

    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr peer_detection_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr scalar_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr net_pub_;

    // for debugging
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr avoidance_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr cohesion_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr descent_pub_;

    // callback functions
    void scalar_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void peer_detection_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // vector math functions
    Eigen::Vector2d get_avoidance();
    Eigen::Vector2d get_cohesion();
    Eigen::Vector2d get_descent();
};