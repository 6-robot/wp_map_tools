#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "wp_map_tools/msg/waypoint.hpp"


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("demo_navi_waypoint");

    auto waypoint_pub = node->create_publisher<std_msgs::msg::String>("waterplus/navi_waypoint",1);

    rclcpp::sleep_for(std::chrono::milliseconds(1000));

    std_msgs::msg::String msg;
    msg.data = "1";

    if(argc > 1)
        msg.data = argv[1];

    waypoint_pub->publish(msg);
    RCLCPP_INFO(node->get_logger(), "Send waypoint name : %s",msg.data.c_str());

    rclcpp::spin_some(node);
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    rclcpp::shutdown();

    return 0;
}