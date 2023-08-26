#include <chrono>
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "wp_map_tools/srv/save_waypoints.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("wp_saver");

    auto client = node->create_client<wp_map_tools::srv::SaveWaypoints>("waterplus/save_waypoints");

    std::string strSaveFile;
    const char* home = getenv("HOME");
    strSaveFile = home;
    strSaveFile += "/waypoints.yaml";

    for (int i = 1; i < argc; i++)
    {
        if (!strcmp(argv[i], "-f"))
        {
            if (++i < argc)
            {
                strSaveFile = argv[i];
            }
        }
    }

    auto request = std::make_shared<wp_map_tools::srv::SaveWaypoints::Request>();
    request->filename = strSaveFile;

    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting...");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "Service not available, waiting...");
    }

    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "Save waypoints to the file: %s", request->filename.c_str());
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service save_waypoints");
    }

    rclcpp::shutdown();
    return 0;
}