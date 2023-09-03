#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_ros/transform_listener.h>
#include "wp_map_tools/msg/waypoint.hpp"
#include <wp_map_tools/srv/get_waypoint_by_name.hpp>
#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

static bool bNewCmd = false;
std::shared_ptr<rclcpp::Node> node;
rclcpp::Client<wp_map_tools::srv::GetWaypointByName>::SharedPtr cliGetWPName;
static std::string waypoint_name;
static geometry_msgs::msg::Pose wp_pose;
static rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub;
static std_msgs::msg::String result_msg;
rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;

void NaviWaypointCB(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(node->get_logger(), "[NaviWaypointCB] 目标航点= %s",msg->data.c_str());
    waypoint_name = msg->data;
    
    bNewCmd = true;
}

void resultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(node->get_logger(), "Success!!!");
        result_msg.data = "navi done";
        break;
        case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_INFO(node->get_logger(), "Goal was aborted");
        result_msg.data = "navi aborted";
        break;
        case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(node->get_logger(), "Goal was canceled");
        result_msg.data = "navi canceled";
        break;
        default:
        RCLCPP_INFO(node->get_logger(), "Unknown result code");
        result_msg.data = "navi Unknown result";
        break;
    }
    result_pub->publish(result_msg);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("wp_navi_server");
    RCLCPP_INFO(node->get_logger(), "wp_navi_server 节点启动！");

    auto navi_name_sub = node->create_subscription<std_msgs::msg::String>("waterplus/navi_waypoint",10,NaviWaypointCB);

    result_pub = node->create_publisher<std_msgs::msg::String>("waterplus/navi_result", 10);
    cliGetWPName = node->create_client<wp_map_tools::srv::GetWaypointByName>("waterplus/get_waypoint_name");

    navigation_client_ = rclcpp_action::create_client<NavigateToPose>(node, "navigate_to_pose");

    // Wait for the navigation action server to become available
    while (!navigation_client_->wait_for_action_server(std::chrono::seconds(2))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the action server.");
        return 0;
      }
      RCLCPP_INFO(node->get_logger(), "Waiting for the action server to be available...");
    }

    rclcpp::Rate r(30);

    while (rclcpp::ok())
    {
        if (bNewCmd)
        {
            // 查询航点对应的坐标
            auto srvN = std::make_shared<wp_map_tools::srv::GetWaypointByName::Request>();
            srvN->name = waypoint_name;
            RCLCPP_INFO(node->get_logger(), "wp_navi_server 发送航点坐标请求");
            auto result = cliGetWPName->async_send_request(srvN);
            // Wait for the result.
            if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto response = result.get();
                wp_pose = response->pose;
                std::string name = response->name;
                float x = response->pose.position.x;
                float y = response->pose.position.y;
                RCLCPP_INFO(node->get_logger(), "Get_wp_name: name = %s (%.2f,%.2f)", name.c_str(), x, y);
            }
            else
            {
                RCLCPP_ERROR(node->get_logger(), "Failed to call service get_waypoint_name");
                bNewCmd = false;
                continue;
            }

            while (!cliGetWPName->wait_for_service(1s)) 
            {
                if (!rclcpp::ok()) 
                {
                    RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    break;
                }
                RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
            }


            RCLCPP_ERROR(node->get_logger(), "开始导航");
            // wait for the action server to come up
            while (!navigation_client_->wait_for_action_server(std::chrono::seconds(5)))
            {
                if (!rclcpp::ok())
                    break;
                RCLCPP_INFO(node->get_logger(), "Waiting for the move_base action server to come up");
            }
           
            // Create the navigation goal request
            auto goal_msg = NavigateToPose::Goal();

            goal_msg.pose.header.stamp = node->get_clock()->now();
            goal_msg.pose.header.frame_id = "map";
            goal_msg.pose.pose = wp_pose;
            
            // Send the navigation goal request
            auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
            send_goal_options.result_callback = std::bind(&resultCallback, std::placeholders::_1);
            auto goal_handle_future = navigation_client_->async_send_goal(goal_msg, send_goal_options);

            // Wait for the goal to complete
            // if (rclcpp::spin_until_future_complete(node->get_node_base_interface(), goal_handle_future) == rclcpp::FutureReturnCode::SUCCESS)
            // {
            //     RCLCPP_INFO(node->get_logger(), "Arrived at WayPoint !");
            //     result_msg.data = "navi done";
            // }
            // else
            // {
            //     RCLCPP_WARN(node->get_logger(), "Failed to get to WayPoint ...");
            //     result_msg.data = "navi failure";
            // }

            // result_pub->publish(result_msg);
            bNewCmd = false;
        }

        rclcpp::spin_some(node);
        r.sleep();
    }

    rclcpp::shutdown();

    return 0;
}