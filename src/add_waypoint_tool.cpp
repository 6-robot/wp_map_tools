/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/* @author Zhang Wanjie                                             */

#include "wp_map_tools/add_waypoint_tool.hpp"
// #include <wp_map_tools/srv/GetWaypointByName.h>
#include "rviz_common/load_resource.hpp"


static int nWaypointCount = 0;

namespace wp_map_tools
{
    AddWaypointTool::AddWaypointTool(): rviz_default_plugins::tools::PoseTool()
    {
        shortcut_key_ = 'a';
        topic_property_ = new rviz_common::properties::StringProperty( "Topic", "/waterplus/add_waypoint","The topic on which to add new waypoints.",
        getPropertyContainer(), SLOT( updateTopic() ), this );
    }

    AddWaypointTool::~AddWaypointTool()
    {
    }

    void AddWaypointTool::onInitialize()
    {
        rviz_default_plugins::tools::PoseTool::onInitialize();
        setName( "Add Waypoint" );
        setIcon(rviz_common::loadPixmap("package://wp_map_tools/icons/classes/AddWaypointTool.png"));
        updateTopic();
    }

    void AddWaypointTool::updateTopic()
    {
        raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
        pub_ = raw_node->create_publisher<wp_map_tools::msg::Waypoint>(topic_property_->getStdString(), 1);
        // pub_ = nh_->create_publisher<wp_map_tools::msg::Waypoint>( topic_property_->getStdString(), 1);
        // cliGetWPName = nh_.serviceClient<wp_map_tools::srv::GetWaypointByName>("/waterplus/get_waypoint_name");
    }

    void AddWaypointTool::onPoseSet(double x, double y, double theta)
    {
       
        // RCLCPP_WARN(raw_node->get_logger(),"onPoseSet 响应 x = %.2f y=%.2f theta= %.2f",x,y,theta);
        // 将欧拉角转换成四元数
        std::string fixed_frame = context_->getFixedFrame().toStdString();
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, theta);
        geometry_msgs::msg::PoseStamped new_pos;
        new_pos.header.stamp = rclcpp::Clock().now();  // 设置时间戳
        new_pos.header.frame_id = fixed_frame;     // 设置框架ID
        geometry_msgs::msg::Quaternion msgQuat;
        tf2::convert(quat, msgQuat);
        new_pos.pose.orientation = msgQuat;
        new_pos.pose.position.x = x;
        new_pos.pose.position.y = y;

        nWaypointCount ++;
        std::ostringstream stringStream;
        stringStream << nWaypointCount;
        std::string strNewName = stringStream.str();

        // 查询是否存在同名节点。若存在同名，则节点号加1
        // wp_map_tools::srv::GetWaypointByName srvN;
        // srvN.request.name = strNewName;
        // while (cliGetWPName.call(srvN))
        // {
        //     nWaypointCount ++;
        //     std::ostringstream stringStream;
        //     stringStream << nWaypointCount;
        //     strNewName = stringStream.str();
        //     srvN.request.name = strNewName;
        // }
        // RCLCPP_WARN(raw_node->get_logger(),"New waypoint name = %s",strNewName.c_str());

        wp_map_tools::msg::Waypoint new_waypoint;
        new_waypoint.name = strNewName;
        new_waypoint.pose = new_pos.pose;
        pub_->publish(new_waypoint);
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(wp_map_tools::AddWaypointTool,rviz_common::Tool)
