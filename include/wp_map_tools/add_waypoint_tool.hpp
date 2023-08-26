#ifndef ADD_WAYPOINT_TOOL_H
#define ADD_WAYPOINT_TOOL_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
    # include <QObject>
    # include <rclcpp/rclcpp.hpp>
    # include <rviz_common/tool.hpp>
    #include "rviz_default_plugins/tools/pose/pose_tool.hpp"
    #include <rviz_default_plugins/tools/pose/pose_tool.hpp>
    #include "rviz_default_plugins/visibility_control.hpp"
    #include <rviz_common/display_context.hpp>
    #include <rviz_common/properties/string_property.hpp>
#endif

#include <wp_map_tools/msg/waypoint.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>


namespace rviz_common
{

    class DisplayContext;

    namespace properties
    {
        class StringProperty;
    }  // namespace properties
}  // namespace rviz_common

namespace wp_map_tools
{
    class Arrow;
    class DisplayContext;
    class StringProperty;

    class RVIZ_DEFAULT_PLUGINS_PUBLIC AddWaypointTool: public rviz_default_plugins::tools::PoseTool
    {
        Q_OBJECT
        public:
        AddWaypointTool();
        ~AddWaypointTool() override;

        void onInitialize() override;

        protected:
        void onPoseSet(double x, double y, double theta) override;

        private Q_SLOTS:
        void updateTopic();

        private:
        rclcpp::Node::SharedPtr raw_node;
        rclcpp::Publisher<wp_map_tools::msg::Waypoint>::SharedPtr pub_;
        // ros::ServiceClient cliGetWPName;
        rviz_common::properties::StringProperty* topic_property_;
    };

}

#endif // ADD_WAYPOINT_TOOL_H