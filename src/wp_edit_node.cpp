#include <memory>
#include <sstream>
#include <fstream>
#include <string>
#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include "wp_map_tools/msg/waypoint.hpp"
#include "wp_map_tools/srv/save_waypoints.hpp"
#include "wp_map_tools/srv/get_waypoint_by_name.hpp"

static std::shared_ptr<rclcpp::Node> node;
static std::vector <wp_map_tools::msg::Waypoint> arWaypoint;
static std::vector <wp_map_tools::msg::Waypoint> arCharger;
static interactive_markers::InteractiveMarkerServer* pWaypointServer = NULL;

static interactive_markers::MenuHandler* pMenuWaypoint = NULL;

bool bDeleteWaypoint = false;
std::string strDelWaypointName;

std::string Flt2Str(float inVal)
{
    std::ostringstream stringStream;
    stringStream << inVal;
    std::string retStr = stringStream.str();
    return retStr;
}

std::string Int2Str(int inVal)
{
    std::ostringstream stringStream;
    stringStream << inVal;
    std::string retStr = stringStream.str();
    return retStr;
}

bool SaveWaypointsToFile(std::string inFilename)
{
    // 创建空内容
    YAML::Node content;
    content = YAML::Node();

    // 1 航点坐标
    int nNumWP = arWaypoint.size();
    content["Waypoints_Num"] = nNumWP;
    for(int i=0; i<nNumWP; i++)
    {
      YAML::Node node;
      node["Type"] = "Waypoint";
      node["Name"] = arWaypoint[i].name;
      node["Pos_x"] = Flt2Str(arWaypoint[i].pose.position.x);
      node["Pos_y"] = Flt2Str(arWaypoint[i].pose.position.y);
      node["Pos_z"] = Flt2Str(arWaypoint[i].pose.position.z);
      node["Ori_x"] = Flt2Str(arWaypoint[i].pose.orientation.x);
      node["Ori_y"] = Flt2Str(arWaypoint[i].pose.orientation.y);
      node["Ori_z"] = Flt2Str(arWaypoint[i].pose.orientation.z);
      node["Ori_w"] = Flt2Str(arWaypoint[i].pose.orientation.w);
      std::string waypointKey = "Waypoint_" + std::to_string(i+1);
      content[waypointKey] = node;
    }

    // 2 充电桩坐标（未实现）
    // int nNumCharger = arCharger.size();

    // 写入YAML文件
    std::ofstream outfile(inFilename);
    if (!outfile)
    {
        std::cerr <<  "写入文件 "<< inFilename <<"失败！" << std::endl;
        return false;
    }

    outfile << content;
    outfile.close();

    return true;
}

bool saveWaypoints(const std::shared_ptr<wp_map_tools::srv::SaveWaypoints::Request> req,
                   std::shared_ptr<wp_map_tools::srv::SaveWaypoints::Response> res)
{
    res->result = SaveWaypointsToFile(req->filename);
    return true;
}

bool getWaypointByName(const std::shared_ptr<wp_map_tools::srv::GetWaypointByName::Request> req,
                       std::shared_ptr<wp_map_tools::srv::GetWaypointByName::Response> res)
{
    std::string reqName = req->name;
    int nNumWP = arWaypoint.size();
    bool bResultGetWP = false;
    for (int i = 0; i < nNumWP; i++)
    {
        if (arWaypoint[i].name == reqName)
        {
            res->name = arWaypoint[i].name;
            res->pose = arWaypoint[i].pose;
            bResultGetWP = true;
            break;
        }
    }
    if (bResultGetWP == true)
    {
        RCLCPP_INFO(node->get_logger(), "Get_wp_name: name = %s", res->name.c_str());
        return true;
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "Get_wp_name: failed! There is no waypoint name %s", reqName.c_str());
        return false;
    }
}

bool LoadWaypointsFromFile(std::string inFilename)
{
    // 检查文件是否存在
    std::ifstream file(inFilename);
    bool fileExists = file.good();
    file.close();

    YAML::Node yamlNode;

    if (fileExists)
    {
        // 文件存在，加载内容
        yamlNode = YAML::LoadFile(inFilename);

        // 检查是否成功加载了配置
        if (yamlNode.IsNull()) 
        {
            std::cerr << inFilename <<"包含无效文件格式" << std::endl;
            return false;
        }
    }
    else
    {
        // 文件不存在，直接返回
        return false;
    }

    // 提取 waypoints_num
    int numWaypoints = yamlNode["Waypoints_Num"].as<int>();

    arWaypoint.clear();
    wp_map_tools::msg::Waypoint newWayPoint;
    // 提取每个 waypoint 的数据
    for (int i = 0; i < numWaypoints; ++i) 
    {
        std::string waypointKey = "Waypoint_" + std::to_string(i+1);
        if(yamlNode[waypointKey]["Type"].as<std::string>() == "Waypoint")
        {
            newWayPoint.name = yamlNode[waypointKey]["Name"].as<std::string>();
            newWayPoint.pose.position.x = yamlNode[waypointKey]["Pos_x"].as<double>();
            newWayPoint.pose.position.y = yamlNode[waypointKey]["Pos_y"].as<double>();
            newWayPoint.pose.position.z = yamlNode[waypointKey]["Pos_z"].as<double>();
            newWayPoint.pose.orientation.x = yamlNode[waypointKey]["Ori_x"].as<double>();
            newWayPoint.pose.orientation.y = yamlNode[waypointKey]["Ori_y"].as<double>();
            newWayPoint.pose.orientation.z = yamlNode[waypointKey]["Ori_z"].as<double>();
            newWayPoint.pose.orientation.w = yamlNode[waypointKey]["Ori_w"].as<double>();
            arWaypoint.push_back(newWayPoint);

            // 打印每个 waypoint 的数据
            // std::cout << "Waypoint " << i << ":" << std::endl;
            // std::cout << "Name: " << newWayPoint.name << std::endl;
            // std::cout << "Pos_x: " << newWayPoint.pose.position.x << std::endl;
            // std::cout << "Pos_y: " << newWayPoint.pose.position.y << std::endl;
            // std::cout << "Pos_z: " << newWayPoint.pose.position.z << std::endl;
            // std::cout << "Ori_x: " << newWayPoint.pose.orientation.x << std::endl;
            // std::cout << "Ori_y: " << newWayPoint.pose.orientation.y << std::endl;
            // std::cout << "Ori_z: " << newWayPoint.pose.orientation.z << std::endl;
            // std::cout << "Ori_w: " << newWayPoint.pose.orientation.w << std::endl;
            // std::cout << std::endl;
        }
    }

    return true;
}


// 移动航点的回调函数
void processWaypointFeedback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback,
  rclcpp::Logger logger)
{
    // std::ostringstream oss;
    // oss << feedback->marker_name << " is now at " << feedback->pose.position.x << ", " <<
    //   feedback->pose.position.y << ", " << feedback->pose.position.z;
    // RCLCPP_WARN(logger, "%s", oss.str().c_str());

    int nNumWP = arWaypoint.size();
    for(int i=0; i<nNumWP ; i++ )
    {
        if(feedback->marker_name == arWaypoint[i].name)
        {
            arWaypoint[i].pose = feedback->pose;
        }
    }
}


// 向服务器添加新的航点操作标记
void NewWaypointInterMarker(interactive_markers::InteractiveMarkerServer* inServer,std::string inName, geometry_msgs::msg::Pose InPose)
{
    visualization_msgs::msg::InteractiveMarker wp_itr_marker;
    visualization_msgs::msg::InteractiveMarkerControl wp_dis_ctrl;
    visualization_msgs::msg::InteractiveMarkerControl move_control;
    wp_itr_marker.header.stamp=node->get_clock()->now();
    wp_itr_marker.name = inName;
    wp_itr_marker.description = inName;
    wp_itr_marker.pose = InPose;
    wp_itr_marker.header.frame_id = "map";
    wp_itr_marker.header.stamp=node->get_clock()->now();

    // 显示3D模型
    visualization_msgs::msg::Marker wp_dis_marker;
    wp_dis_marker.action = visualization_msgs::msg::Marker::ADD;
    wp_dis_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    wp_dis_marker.mesh_resource = "package://wp_map_tools/meshes/waypoint.dae";
    wp_dis_marker.scale.x = 1;
    wp_dis_marker.scale.y = 1;
    wp_dis_marker.scale.z = 1;
    wp_dis_marker.color.r = 1.0;
    wp_dis_marker.color.g = 0.0;
    wp_dis_marker.color.b = 1.0;
    wp_dis_marker.color.a = 1.0;
    wp_dis_ctrl.markers.push_back( wp_dis_marker );

    // 显示航点名称文字
    visualization_msgs::msg::Marker text_marker;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.scale.z = 0.3;
    text_marker.color.r = 0;
    text_marker.color.g = 0;
    text_marker.color.b = 1;
    text_marker.color.a = 1.0;
    text_marker.text = inName;
    text_marker.pose.position.z = 0.8;
    wp_dis_ctrl.markers.push_back( text_marker );
    
    wp_dis_ctrl.always_visible = true;
    wp_itr_marker.controls.push_back( wp_dis_ctrl );

    // 操作设置
    move_control.name = "move_x";
    move_control.orientation.w = 1.0;
    move_control.orientation.x = 1.0;
    move_control.orientation.y = 0.0;
    move_control.orientation.z = 0.0;
    move_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    wp_itr_marker.controls.push_back(move_control);
    move_control.name = "move_z";
    move_control.orientation.x = 0.0;
    move_control.orientation.z = 1.0;
    wp_itr_marker.controls.push_back(move_control);
    move_control.name = "rotate_z";
    move_control.orientation.x = 0.0;
    move_control.orientation.y = 1.0;
    move_control.orientation.z = 0.0;
    move_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    wp_itr_marker.controls.push_back(move_control);

    // 加入菜单
    visualization_msgs::msg::Marker menu_marker;
    menu_marker.type = visualization_msgs::msg::Marker::CUBE;
    menu_marker.scale.x = 0.5;
    menu_marker.scale.y = 0.5;
    menu_marker.scale.z = 0.5;
    menu_marker.color.r = 0.9;
    menu_marker.color.g = 0.9;
    menu_marker.color.b = 0.9;
    menu_marker.color.a = 0.0;  //全透明
    visualization_msgs::msg::InteractiveMarkerControl menu_control;
    menu_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
    menu_control.always_visible = true;
    menu_control.markers.push_back( menu_marker );
    wp_itr_marker.controls.push_back( menu_control );

    // inServer->insert(wp_itr_marker, &processWaypointFeedback);
    inServer->insert(
    wp_itr_marker, 
    std::bind(&processWaypointFeedback, std::placeholders::_1, node->get_logger()));

    inServer->applyChanges();
}

// 菜单删除航点的回调函数
void DeleteWaypointCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback, const rclcpp::Node::SharedPtr& node)
{
    strDelWaypointName = feedback->marker_name;
    bDeleteWaypoint = true;

    pWaypointServer->erase(strDelWaypointName);
    pWaypointServer->applyChanges();
    for(std::vector<wp_map_tools::msg::Waypoint>::iterator iter=arWaypoint.begin(); iter!=arWaypoint.end(); )
    {
        if( (*iter).name == strDelWaypointName)
            iter = arWaypoint.erase(iter);
        else
            iter ++ ;
    }
    RCLCPP_WARN(node->get_logger(),"Menu - Delete waypoint %s",strDelWaypointName.c_str());
}

// 添加新航点回调函数
void AddWayPointCallback(const wp_map_tools::msg::Waypoint::ConstPtr& wp)
{
    RCLCPP_WARN(node->get_logger(),"Add_waypoint: %s (%.2f %.2f) (%.2f %.2f %.2f %.2f) ",wp->name.c_str(), wp->pose.position.x, wp->pose.position.y, wp->pose.orientation.x, wp->pose.orientation.y, wp->pose.orientation.z, wp->pose.orientation.w);
    wp_map_tools::msg::Waypoint newWayPoint;
    newWayPoint = *wp;
    int nWPNum = arWaypoint.size();
    for(int i= 0;i<nWPNum;i++)
    {
        if(newWayPoint.name == arWaypoint[i].name)
        {
            newWayPoint.name = newWayPoint.name + "_1";
        }
    }
    arWaypoint.push_back(newWayPoint);
    if(pWaypointServer != NULL)
    {
        NewWaypointInterMarker( pWaypointServer, newWayPoint.name, newWayPoint.pose );
        pMenuWaypoint->apply( *pWaypointServer, newWayPoint.name );
        //通知client（RVIZ）更新显示
        pWaypointServer->applyChanges();
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>("wp_edit_node");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    // 创建服务
    interactive_markers::InteractiveMarkerServer server("waypoint_markers", node);
    pWaypointServer = &server;

    ///////////////////////////////////////////////////////////////////////
    // create an interactive marker for our server
    interactive_markers::MenuHandler menu_waypoint;
    menu_waypoint.insert("Delete", [node](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback) {
    DeleteWaypointCallback(feedback, node);
    });
    pMenuWaypoint = &menu_waypoint;

    //服务和话题初始化
    auto add_waypoint_sub = node->create_subscription<wp_map_tools::msg::Waypoint>("waterplus/add_waypoint",10,AddWayPointCallback);
    auto service = node->create_service<wp_map_tools::srv::SaveWaypoints>("waterplus/save_waypoints", saveWaypoints);
    auto srvGetWPName = node->create_service<wp_map_tools::srv::GetWaypointByName>("waterplus/get_waypoint_name",std::bind(&getWaypointByName, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(node->get_logger(), "wp_edit_node 初始化完毕");

    // 读取YAML文件
    std::string strLoadFile;
    char const* home = getenv("HOME");
    strLoadFile = home;
    strLoadFile += "/waypoints.yaml";

    RCLCPP_INFO(node->get_logger(), "文件strLoadFile: %s", strLoadFile.c_str());

    node->declare_parameter("load", strLoadFile);

    strLoadFile = node->get_parameter("load").as_string();

    RCLCPP_INFO(node->get_logger(), "文件strLoadFile: %s", strLoadFile.c_str());
    if (strLoadFile.length() > 0)
    {
        RCLCPP_INFO(node->get_logger(), "Load waypoints from file: %s", strLoadFile.c_str());
        LoadWaypointsFromFile(strLoadFile);
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "strLoadFile is empty. Failed to load waypoints!");
    }

    int nWPNum = arWaypoint.size();
    for(int i= 0;i<nWPNum;i++)
    {
        
        NewWaypointInterMarker( pWaypointServer, arWaypoint[i].name, arWaypoint[i].pose );
        pMenuWaypoint->apply( *pWaypointServer, arWaypoint[i].name );
    }
    //通知client（RVIZ）更新显示
    pWaypointServer->applyChanges();

    // start processing callbacks
    executor.spin();

    rclcpp::shutdown();

    return 0;
}