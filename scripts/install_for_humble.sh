#!/bin/bash
mkdir -p ~/.gazebo
mkdir -p ~/.gazebo/models
cp -r ../../wp_map_tools ~/.gazebo/models/
sudo apt install -y ros-humble-nav2-bringup
sudo apt install -y ros-humble-pluginlib
sudo apt install -y libyaml-cpp-dev
