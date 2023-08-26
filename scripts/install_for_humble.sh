#!/bin/bash
mkdir -p ~/.gazebo
mkdir -p ~/.gazebo/models
cp -r ../../wp_map_tools ~/.gazebo/models/
sudo apt install libtinyxml-dev
sudo apt install -y ros-humble-nav2-bringup
