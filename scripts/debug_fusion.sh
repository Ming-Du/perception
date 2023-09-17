#!/bin/bash
BASE_DIR="$(rospack find config)/vehicle/perception"
PERCEPTION_LIDAR="roslaunch ${BASE_DIR}/lidar/perception_lidar.launch"
PERCEPTION_FUSION="roslaunch ${BASE_DIR}/fusion/perception_fusion.launch"
LOCALIZATION="rosrun localization localization_node"
RVIZ="rviz -d $(rospack find perception_fusion)/../scripts/multi_sensor_fusion.rviz"

CORE_ID=$(ps -ef | grep roscore | grep -v grep | awk '{print $2}')
if [ ! ${CORE_ID} ]; then
  echo "starting roscore..."
  roscore &
  sleep 2
else
  echo "found roscore, is relative nodes on?"
  echo "..."
  NODE_ID=$(rosnode list | grep -E "perception")
  if [ ! -n "${NODE_ID}" ]; then
    echo "relative nodes not found"
  else
    echo "relative nodes found, killing."
    rosnode kill ${NODE_ID}
    echo "nodes restart."
    sleep 2
  fi
  SECOND_ID=$(rosnode list | grep -E "localization|rviz")
  if [ ! -n "${SECOND_ID}" ]; then
    echo "starting localization & rviz"
    gnome-terminal --tab -- ${RVIZ}
    gnome-terminal --tab -- ${LOCALIZATION}
  fi
fi

gnome-terminal --tab -- ${PERCEPTION_LIDAR} 
gnome-terminal --tab -- ${PERCEPTION_FUSION}
