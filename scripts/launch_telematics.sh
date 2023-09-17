#!/bin/bash
CORE="roscore"
DRIVERS="roslaunch launch drivers.launch"
PERCEPTION="roslaunch launch perception.launch"
LOCALIZATION="rosrun localization localization_node"
TELEMATICS="roslaunch telematics telematics.launch"

CORE_ID=$(ps -ef | grep roscore | grep -v grep | awk '{print $2}')
if [ ! ${CORE_ID} ]; then
  echo "starting roscore..."
  gnome-terminal --tab -- ${CORE}
  sleep 4
else
  NODE_ID=$(rosnode list | grep -E "sensor|perception|localization|telematics")
  if [ ! -n "${NODE_ID}" ]; then
    echo "relative nodes not found, starting."
  else
    echo "relative nodes found, killing."
    rosnode kill ${NODE_ID}; sleep 2
  fi
fi

gnome-terminal --tab -- ${DRIVERS}
gnome-terminal --tab -- ${PERCEPTION}
gnome-terminal --tab -- ${LOCALIZATION}
gnome-terminal --tab -- ${TELEMATICS}
