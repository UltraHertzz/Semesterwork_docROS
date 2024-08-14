#! /bin/bash

#! /bin/bash
cd $HOME/ros2_ws
source $HOME/ros2_ws/install/setup.bash
DATE=$(date +"%Y-%m-%d-%H-%M-%S")
LOG_FILE_PATH="$HOME/ros2_ws/log/runtime/bare_matel/bare_matel_sensor_${DATE}.log"
if [ "$ROS_DOMAIN_ID" != "30" ]; then
    export ROS_DOMAIN_ID=30
fi
echo "exec start time: $(date +"%s.%N")" >> $LOG_FILE_PATH
ros2 launch bno055 data_collection.launch.py >> ~/ros2_ws/log/runtime/bare_matel/bare_matel_${DATE}.log 2>&1 &
ros2 echo 