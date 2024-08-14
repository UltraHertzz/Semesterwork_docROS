# to check the topic content, you should get root privilege

# echo control on orangepi
# sudo bash -c "source /home/orangepi/ros2_ws/install/setup.bash && ros2 launch controller control.launch.py" 

#!/bin/bash
DATE=$(date +"%Y-%m-%d-%H-%M-%S")
LOG_FILE_PATH="$HOME/ros2_ws/log/runtime/bare_matel/bare_matel_control_${DATE}.log"
mkdir -p $HOME/ros2_ws/log/runtime/bare_matel
echo control with pc
echo "exec start time: $(date +"%s.%N")" >> $LOG_FILE_PATH
cd $HOME/ros2_ws
sudo bash -c "source /home/orangepi/ros2_ws/install/setup.bash && ros2 launch controller control_mecanum.launch.py >> $LOG_FILE_PATH"