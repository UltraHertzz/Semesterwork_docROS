#!/bin/bash

# config for log file
TOPIC_NAME="/exec_time/controller_to_driver"
EXP_TIME=20



LOG_FILE="$HOME/ros2_ws/log/runtime/bare_matel/topic/${TOPIC_NAME//\//_}.log"
mkdir -p "$(dirname "$LOG_FILE")"
cd $HOME/ros2_ws
if [ -f"$LOG_FILE" ];then
    rm "$LOG_FILE"
    echo "log file deleted"
fi
source install/setup.bash
echo "exec start time: $(date +"%s.%N")"
./script/sudo_run_controller.sh &

while true; do

    output=$(timeout 5 ros2 topic echo $TOPIC_NAME 2>&1 )
    
    if echo "$output" | grep -q "does not appear to be published yet\|Could not determine the type for the passed topic"; then
        echo "Warning detected, retrying..."
        ((attempt++))
    else
        echo "received data at time: $(date +"%s.%N")"
        break
    fi
    
    sleep 0.0001
done

sleep $EXP_TIME
echo $EXP_TIME
pkill -f ros2

echo "kill all ros2 process."
echo "exec end time: $(date +"%s.%N")"