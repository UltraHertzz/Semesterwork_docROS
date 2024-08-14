#!/bin/bash

# 日志文件路径
LOG_FILE="~/ros2_ws/log/runtime/ps_aux_listener_$(date +"%H:%S").log"

# 无限循环，按照0.1秒的周期执行
while true
do
    # 获取ROS 2节点列表
    for node in $(ros2 node list)
    do
        # 格式化节点名以适应grep命令
        formatted_node=$(echo $node | sed 's/\//\\\//g')
        
        # 使用ps aux获取相关进程信息
        ps aux | grep "$formatted_node" | grep -v grep >> $LOG_FILE
    done
    
    # 每0.1秒执行一次
    sleep 0.1
done
