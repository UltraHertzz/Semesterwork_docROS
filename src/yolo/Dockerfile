FROM ros:humble-ros-base-jammy

ENV ROS_WS=/root/ros_ws
#WORKDIR /home/orangepi/ros2_ws
WORKDIR $ROS_WS

RUN mkdir -p $ROS_WS/src

COPY ./ src/yolo

EXPOSE 80

RUN /bin/bash -c "apt-get update && apt-get upgrade -y && \ 
                  apt-get install python3-pip -y && \
                  apt-get install ros-humble-cv-bridge -y"

RUN pip install ultralytics

RUN /bin/bash -c "cd $ROS_WS && colcon build --symlink-install"

CMD [ "/bin/bash" ,"-c", "source ${ROS_WS}/install/setup.bash && \
                          export ROS_DOMAIN_ID=30 && \
                          ros2 run yolo yolo_node" ]
