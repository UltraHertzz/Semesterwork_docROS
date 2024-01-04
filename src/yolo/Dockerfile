FROM ros:humble-ros-base-jammy

ENV ROS_WS = "/ros_ws"

RUN mkdir -p ${ROS_WS}/src

#WORKDIR /home/orangepi/ros2_ws
WORKDIR ${ROS_WS}

COPY ./yolo src/yolo

EXPOSE 80

RUN apt-get update

RUN apt-get upgrade -y

RUN apt-get install python3-pip -y

RUN apt-get install ros-humble-cv-bridge -y

RUN pip install ultralytics

RUN colcon build

RUN source install/setup.bash

RUN export ROS_DOMAIN_ID=30

CMD [ "ros2","run","yolo","yolo" ]
