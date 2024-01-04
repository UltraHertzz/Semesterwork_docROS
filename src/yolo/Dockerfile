FROM ros:humble-ros-core-jammy

#ENV ROS_WS = /ros_ws

#RUN mkdir -p ${ROS_WS}/src

WORKDIR /home/orangepi/ros2_ws
#WORKDIR ${ROS_WS}

#COPY ./yolo src/yolo

RUN apt-get update && apt-get upgrade && apt-get install python3-pip

RUN apt-get install ros-humble-cv-bridge && pip install ultralytics

EXPOSE 80

RUN /bin/bash -c 'source install/setup.bash'

RUN /bin/bash -c 'export ROS_DOMAIN_ID=30'

CMD [ "ros2","run","yolo","yolo" ]
