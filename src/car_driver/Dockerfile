FROM ros:humble-ros-base-jammy

ENV ROS_WS=/root/ros_ws
#WORKDIR /home/orangepi/ros2_ws
WORKDIR $ROS_WS

RUN mkdir -p $ROS_WS/src

COPY ./ src/car_driver

EXPOSE 79

RUN /bin/bash -c "apt-get update && \
                  apt-get install python3-pip -y && \
                  cd / && \
                  git clone https://github.com/UltraHertzz/OPI_GPIO_for_OrangePi5Plus.git && \
                  pip install OPi.GPIO && \
                  cp -r /OPI_GPIO_for_OrangePi5Plus/OPi/ /usr/local/lib/python3.10/dist-packages/"

RUN /bin/bash -c "cd $ROS_WS/ && colcon build"

CMD [ "/bin/bash" ,"-c", "source ${ROS_WS}/install/setup.bash && \
                          export ROS_DOMAIN_ID=30 && \
                          ros2 run car_driver drive_node" ]
