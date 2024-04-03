# to check the topic content, you should get root privilege

# echo control on orangepi
# sudo bash -c "source /home/orangepi/ros2_ws/install/setup.bash && ros2 launch controller control.launch.py" 

echo control with pc
sudo bash -c "source /home/orangepi/ros2_ws/install/setup.bash && ros2 launch controller remote_control_pi.launch.py"