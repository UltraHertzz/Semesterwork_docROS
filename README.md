# ReadMe file to Semester work docker + ROS

**Important:** 
1. the system is running in ROS_DOMAIN_ID=30, you need to switch to this domain, otherwise you cannot find message at all.
2. It is essential that the control system should run with sudo, because the orangepi need root privilege to access to gpio pin. If you want to run single node with ros2 run, follow the instruction in "Launch Single Node with sudo"

## Set ROS_DOMAIN_ID
```bash
export ROS_DOMAIN_ID=30 # Set ROS domain ID, no space is critical
echo $ROS_DOMAIN_ID # Check if it works, the terminal should print 30

```

## Launch Control System (Device: Game Pad)
```bash
./sudo_run_controller.sh
```

## Launch data collection System (IMU,Lidar)
```bash
ros2 launch bno055 data_collect_launch.py
```

## Launch Single Node with sudo
```bash
# in workspace
sudo su # switch to superuser
source install/setup.bash
export ROS_DOMAIN_ID=30 # important! because all node are running in this domain
# you should also do this if you want to check topic information, because the message from root is not accesable of non-root user, pretty anoying  :(
```