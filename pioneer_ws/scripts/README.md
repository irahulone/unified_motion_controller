Booting with ROS2

1. set ROBOT_ID in start_ros2.sh

Environment="ROBOT_ID=p#"  
User=pioneer-#  
WorkingDirectory=/home/pioneer-#/ros2_ws  
ExecStart=/bin/bash /home/pioneer-#/ros2_ws/pioneer_ws/scripts/start_ros2.sh

2. set up service
```
$ sudo mv ros2_start.service /etc/systemd/system/ros2_start.service
$ sudo systemctl daemon-reload
$ sudo systemctl enable ros2_start.service
$ sudo systemctl start ros2_start.service
$ sudo systemctl status ros2_start.service
```
