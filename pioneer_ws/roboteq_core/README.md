# roboteq_core


Python script that interacts with the onboard roboteq controller and pulls live data

## Installation

Clone the repository:
```
git clone git@github.com:irahulone/roboteq_core.git
```
## File Description
[main.py](/roboteq_core/main.py) - Start of live data gathering.<br>
[connect.py](/roboteq_core/connect.py) - Methods that involve connecting and sending read and write requests to the component of interest.<br>
[status.py](/roboteq_core/status.py) - Contains all methods used to get the status of a component such as getting the 12v battery reading or velocity reading from drive units.<br>
[publishers/battery.py](/roboteq_core/publishers/battery.py) - Publisher that broadcasts battery voltage readings to a topic.<br>
[publishers/drive_inverter.py](/roboteq_core/publishers/drive_unit.py) - Publisher that broadcasts velocity of drive units.<br>
[publishers/shell_user_ip.py](/roboteq_core/publishers/shell_user_ip.py) - Publisher that broadcasts the user and IP address of the robot.<br>
[subscribers/move.py](/roboteq_core/subscribers/move.py) - Subscriber that listens to the topic that broadcasts kinematic values and converts them to move commands that are sent to the motors.<br>


## Open Loop Testing
#### Execution end to end example
Window 1 (vehicle side): Start teleop_core
```commandline
ssh edu-robot-1@<ip_address>
cd to workspace directory
source /opt/ros/<distro>/setup.bash
source install/setup.bash
ros2 launch telop_core teleop_node.launch.py
```
Window 2 (vehicle side): Start movebase_kinematics
```commandline
ssh edu-robot-1@<ip_address>
cd to workspace directory
source /opt/ros/<distro>/setup.bash
source install/setup.bash
ros2 run locomotion_core movebase_kinematics
```
Window 3 (local): Start roboteq_core
```commandline
cd to workspace directory
source /opt/ros/<distro>/setup.bash
colcon build
source install/setup.bash
ros2 run roboteq_package roboteq_interface
```
Confirm roboteq_move_subscriber is receiving messages from movebase_kinematics using rqt_graph

Window 4 (local): Display live data
```commandline
ros2 topic echo /*/battery_voltage
data: 13.10000
data: 13.10000
...
```
While using joystick to move wheels
```commandline
ros2 topic echo /*/di_velocity
(while using the joystick)
---
data:
- 3.0
- -3.0
---
---
data:
- -8.0
- 8.0
---
data:
- 7.0
- 7.0
---
---
- -20.0
- -20.0
---
```
