# ros2

## Install ros2
Run file:
```
sudo chmod +x install_ros2.sh
sudo ./install_ros2.sh
```

## Try some examples
In one terminal, source the setup file and then run a C++ talker:
```
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

In another terminal source the setup file and then run a Python listener:
```
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```
## Uninstall
```
sudo apt remove ~nros-<rosdistro>-* && sudo apt autoremove
sudo rm /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt autoremove
# Consider upgrading for packages previously shadowed.
sudo apt upgrade
```

## Create workspace
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

## Build workspace
```
colcon build
```
or
```
colcon build --symlink-install
```

## Ros package
Format:
```
my_package/
     CMakeLists.txt
     include/my_package/
     package.xml
     src/
```

Create pkg and node:
```
ros2 pkg create <package_name> --build-type <build-type> --dependencies <dependencies> --node-name <node-name>
Ex: ros2 pkg create pkg_helloworld_py --build-type ament_python --dependencies rclpy --node-name helloworld
```

Create package python:
```
ros2 pkg create --build-type ament_python --license Apache-2.0 <package_name>
```

Create package cpp:
```
ros2 pkg create --build-type ament_cmake --license Apache-2.0 <package_name>
```

Build only package:
```
colcon build --packages-select my_package
```

## Ros topic
```
ros2 topic echo <topic_name> : To see the data being published on a topic
Ex: ros2 topic echo /turtle1/cmd_vel

ros2 topic pub <topic_name> <msg_type> '<args>'
Ex: ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

ros2 topic hz /turtle1/pose
ros2 topic bw /turtle1/pose

ros2 topic find <topic_type>
Ex: ros2 topic find geometry_msgs/msg/Twist
```

## Ros service
```
ros2 service call <service_name> <service_type> <arguments>
Ex: ros2 service call /clear std_srvs/srv/Empty

ros2 service echo <service_name | service_type> <arguments>
```

## Ros bag
```
ros2 bag record <topic_name>: Record a single topic
ros2 bag record -o subset /turtle1/cmd_vel /turtle1/pose: Record multiple topics

ros2 bag info <bag_file_name>

ros2 bag play <bag_file_name>
ros2 bag play --publish-service-requests <bag_file_name>
```

## Ros param
```
ros2 param list

ros2 param get <node_name> <parameter_name>
Ex: ros2 param get /turtlesim background_g

ros2 param set <node_name> <parameter_name> <value>
Ex: ros2 param set /turtlesim background_r 150

ros2 param dump <node_name> :view all of a node’s current parameter values
Ex: ros2 param dump /turtlesim > turtlesim.yaml

ros2 param load <node_name> <parameter_file> :load parameters from a file to a currently running node
Ex: ros2 param load /turtlesim turtlesim.yaml

ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
Ex: ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim.yaml
```

## Document
```
http://www.yahboom.net/study/ROSMASTER-X3
https://github.com/Robotisim/robotics_software_engineer
https://github.com/Robotisim/mobile_robotics_ROS2
https://github.com/ros-mobile-robots/mobile_robot_description/tree/ros2-foxy
```

