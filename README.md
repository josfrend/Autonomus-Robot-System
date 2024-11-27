# Autonomous Robot System
This project presents the development of an implementation in the Puzzlebot, an educational mobile robot designed to close the gap between education and industrial robotics. The Puzzlebot employs a differential drive system for precise maneuverability, making it suitable for a range of applications. Equipped with a NVIDIA Jetson Nano, a LiDAR sensor, and additional hardware, the robot integrates advanced algorithms and sensor fusion techniques to achieve effective navigation and obstacle avoidance. The navigation system utilizes the Bug2 algorithm for path planning, while odometry and an Extended Kalman Filter enhance localization accuracy. The early Puzzlebot's capabilities were validated through simulation using Gazebo software, ensuring robust performance in dynamic settings. Thus, this project aims to provide a versatile platform for both educational and experimental purposes in robotics.

## Installation


The implementation runs on ROS2, Humble along its necessary libraries, and uses Gazebo Garden for the simulation environment. 

Clone the repository: 

~~~ shell 
git clone https://github.com/josfrend/Autonomus-Robot-System.git
~~~

Go in the repository folder and run the `setup.sh` file to setup a new workspace and install the necessary resources (if interested in the particular installation, consult the [official ROS2 documentation](https://docs.ros.org/en/humble/Installation.html) and  the [official Gazebo documentation](https://gazebosim.org/docs/garden/ros_installation/))

~~~ shell 
cd Autonomus-Robot-System
chmod +x ./setup.sh
./setup.sh
~~~

## Usage

### Simulation 
To use the simulated environment use the its respective launch file:
~~~ shell
ros2 launch puzzlebot_gazebo ros_ign_bridge.launch.py
~~~

Then launch the algorithms to operate the robot:
~~~ shell 
ros2 launch puzzlebot_challenge puzzlebot_operation.launch.py
~~~
