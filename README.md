# Turtlebot3 Side-by-Side Navigation System

### Collaborators: Jun Hu, Adrian Hilton, Shaden Shaar, Rajiv Thummala

## Introduction
This repository hosts the development of an innovative navigation system for Turtlebot3 robots, designed to enable them to navigate side-by-side in dynamic environments without a predetermined goal. Inspired by the research paper "Destination Unknown: Walking Side-by-Side without Knowing the Goal", this project applies theoretical concepts to practical autonomous navigation.

Our main goal is to develop and implement algorithms for two Turtlebot3 Waffle Pi robots, enabling them to coordinate movements, avoid obstacles in real-time, and dynamically adjust paths based on environmental feedback and sensor inputs.

### Technologies
- Robot Operating System (ROS) Noetic
- Gazebo 11 for simulation
- Turtlebot3 Waffle Pi robots

## Installation
### Prerequisites
- Ubuntu 20.04
- ROS Noetic
- Gazebo 11
- Turtlebot3 packages

### Setting Up
Follow these steps to set up your Turtlebot3 simulation environment:
1. **Configure Turtlebot3 Model**: Add `export TURTLEBOT3_MODEL=waffle_pi` to your `.bashrc` file and source it.

2. **Create and Initialize the ROS Workspace:**
   ```bash
   mkdir -p ~/turtlebot3_ws/src
   cd ~/turtlebot3_ws/
   catkin_make

### Running Simulations
1. **Source your ROS Workspace:**
   Ensure that your terminal session is using your ROS workspace's configuration:
   ```bash
   source ~/turtlebot3_ws/devel/setup.bash

2. **Control the Robot**: Use ROS topics or Rviz to control the simulated Turtlebot3. You can publish velocity commands to `/cmd_vel` to move the robot.

### Real-world Deployment
1. **Robot Setup**: Ensure your Turtlebot3 Waffle Pi robots are charged and connected to your network.
2. **Connect to the Robot**: SSH into each Turtlebot3 or connect via a terminal if using a direct connection.
3. **Launch ROS Nodes**: On each Turtlebot3, launch the necessary ROS nodes for navigation, sensors, etc.
4. **Control and Monitor**: Use your ROS setup to control the robots and monitor their sensors. Ensure you have a safe, open space for testing.

## Features
- Side-by-side navigation algorithms
- Real-time obstacle avoidance
- Dynamic path planning and sub-goal estimation
- Integration with ROS for effective robot control and sensor data processing

## License
This project is licensed under the [MIT License](LICENSE) - see the LICENSE file for details.
