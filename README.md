# Graduation Project: Autonomous Mobile Robot (AMR)

🎉 I am thrilled to announce the successful completion and submission of my graduation project: an Autonomous Mobile Robot (AMR) designed for warehouse navigation using ROS!

This innovative project leverages cutting-edge autonomous driving technology to enhance warehouse operations. The AMR excels in navigating complex environments, effectively avoiding obstacles, and optimizing routes to improve efficiency. It utilizes advanced SLAM (Simultaneous Localization and Mapping) techniques and a robust navigation stack to ensure precise and reliable performance.

## Features

- **SLAM**: Utilizes gmapping and Hector SLAM for real-time mapping and localization.
- **Navigation Stack**: Employs the ROS navigation stack with move_base and AMCL for path planning and obstacle avoidance.
- **Motor Control**: Implements a PID controller for precise motor control.
- **Power Management**: Includes LiPo batteries with suitable chargers and a custom-built BMS.
- **Web Interface**: Provides a React-based web interface for remote control and monitoring using rosbridge.
- **Arduino Integration**: Custom Arduino nodes to interface hardware components with the ROS ecosystem.

## Repository Structure

- **my_robot_app/**: Contains the React application for the web interface.
- **my_robot_ws/**: Includes the ROS workspace with packages for SLAM, navigation, motor control, and Arduino integration.
- **arduino_nodes/**: Arduino sketches and libraries for hardware interfacing.
- **README.md**: Documentation and overview of the project.

## Getting Started

### Prerequisites

- **Hardware**: Raspberry Pi 4, Arduino boards, motors, LiPo batteries, sensors (e.g., LIDAR), etc.
- **Software**: ROS Noetic, Ubuntu 20.04, Python 3.x, Node.js, npm, Arduino IDE.

### Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/mohammedhassan9748/GraduationProject.git
   cd GraduationProject
   ```

2. **Setup ROS Workspace:**
   ```bash
   cd my_robot_ws
   catkin_make
   source devel/setup.bash
   ```

3. **Install Dependencies:**
   ```bash
   cd src
   rosdep install --from-paths . --ignore-src -r -y
   ```

4. **Setup Web Interface:**
   ```bash
   cd ../my_robot_app
   npm install
   npm start
   ```

5. **Upload Arduino Nodes:**
   - Open the Arduino IDE.
   - Open and upload the sketches from the `arduino_nodes/` directory to the respective Arduino boards.

## Usage

### Running the Robot

1. **Launch ROS Core:**
   ```bash
   roscore
   ```

2. **Launch Arduino Nodes:**
   ```bash
   rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0
   ```

3. **Launch SLAM:**
   ```bash
   roslaunch mybot_nav SLAM_REAL.launch
   ```

4. **Launch Navigation:**
   ```bash
   roslaunch mybot_pp test_amcl.launch
   ```

5. **Control the Robot:**
   Open the web interface at `https://graduation-project-brown.vercel.app` and use the controls to navigate the robot.

## Documentation

### ROS Packages

- **my_robot_slam**: Configuration and launch files for Hector SLAM.
- **my_robot_navigation**: Configuration and launch files for the navigation stack.
- **my_robot_control**: PID controller for motor control.
- **my_robot_description**: URDF and other files describing the robot model.
- **arduino_nodes**: Arduino sketches for interfacing sensors and actuators with ROS.

### Web Interface

The web interface allows operators to:
- Monitor battery status and power state.
- Control the robot using a virtual joystick.
- Set navigation goals and view real-time navigation status.

## Demonstration

You can check the robot’s mapping and navigation process through the video found in this [link](https://lnkd.in/dUFudZwP).

## Contributors

This project would not have been possible without the incredible support and hard work of my teammates:
- **Youssef Aly**
- **Sondos Mohamed**
- **Rawan Ahmed**

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
