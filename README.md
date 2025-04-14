# Differential Drive - ROS2

![](demonstration.gif)

See installations instructions below!

## Overview
This repo contains an experimental ROS2-based project that integrates Raspberry Pi (running ROS2) with an Arduino microcontroller to control a two-wheeled differential drive robot. My personal goal for this project was to bridge the gap between advanced control algorithms and actual hardware. I definitely underestimated the difficulty of this as simulations are a totally different thing compared to real-world hardware!


### Structure

The repository is organized as a ROS2 workspace (`ros_ws`) containing several packages, and an `arduino` folder with the microcontroller firmware. This structure separates high-level control logic from low-level hardware interfacing.

#### Key components
- Arduino Firmware (`arduino/MD25_Motor_Bridge_ROS_Communication.ino`): Runs on the Arduino microcontroller. It interfaces with the MD25 motor driver (for motor control and encoder readings) and implements a PID control loop for motor speeds. It receives high-level velocity commands over serial which are then transformed into wheel speeds by calculating the inverse kinematics. It also sends back encoder counts and wheel speeds over serial at 50 Hz.

- Arduino Bridge Node (`ros_ws/src/my_cpp_pkg/src/arduino_bridge_node`): A C++ ROS2 node that connects the Raspberry Pi to the Arduino via a UART/USB serial link. It opens the serial port at the configured baud (115200 bps, as set in the Arduino code) and handles two (currently used) ROS topics:

    - Subscribes to arduino/cmd_vel (my_robot_interfaces/CmdDriveVel) – upon receiving a message, it formats and forwards it to the Arduino as a serial string.

    - Publishes arduino/motor_odom_info (my_robot_interfaces/MotorOdomInfo) – as it reads lines from the Arduino, it populates this message with the latest encoder and RPM data.
    
    This node is crucial for hardware interfacing.

- MPC Controller Node (`ros_ws/src/my_py_pkg/mpc_diffdrive_control.py`):    A Python ROS2 node that implements the high-level control of the robot. It uses acados to create an optimal control solver for the robot’s motion. What it does:

  - Subscribes to wheel odometry info and computes the robot’s pose (integrating encoder data to estimate how far it has moved and turned).

  - Subscribes to a target command (which can come from a topic or the MoveTo action) specifying where the robot should go or what velocity it should achieve.

  - Runs the MPC loop at 12 Hz. The MPC uses the robot’s current state as the starting point and predicts the best control inputs to reach the target state over the horizon. It accounts for kinematic constraints (differential drive limits) and can include obstacle avoidance in the optimization cost.

  - Publishes the resulting wheel commands as CmdDriveVel for the serial node to actuate.
    


### Hardware

- Motors, Motor Driver and Wheels:  [MD25 - Dual 12Volt 2.8Amp H Bridge Motor Drive for EMG30](https://static.rapidonline.com/pdf/70-6403_v1.pdf)

- Raspberry Pi 5
- Arduino Uno
- [YDLIDAR T-mini Plus](https://www.ydlidar.com/Public/upload/files/2024-05-24/YDLIDAR%20T-mini%20Plus%20Data%20Sheet_V1.1%20(240131).pdf)

### Installation Instructions

Clone the repository:

```bash
git clone https://github.com/DPMA00/DiffDrive-ROS.git
```

Make sure you have ROS2 installed on your system. See instructions [here](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).


There are some external packages and libraries which you'll need to install now:

- Acados:
```bash
git clone https://github.com/acados/acados.git
```

Follow further installation instructions on their [webpage](https://docs.acados.org/installation/index.html).

Note that for c code generation, the t_renderer executable in `acados/bin` seems to be configured for the x86 architecture by default. If you're working on an ARM-based machine (such as Raspberry Pi), you'll need to obtain and configure the version of the renderer that is compatible with ARM. Check this yourself on [this](https://github.com/acados/tera_renderer/releases/) page.


- CasADi, PyYAML

As Acados uses CasADi and pyYAML, ROS needs to find these libraries, and the best way it worked for me personally was to create a python3 virtual environment:

```bash
python3 -m venv /path/to/directory/virtual_env_name
pip install casadi PyYAML
```

When you plug the Arduino, and the LiDAR sensor into your system via USB, make sure they are seen by your system. Run the following:
```bash
ls /dev/ttyUSB*
ls /dev/ttyACM*
```
If you don't see something like /dev/ttyUSB0 and /dev/ttyACM0, then it might not be connected properly or it is connected to a wrong port. Unplug and plug back in and check again.

If they are detected but ROS2 can not access them, then adjust the permissions:

```bash
sudo chmod 666 /dev/ttyUSB0
sudo usermod -a -G dialout $USER
```

### Testing

```bash
colcon build
```
Now activate your ros workspace and your virtual environment in the following order:

```
source opt/ros/jazzy/setup.bash
source ~/Diffdrive/ros_ws/setup.bash
source virtual_env/name/bin/activate
```

Run all the nodes in seperate terminals (or run the launch file):
```
ros2 run my_cpp_pkg arduino_bridge_node
ros2 run my_py_pkg mpc_diffdrive_control
ros2 topic pub /diffdrive/target_pos geometry_msgs/msg/Vector3 "{x: 0.0, y: 0.0, z: 0.0}"
```

Happy experimenting!