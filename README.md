# Differential Drive - ROS2

## Overview
This repo contains an experimental ROS2-based project that integrates Raspberry Pi (running ROS2) with an ESP32 microcontroller to control a two-wheeled differential drive robot. My personal goal for this project was to bridge the gap between advanced control algorithms in simulation vs actual hardware.

### Structure

The repository is organized as a ROS2 workspace (`ros_ws`) containing several packages, and an `MCU` folder with the microcontroller firmware. This structure separates high-level control logic from low-level hardware interfacing.

#### Key components
- MCU Firmware (`MCU/src/`): Runs on a ESP32 microcontroller. It interfaces with the MD25 motor driver (for motor control and encoder readings) and implements a PID control loop for motor speeds. It receives high-level velocity commands over serial which are then transformed into wheel speeds by calculating the inverse kinematics. It also sends back encoder counts and wheel speeds over serial at 50 Hz.

- [MCU Bridge Node](https://github.com/DPMA00/DiffDrive-ROS/blob/master/ros_ws/src/my_cpp_pkg/src/mcu_bridge_node.cppy): A C++ ROS2 node that connects the Raspberry Pi to the microcontroller via a UART/USB serial link. It opens the serial port at the configured baud rate and handles three (currently used) ROS topics to interface with the hardware.

- [MPCC Controller Node](https://github.com/DPMA00/DiffDrive-ROS/blob/master/ros_ws/src/my_py_pkg/my_py_pkg/mpcc_diffdrive_control.py) (**Minor issues still persist**): This node implements a model predictive contouring controller for trajectory following. State feedback is obtained by subscribing to the `robot/odom` topic, which provides encoder-based dead-reckoning odometry. The controller assumes a **fixed reference track**, therefore it cannot be updated at runtime.
  
- [MPC Point Control Node](https://github.com/DPMA00/DiffDrive-ROS/blob/master/ros_ws/src/my_py_pkg/my_py_pkg/mpc_diffdrive_control.py) (**Old test node**): A Python ROS2 node that computes robot pose using dead-reckoning odometry. The pose is updated at a fixed time step and used directly as the system state for an MPC point control loop running at 12Hz, with support for cicrcular obstacle avoidance.    


### Hardware

- Motors, Motor Driver and Wheels:  [MD25 - Dual 12Volt 2.8Amp H Bridge Motor Drive for EMG30](https://static.rapidonline.com/pdf/70-6403_v1.pdf)

- Raspberry Pi 5
- ESP32
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

When you plug the microcontroller, and the LiDAR sensor into your system via USB, make sure they are seen by your system. Run the following:
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
ros2 run my_cpp_pkg mcu_bridge_node
ros2 launch my_robot_bringup diffdrive_odom_launch.py
ros2 run my_py_pkg mpcc_diffdrive_control
```

### Bugs and Issues

While the overall formulation of the MPCC is functional, some non-intuitive behaviors still persist in the predicted trajectories at certain points in the track. Moreover, the system currently relies on dead-reckoning from wheel odometry for state estimation. As no external correction or sensor fusion is applied, those pose estimates are subject to drift over time. An improvement would be the integration of an EKF-based state estimator, or a full SLAM implementation.