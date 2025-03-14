# Maxon EPOS2/EPOS4 ROS2 Driver

## Overview

This ROS2 package provides a driver for Maxon EPOS2 and EPOS4 motor controllers, designed for the roboendo project. The driver enables communication with the motor controllers via USB and CAN, allowing position control and real-time feedback.

## Prerequisites

### Dependencies

Ensure you have the following dependencies installed:

- ROS2 (Humble or later recommended)
- `rclcpp` (ROS2 C++ client library)


## Installation

1. **Clone the repository** into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone git@github.com:connorguzi/ros2MaxonDriver.git
   ```
2. **Install the necessary EPOS drivers**
    ```bash
    cd ros2MaxonDriver/
    ./EPOS-Linux-Library-En/EPOS_Linux_Library/install.sh
    ```
3. **Build the package**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select maxon_epos_msgs maxon_driver motor_test
   ```
4. **Source the workspace**:
   ```bash
   source install/setup.bash
   ```

## Custom Messages

This package uses the `maxon_epos_msgs` package, which defines:

- `maxon_epos_msgs/msg/MotorState`
- `maxon_epos_msgs/msg/MotorStates`

Ensure this message package is available in your workspace.

## Usage

### Running the Driver

To start the Maxon motor driver node, run:

```bash
ros2 run maxon_driver maxon_driver
```

### Command Line Arguments
```bash
Usage:
    -h  : this help
    -n  : node id (default 1)
    -d  : device name (EPOS2, EPOS4, default - EPOS4)
    -s  : protocol stack name (MAXON_RS232, CANopen, MAXON SERIAL V2, default - MAXON SERIAL V2)
    -i  : interface name (RS232, USB, CAN_ixx_usb 0, CAN_kvaser_usb 0,... default - USB)
    -p  : port name (COM1, USB0, CAN0,... default - USB0)
    -b  : baudrate (115200, 1000000,... default - 1000000)
    -l  : list available interfaces (valid device name and protocol stack required)
    -r  : list supported protocols (valid device name required)
    -v  : display device version
    -c  : control method (0 for position and 1 for velocity)
```

### Publishing Commands

To send position or velocity commands, publish messages to the appropriate topic (ensure the message structure matches `maxon_epos_msgs::msg::MotorState`).

Example:

```bash
ros2 topic pub /motor_command maxon_epos_msgs/msg/MotorState '{position: 1000, velocity: 500}'
```

### Subscribing to Motor Feedback

To view real-time motor feedback:

```bash
ros2 topic echo /motor_states
```

### Testing the Motors

A provided Python script, `motor_test.py`, allows manual testing of the motors by entering position and velocity values via the terminal. It publishes `MotorStates` messages to the `/maxon_bringup/set_all_states` topic. Currently you must give both a position and velocity command in the format: position, velocity. Only the position is used when in position control and only velocity is used when in velocity control.

To run the test:

```bash
ros2 run motor_test motor_test
```

This script will prompt you to enter position values for four motors, then publish the commands to the appropriate topic.

## Notes

- Always source the workspace before running the node.
- The driver first initializes the USB gateway motor before connecting additional motors via CAN.
- Ensure the motor driver is running before sending commands.

## Contributors

- Connor Guzikowski
