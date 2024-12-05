# HIWIN Robot Client Library

**HIWIN Robot Client Library** is a lightweight and efficient C++ library for controlling HIWIN robots. This library provides functionalities to perform basic motion control, speed configuration, and other essential robotic operations.

## Features
- **Basic Motion Control**: 
  - Joint space motion.
  - Cartesian space motion.
- **Speed and Acceleration Settings**: 
  - Control of joint and Cartesian velocities.
  - Acceleration and deceleration customization.
- **State Monitoring**: 
  - Read robot state, position, and joint values.
  - Error and status monitoring.

## Getting Started

### Prerequisites
  - **Operating System**: Ubuntu 20.04 or later.
  - **Linux socket communication**: Socket communication is currently based on Linux sockets.

### Installation
1. Clone the repository:
```bash
git clone https://github.com/HIWINCorporation/hiwin_robot_client_library.git
cd hiwin_robot_client_library
```
2. Build the library:
```bash
mkdir build && cd build
cmake ..
make
```
3. Install the library:
```bash
sudo make install
```

## Usage Example
Here is a quick example of how to use the library to control a HIWIN robot:
```cpp
#include <iostream>
#include <vector>
#include <hiwin_robot_client_library/hiwin_driver.hpp>

int main() {
    // Define the robot's IP address
    const std::string robot_ip = "192.168.0.1";

    // Initialize the HIWIN driver
    hrsdk::HIWINDriver robot_driver(robot_ip);

    // Connect to the robot
    if (!robot_driver.connect()) {
        std::cerr << "Failed to connect to the robot at " << robot_ip << std::endl;
        return -1;
    }
    std::cout << "Successfully connected to the robot!" << std::endl;

    // Send a joint space command (example joint positions)
    std::vector<double> target_positions = {0.0, -0.5, 0.3, 0.0, 1.0, -0.2}; // In radians
    float goal_time = 2.0;  // Move to target positions in 2 seconds
    robot_driver.writeJointCommand(target_positions, goal_time);
    std::cout << "Sent joint command to the robot." << std::endl;

    // Retrieve and display the current joint positions
    std::vector<double> current_positions;
    robot_driver.getJointPosition(current_positions);
    std::cout << "Current joint positions: ";
    for (const auto& pos : current_positions) {
        std::cout << pos << " ";
    }
    std::cout << std::endl;

    // Retrieve and display the robot's current mode
    hrsdk::ControlMode robot_mode;
    robot_driver.getRobotMode(robot_mode);
    std::cout << "Robot mode: " << static_cast<int>(robot_mode) << std::endl;

    // Check if the robot is in an error state
    if (robot_driver.isInError()) {
        std::cerr << "Robot is in an error state!" << std::endl;
    } else {
        std::cout << "Robot is functioning normally." << std::endl;
    }

    // Abort any motion (if necessary)
    robot_driver.motionAbort();
    std::cout << "Motion aborted." << std::endl;

    // Disconnect from the robot
    robot_driver.disconnect();
    std::cout << "Disconnected from the robot." << std::endl;

    return 0;
}

```