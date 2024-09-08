
# USI Angry Turtle

This ROS2 package, `usi_angry_turtle`, controls a "turtle" in the `turtlesim` simulation. The turtle displays "angry" behavior by moving in erratic patterns. This project is designed to demonstrate basic ROS2 skills, including node creation, service handling, and parameter setting.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Package Structure](#package-structure)
- [Installation](#installation)
- [Running the Node](#running-the-node)
- [Usage](#usage)
- [Skills Demonstrated](#skills-demonstrated)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

## Overview

The `usi_angry_turtle` package provides a Python-based ROS2 node that interacts with the `turtlesim` simulation. The node, named `angry_turtle_controller`, controls a turtle's movement, making it "angry" by having it move unpredictably. The project is an educational tool, designed to help users understand basic ROS2 concepts like nodes, parameters, and services.

## Features

- **Random Movement:** The turtle moves in random directions with variable speeds, simulating erratic behavior.
- **Customizable Behavior:** Parameters can be adjusted to control the speed, frequency of direction changes, and other aspects of the turtle's behavior.
- **ROS2 Integration:** Demonstrates how to create and run ROS2 nodes, handle services, and manage parameters.

## Package Structure

The package is organized as follows:

```
usi_angry_turtle/
├── CMakeLists.txt                  # CMake build script
├── package.xml                     # Package metadata
├── src/
│   └── angry_turtle_controller.py  # Main ROS2 node script
└── setup.py                        # Setup script (optional for Python packages)
```

### Node: `angry_turtle_controller`

This is the main node that controls the turtle's movement in the `turtlesim` simulation. It subscribes to topics and can call services to reset the turtle's position or change its background color.

## Installation

Follow these steps to install and build the package:

### Prerequisites

- ROS2 Humble or later
- `turtlesim` package installed (usually comes with ROS2)
- Python 3.8 or later

### Building the Package

1. **Clone the Repository:**
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/your-username/usi_angry_turtle.git
   ```

2. **Build the Package:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select usi_angry_turtle
   ```

3. **Source the Workspace:**
   ```bash
   source install/setup.bash
   ```

## Running the Node

After building the package, you can run the `angry_turtle_controller` node:

```bash
ros2 run usi_angry_turtle angry_turtle_controller
```

Make sure to have the `turtlesim` simulator running:

```bash
ros2 run turtlesim turtlesim_node
```

## Usage

Once the node is running, the turtle will start moving erratically within the `turtlesim` window. You can customize the turtle's behavior by modifying parameters or sending commands via ROS2 services.

### Customizing Parameters

You can customize the turtle's behavior by setting ROS2 parameters. For example, you can adjust the speed or frequency of direction changes:

```bash
ros2 param set /angry_turtle_controller speed 2.0
ros2 param set /angry_turtle_controller turn_frequency 1.0
```

### Available Services

The node can interact with the following services:

- **Reset the Turtle:** Resets the turtle's position to the center.
  ```bash
  ros2 service call /reset std_srvs/srv/Empty
  ```

- **Change Background Color:** Changes the background color of the `turtlesim` window.
  ```bash
  ros2 service call /clear std_srvs/srv/Empty
  ```

## Skills Demonstrated

The `usi_angry_turtle` package showcases several key ROS2 skills:

1. **Node Creation:** Understanding how to create and run a ROS2 node in Python.
2. **Parameter Management:** Demonstrates how to use ROS2 parameters to control node behavior.
3. **Service Handling:** Interacts with ROS2 services to reset the turtle's position and change the background color.
4. **Topic Subscription:** Although not used explicitly, it lays the groundwork for subscribing to topics for more complex behaviors.

## Troubleshooting

### Package Not Found

If you encounter the error `Package 'usi_angry_turtle' not found`, make sure you have:

1. Built the package with `colcon build`.
2. Sourced the setup script with `source install/setup.bash`.

### Node Not Executing

If the node does not seem to execute, ensure the Python script has execution permissions:

```bash
chmod +x src/usi_angry_turtle/src/angry_turtle_controller.py
```

## Contributing

Contributions are welcome! Please fork this repository and submit a pull request for any improvements or fixes.



