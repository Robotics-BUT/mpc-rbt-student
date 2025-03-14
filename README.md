# MPC-RBT Student

<div align="center">

[![Tests](https://github.com/Robotics-BUT/mpc-rbt-student/actions/workflows/test.yml/badge.svg?branch=main)](https://github.com/Robotics-BUT/mpc-rbt-student/actions/workflows/test.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

</div>

This repository is a template for all student solutions for lab tasks in the MPC-RBT course. The `main` branch implements a ROS 2 package `mpc-rbt-solution` that will serve as the main project for the duration of the MPC-RBT course. Other branches will contain templates for other introductory labs.

## Installation

Follow installation steps in the [mpc-rbt-simulator](https://github.com/Robotics-BUT/mpc-rbt-simulator) repository to install the required versions of Ubuntu, ROS, and Webots, and to prepare your workspace.

Create a fork of this repository, navigate to your created workspace `src` directory (e.g., `mpc-rbt_ws/src`) and clone it.

Install any additional missing dependencies using the following command:

```
rosdep install --from-paths ./ -y -r --ignore-src --rosdistro humble
```

## Package Structure

>TODO

## Usage

Navigate to the workspace root directory (`mpc-rbt_ws`) and build it:

```
colcon build
```

Set up the environment for the workspace:

```
source install/setup.bash
```

Launch all your solutions using:

```
ros2 launch mpc-rbt-solution solution.launch.py
```

## Testing

Navigate to the workspace root directory (`mpc-rbt_ws`) and build it:

```
colcon build
```

Run tests using:

```
colcon test --ctest-args tests
```

View the results using:
```
colcon test-result --verbose --all
```
