# Massage Bed System with ROS

This repository is a ros package for base system of massage bed.
This project is maintained by Toyohashi University of Technology and Aoyama University.



## 1. Description

This package has following source codes in `src` directory.

### a. Control node

The source codes below are like moving a robot or getting value from a sensor. They become a part of application.

| File name                    | Language | Input                                                        | Output             | Description                                                  | How to run                                      |
| ---------------------------- | -------- | ------------------------------------------------------------ | ------------------ | ------------------------------------------------------------ | ----------------------------------------------- |
| `techman_control_bridge.cpp` | C++      | Techman end-effector velocity command from non-ROS program such as Unity in Windows. | Output is nothing. | This program receives Techman end-effector velocity command via TCP and control Techman. | `rosrun ros_massage_bed techman_control_bridge` |
|                              |          |                                                              |                    |                                                              |                                                 |



### b. Application node

The source codes below are sample for demonstration such as simple massage for human.

| File name                  | Language | Input                 | Output                                     | Description                                                  | How to run                                    |
| -------------------------- | -------- | --------------------- | ------------------------------------------ | ------------------------------------------------------------ | --------------------------------------------- |
| `simple_force_massage.cpp` | C++      | Leptrino sensor data. | Techman velocity command for good massage. | This program allow Techman to do massage. This program make Techman keep constant force along z-axis and move sin wave along x-axis. The movie of the this demonstration was shown at the monthly meeting. | `rosrun ros_massage_bed simple_force_massage` |



### c. Test and tool

The source code below are convenient if you get problems and used in order to verify the functions of `control node` programs.

| File name                         | Language | Input | Output                                         | Description                                                  | How to run                                           |
| --------------------------------- | -------- | ----- | ---------------------------------------------- | ------------------------------------------------------------ | ---------------------------------------------------- |
| `test_techman_control_bridge.cpp` | C++      | NA    | Techman velocity command every 5 milliseconds. | This program is used for test of `techman_control_bridge.cpp` | `rosrun ros_massage_bed test_techman_control_bridge` |



## 2. Dependencies

- Ubuntu 20.04 or higher
- [Boost ](https://www.boost.org/)
- [Poco](https://pocoproject.org/)

