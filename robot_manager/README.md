# Structure of this program

This is a ROS program that is divided up into the following directories:

- include/robot_manager: The program header files associated with the contents of src
- msg: Message(.msg) files
- src: The program cpp files
- test: Independent testing programs mostly written in C and compiled using gcc. 

The root also contains the following files:

- CMakeLists.txt: Cmake configurations
- package.xml: Defines the ROS package
- config.yaml: Sample configuration file

# Testing data and programs

Test programs that can be run within robot_manager include:

- example.c: prints incrementing numbers infinitely
- scrolltest.c: prints incrementing numbers up to a given amount provided as a parameter

Add the path to the compiled executables to the configuration file for testing purposes

Other programs that were used for testing new concepts during development include:

pty.c: Testing forking and creating pseudo-terminals, requires -lutil and -lncurses compilation flags

The monitoring_test directory is a separate ROS application that uses the same messages in msg.
Move this directory to the ~/catkin_ws/src directory, compile using catkin_make and run using:
$ rosrun monitoring_test monitoring_test_node

# Third Party Dependencies

These dependencies were not included as part of the application either due to impracticality or the high likelihood that they are already met:

- yaml-cpp: https://github.com/jbeder/yaml-cpp
- ncurses: https://www.gnu.org/software/ncurses/ncurses.html
- boost: https://www.boost.org/
- ROS: http://www.ros.org/

# Building and running

Add the program directory (robot_manager) to the ROS catkin workspace (presumably ~/catkin_ws/src/) and run catkin_make.
The configuration file should be stored in ~/.config/robot_manager/config.yaml however a sample file is provided and will be used automatically instead.
To use the test programs, add your user name to the indicated sections in the config file and compile the relevant programs using gcc.
Run roscore and then run:
$ rosrun robot_manager robot_manager_node

# Key Bindings

- Arrow keys: Control the robot
- Specbar: Stop the robot
- k/j: Menu cursor Up/Down
- h/l: Tab bar cursor Left/Right
- x: Close current tab
- Enter: Open selected menu item
- m/n: Scroll output Up/Down
- p/o: Scroll menu Up/Down
- q: Exit the application
