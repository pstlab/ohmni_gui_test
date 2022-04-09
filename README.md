# Ohmni-robot GUI test
Basic ROS node running a web server for testing the Ohmni robot GUI.

## Installation

This package uses [Crow](https://crowcpp.org/) for implementing the microservices.

1. Download Crow's source code
```
git clone https://github.com/CrowCpp/Crow.git
```
2. Run `mkdir` build inside of crow's source directory
3. Navigate to the new "build" directory and run the following:
```
cmake .. -DCROW_BUILD_EXAMPLES=OFF -DCROW_BUILD_TESTS=OFF
```
4. Run `make install`.

We can now download this package.

1. From the `src` directory of a Catkin workspace, clone this package.
```
git clone https://github.com/pstlab/ohmni_gui_test
```
3. Build the catkin workspace either through
```
catkin build
```
or through
```
catkin_make
```
3. Source the catkin workspace
```
source devel/setup.bash
```
4. If the node is not executed on the robot, change the host and the port in the `launch/ohmni.launch` file and in the `static/main.js`.
5. Execute the ROS node.
```
roslaunch robot_gui ohmni.launch
```
6. Load the `http://127.0.0.1:8080/` page on the robot, taking care to replace host and port acording to the changes, if any, made in point 4.