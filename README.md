# Ohmni-robot GUI test
Basic ROS node running a web server for testing the Ohmni robot GUI.

## Installation

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

