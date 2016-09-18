# MAVSlam - Visual odometry & SLAM for PX4

## 

#### Onboard visual odometry for PX4 

Integrate  Intel® RealSense™ R200 depth camera as an oboard visual input source for PX4 using the [UP-Board]( http://www.up-board.org) as companion:

- Estimate position and speed based on stereo depth images up to 60 fps for LPE
- Stream video to MAVGCL with overlays
- Integration with PX4 via MAVLink ([MAVComm](https://github.com/ecmnet/MAVComm) required)

Prerequisites:

- UP-Board running Ubilinux 3.0 with Java 8 stack connected via serial link to PX4 controller

- Intel® RealSense™ R200 (connected with external power supply)

  ​

#### Desktop odometry demo 

A first implementation using Intel® RealSense™ R200 with boofcv on OSX/Linux platforms. Based on https://github.com/IntelRealSense/librealsense and http://boofcv.org/index.php?title=Main_Page.

![https://raw.githubusercontent.com/ecmnet/MAVSlam/master/MAVSlam/RealSense.png](https://raw.githubusercontent.com/ecmnet/MAVSlam/master/MAVSlam/RealSense.png)