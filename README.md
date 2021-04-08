# myahrs_driver

### Note

This driver is forked from [robotpilot](https://github.com/robotpilot/myahrs_driver). Added functionality for some ros parameters for users + troubleshooting steps at the bottom.

## Overview

This is a driver package for the WITHROBOT's myAHRS+ from [lilliputdirect](http://www.lilliputdirect.com/odroid-myahrs|lilliputdirect) and [hardkernel](http://www.hardkernel.com/main/products/prdt_info.php?g_code=G141464363369). The myAHRS+ is a low cost high performance AHRS(Attitude Heading Reference System) with USB/UART/I2C interface. The myAHRS+ board contains a 3-axis 16-bit gyroscope, a 3-axis 16-bit accelerometer and a 3-axis 13-bit magnetometer. The driver should also work with USB port.

### Axes Convention

The myAHRS+ board used NED type. The myahrs_driver contained in this package converts to the frame conventions of ROS (use the east north up (ENU) convention and right hand rule) before publishing the msgs. The driver use the coordinate frame below. Please see [rep-103 standard](http://www.ros.org/reps/rep-0103.html#axis-orientation) for more information.

 * x forward
 * y left
 * z up


 * NED type IMU: x-north, y-east, z-down, relative to magnetic north.
 * ENU type IMU: x-east, y-north, z-up, relative to magnetic north.

### Original Source

The original source (not support ROS) is maintained in [this github (from withrobot)](https://github.com/withrobot/myAHRS_plus) and tutorials are on the [corresponding wiki page](https://github.com/withrobot/myAHRS_plus/tree/master/tutorial). A 3D visualization test like 3D-box is included in this original source. This package used the myAHRS+ SDK below.

## Video

This is a visualization demonstration using RViz (Credits go to robotpilot).

[![test](http://img.youtube.com/vi/j5v5fKppcQo/0.jpg)](http://www.youtube.com/watch?v=j5v5fKppcQo)

## Installation

Install the package from the github:

```sh
cd ~/catkin_ws/src
git clone https://github.com/y-lai/myahrs_driver.git
cd ~/catkin_ws && catkin_make
```

## Run

Run the driver like so:

```sh
rosrun myahrs_driver myahrs_driver _port:=/dev/ttyACM0 
```

or

```sh
roslaunch myahrs_driver myahrs_driver.launch
```

### roslaunch parameters

Added functionality to customise the node outputs. Edit the launch file to change the arguments.

* publish_tf - Boolean for publishing tf (Default true)
* frame_id - Reference frame for tf (Default "imu_link")
* parent_frame_id - Reference frame for parent frame (Default "base_link")
* topic_prefix - Prefix for published topics (Default "imu"). Published topic names become: 
  * <topic_prefix>/data_raw
  * <topic_prefix>/data
  * <topic_prefix>/mag
  * <topic_prefix>/temperature

## Nodes

Official ROS documentation can be found on the [ROS wiki here](http://wiki.ros.org/myahrs_driver).

## Communication Protocol Manual and Forum

The myAHRS+ protocol can be found [here](https://github.com/withrobot/myAHRS_plus/tree/master/tutorial). The Forum for myAHRS+ user can be found [here](http://forum.odroid.com/viewforum.php?f=109).

## References

### References for myAHRS+ board

* http://www.withrobot.com/myahrs_plus_en/
* http://www.withrobot.com/?wpdmact=process&did=MTE4LmhvdGxpbms=
* https://github.com/robotpilot/myAHRS_plus

* http://www.hardkernel.com/main/products/prdt_info.php?g_code=G141464363369
* http://www.lilliputdirect.com/odroid-myahrs

### References for convention of axes and unit

* http://www.ros.org/reps/rep-0003.html
* http://www.ros.org/reps/rep-0103.html
* https://github.com/paulbovbel/rep/blob/master/rep-0145.rst

### References for similar IMU packages

* http://wiki.ros.org/um6
* http://wiki.ros.org/razor_imu_9dof
* https://github.com/KristofRobot/razor_imu_9dof

* http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html

## Troubleshooting (Linux)

If unable to run ```initialise()``` when running the node or roslaunch, this might come from a few reasons.

### 1. USB connection

Double check the USB connection. Unplug and re-plug the USB connection.

Type in terminal:
```
lsusb | grep STM
```

This should show something like below (the device number might be different, but the ID should be the same):

```
Bus 001 Device 081: ID 0483:5740 STMicroelectronics STM32F407
```

To make sure the myAHRS+ has been recognised by your device, type below:
```
dmesg
```

The last line should show something like below indicating the serial port device has been recognised:
```
[33974.021185] cdc_acm 1-1:1.0: ttyACM0: USB ACM device
```

The port name might change depending on how many serial communication devices are connected to your device. (It's possible to be ttyACMx)

### 2. User access rules

It's possible that your user account does not have the access needed to open the port. Confirm that the device is recognised from above.

Check which group has access to the serial communication port using below (ttyACMx depending on which serial port):
```
ls -l /dev/ttyACM0
```

This gives something like:
```
crw-rw---- 1 root dialout 166, 0 Apr  8 18:52 /dev/ttyACM0
```

Check that your user account is part of the dialout group:
```
groups $USER
```

This should give something like below (the main point is to see whether your account is in the dialout group):
```
y-lai : y-lai adm dialout cdrom sudo dip plugdev lpadmin sambashare
```

If it's not, use the line below (requires sudo access to make user changes):
```
sudo usermod -a -G dialout $USER
```

Type in the password, logout and login to your account again. This should allow you to run the node and obtain data from the myAHRS+.

Note: this is the same process if you are using other MCU/AVRs like the arduino in Linux. [AskUbuntu Reference](https://askubuntu.com/questions/899374/arduino-only-works-in-root)
