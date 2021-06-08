# lsm303dhlc_ros
A  ROS Wrapper for Adafruit LSM303DLHC IMU 

## Introduction
This is a ROS package for publishing (Adafruit) LSM303(DLHC) 6DOF IMU data on Jetson AGX Xavier using I2C. It is tested on Python 3.6 using the Adafruit LSM303 libraries. This package is my take on [OOyindamola's lsm9ds1_ros_python](https://github.com/OOyindamola/lsm9ds1_ros_python).

## Connection
At first I had some problems connecting the IMU to the Jetson AGX Xavier so after some tinkering I foung what worked for me:

   * **VIN** - Pin 2 *(5V)*
   
   * **GND** - Pin 6 *(GND)*
   
   * **SDA** - Pin 3 *(I2C_GP5_DA, I2C Bus 8)*
   
   * **SCL** - Pin 5 *(I2C_GP5_CLK, I2C Bus 8)*

You can use [this guide](https://www.jetsonhacks.com/nvidia-jetson-agx-xavier-gpio-header-pinout/) to better understand the GPIO pinout. 
   
## Dependencies

In order to install this package you are going to need:

1. Python 3.6
2. pip3:
```
$ sudo apt-get update
$ sudo apt-get -y install python3-pip
```
And verify you installed it:
```
$ pip3 --version
```
3. LSM303 libraries:
```
$ sudo pip3 install adafruit-circuitpython-lsm303-accel
```
* LSM303AGR:
```
$ sudo pip3 install adafruit-circuitpython-lis2mdl
```
* LSM303DLH:
```
$ sudo pip3 install adafruit-circuitpython-lsm303dlh-mag
```
You can find more about the LSM303 python libraries [here](https://learn.adafruit.com/lsm303-accelerometer-slash-compass-breakout/python-circuitpython)

## Installation

This package has been tested on ROS Melodic (Ubuntu 18.04) so Melodic is the only distro that I can vouch it is going to work. 

Create a catkin workspace and clone lsm303dlhc_ros:
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/sendrosath/lsm303dhlc_ros
```
Install catkin dependencies (if you haven't already)
```
sudo pip3 install catkin_pkg
```
Build Workspace with Python 3
```
$ cd ~/catkin_ws
$ catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```
> **Note** This will configure catkin_make with Python 3. You may then proceed to use just catkin_make for subsequent builds.

**Warning** Before you catkin_make make sure you have the LSM303DHLC module. Otherwise you gonna need to make the following changes on [scripts/imu_data_publisher.py](scripts/imu_data_publisher.py).

For LSM303AGR you need to make the following changes:

* on line 4:
```
import adafruit_lis2mdl
```
* and on line 13:
```
mag = adafruit_lis2mdl.LIS2MDL(i2c)
```
## Launch

```
roslaunch lsm303dhlc_ros imu_publisher.launch
```

## References
1. Adafruit CircuitPython Library - https://github.com/adafruit/Adafruit_CircuitPython_LSM9DS1
2. Configuring ROS Environment with Python 3 - http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
3. OOyindamola's lsm9ds1_ros_python - https://github.com/OOyindamola/lsm9ds1_ros_python
