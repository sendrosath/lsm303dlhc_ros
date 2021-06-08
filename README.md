# lsm303dlhc_ros
A  ROS Wrapper for Adafruit LSM303DLHC IMU 

## Introduction
This is a ROS package for publishing (Adafruit) LSM303(DLHC) 6DOF IMU data on Jetson AGX Xavier using I2C. It is tested on Python 3.6 using the Adafruit LSM303 libraries. This package is my take on [OOyindamola's lsm9ds1_ros_python](https://github.com/OOyindamola/lsm9ds1_ros_python).

## Connection
At first I had some problems connecting the IMU to the Jetson AGX Xavier so after some tinkering I foung what worked for me:

   * **VIN** - Pin 2 *(5V)*
   
   * **GND** - Pin 6 *(GND)*
   
   * **SDA** - Pin 3 *(I2C_GP5_DA, I2C Bus 8)*
   
   * **SCL** - Pin 5 *(I2C_GP5_CLK, I2C Bus 8)*

You can use [this guide](https://www.jetsonhacks.com/nvidia-jetson-agx-xavier-gpio-header-pinout/) to better understand the GPIO pinout of the Jetson AGX Xavier. 
   
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

## Check if your IMU is recognised by your GPIO
As mentioned earlier the pins we used mean that the I2C connection is managed by the Bus 8.

To check if your IMU is sending and receiving data you run:
```
$ sudo i2cdetect -r -y 8
```
and you should be able to see something like the following:

![image](https://user-images.githubusercontent.com/58865448/121136329-32c17980-c83e-11eb-957a-0094d72333f8.png)

## Installation
This package has been tested on ROS Melodic (Ubuntu 18.04) so Melodic is the only distro that I can vouch it is going to work. 

Create a catkin workspace and clone lsm303dlhc_ros:
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/sendrosath/lsm303dlhc_ros
```
Install catkin dependencies (if you haven't already):
```
sudo pip3 install catkin_pkg
```
Build Workspace with Python 3:
```
$ cd ~/catkin_ws
$ catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```
Make sure your files are executables:
```
$ chmod +x src/lsm303dlhc_ros/launch/imu_publisher.launch
$ chmod +x src/lsm303dlhc_ros/scripts/imu_data_publisher.py 
```
> **Note** This will configure catkin_make with Python 3. You may then proceed to use just catkin_make for subsequent builds.

**Warning** Before you catkin_make make sure you have the LSM303DLHC module. Otherwise, if you have LSM303AGR you gonna need to make the following changes on [scripts/imu_data_publisher.py](scripts/imu_data_publisher.py):

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
roslaunch lsm303dlhc_ros imu_publisher.launch
```

![image](https://user-images.githubusercontent.com/58865448/121135712-72d42c80-c83d-11eb-94f6-ad659d233972.png)

## References
1. Jetson AGX Xavier GPIO Pinout - https://www.jetsonhacks.com/nvidia-jetson-agx-xavier-gpio-header-pinout/
2. Adafruit CircuitPython Library - https://github.com/adafruit/Adafruit_CircuitPython_LSM9DS1
3. Configuring ROS Environment with Python 3 - http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
4. OOyindamola's lsm9ds1_ros_python - https://github.com/OOyindamola/lsm9ds1_ros_python
