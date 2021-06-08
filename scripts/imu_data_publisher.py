#!/usr/bin/env python3
import time
import board
import adafruit_lsm303_accel
import adafruit_lsm303dlh_mag

import numpy as np
import rospy
from sensor_msgs.msg import Imu

# I2C connection:
i2c = board.I2C()
mag = adafruit_lsm303dlh_mag.LSM303DLH_Mag(i2c)
accel = adafruit_lsm303_accel.LSM303_Accel(i2c)

def read_data():
    pub = rospy.Publisher('/imu/data_raw', Imu, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    imu_msg = Imu()
    
    while not rospy.is_shutdown():
        # Read acceleration, magnetometer, gyroscope, temperature.
        accel_x, accel_y, accel_z = accel.acceleration
        mag_x, mag_y, mag_z = mag.magnetic
        
        #Header
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"
        
        #linear accelerations from accelerometer
        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z
        
        # Print values.
        print('Acceleration (m/s^2): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(accel_x, accel_y, accel_z))
        print('Magnetometer (gauss): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(mag_x, mag_y, mag_z))
        
        pub.publish(imu_msg)
        rate.sleep()

        
if __name__ == '__main__':
    try:
        read_data()
    except rospy.ROSInterruptException:
        pass
