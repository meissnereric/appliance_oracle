#!/usr/bin/env python
# Reads data from current sensor and publishes the data
# Written by Ashish Derhgawen (ashish.derhgawen@gmail.com)
# October, 2015

import rospy
import serial
from std_msgs.msg import Int16

ser = serial.Serial('/dev/ttyACM1', 9600)

def publisher():
    pub = rospy.Publisher('power', Int16, queue_size=10)
    rospy.init_node('power', anonymous=True)
    rate = rospy.Rate(100) # 100 Hz
    while not rospy.is_shutdown():
        sensor_data = ser.readline().rstrip('\r\n')

        if ',' in sensor_data: # Check if data is valid
            timestamp = sensor_data.split(",")[0]
            sensor_val = int(sensor_data.split(",")[1])

            rospy.loginfo(sensor_val)
            pub.publish(sensor_val)

        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
