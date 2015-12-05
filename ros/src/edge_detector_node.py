#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
#from std_msgs.msg import Int16MultiArray
from appliance_oracle.msg import Int16Array
from pykalman import KalmanFilter
import numpy as np
import matplotlib.pyplot as plt

buf = Int16Array()
buf_capacity = 180

thresh = 200
trigger_flag = False
zero_flag = True

wait_counter = 0
buf_counter = 0

baseline_voltage = 0

# create a Kalman Filter by hinting at the size of the state and observation
# space.  If you already have good guesses for the initial parameters, put them
# in here.  The Kalman Filter will try to learn the values of all variables.
kf = KalmanFilter(transition_matrices=np.array([[1, 1], [0, 1]]),
                  transition_covariance=0.01 * np.eye(2))

def normList(L, normalizeTo=1):
    vMax = max(L)
    return [ x/(vMax*1.0)*normalizeTo for x in L]

def edge_detector(sensor_val):
    global wait_counter
    global zero_flag
    global trigger_flag
    global buf
    global buf_counter
    global baseline_voltage

    buf.data.append(sensor_val.data)
    if (len(buf.data) > buf_capacity):
        del buf.data[0]

    if (len(buf.data) == buf_capacity):
        if (trigger_flag == False):
            baseline_voltage = sum(buf.data[0:10])/10.0
        #print baseline_voltage

        #if diff > 100:
        #rospy.loginfo(rospy.get_caller_id() + "I heard %d", data.data)
       # average_voltage = sum(buf.data[buf_capacity-10:buf_capacity])/10.0
        #print 'avg, baseline %d %d' % (average_voltage, baseline_voltage)
        
        if (sensor_val.data - baseline_voltage > thresh):
            if ((trigger_flag == False) and (zero_flag == True)):
                trigger_flag = True     
                rospy.loginfo("Capturing appliance signature")

            wait_counter = 0
            zero_flag = False
        else:
            wait_counter += 1
            if (wait_counter > buf_capacity):
                trigger_flag = False
                zero_flag = True
                buf.data = []
                print '.'
                wait_counter = 0

        if (trigger_flag):
            if (buf_counter < buf_capacity ):
                buf_counter += 1
            else:
                buf_counter = 0
                rospy.loginfo("Capturing complete")

                buf.data[:] = [value - baseline_voltage for value in buf.data]
                
                # Kalman smoothing
                states_pred = kf.em(buf.data).smooth(buf.data)[0]
                states_pred = states_pred.tolist()
                buf.data = [item[0] for item in states_pred]

                # Normalize data
                #buf.data = normList(buf.data)
    
                pub.publish(buf)

                buf.data = []
                trigger_flag = False
                baseline_voltage = sum(buf.data[0:10])/10.0


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.


    rospy.Subscriber("power", Int16, edge_detector)

    # spin() simply keeps python from exiting until this node is stopped
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('edge_detector', anonymous=True)
    pub = rospy.Publisher('edge', Int16Array, queue_size=10)
    listener()
