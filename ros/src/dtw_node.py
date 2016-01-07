#!/usr/bin/env python
import rospy
from appliance_oracle.msg import Int16Array
from dtw import dtw
import os
from ast import literal_eval
import matplotlib.pyplot as plt

signal_signatures = []

#signal is a np array of current readings
def score_all_signatures(signal):
    global signal_signatures
   # plt.plot(signal.data)
   # plt.show()

    if not signal_signatures:
        new_appliance(signal)
        return
    else:
        rospy.loginfo("Recognizing signature")
        signatures = [sig[1] for sig in signal_signatures]
        appliance_idx = 0

        # Recognize similarity using DTW
        for sig in signatures:
            dist,cost,path = dtw(signal, sig)
            matches.append( (signal_signatures[appliance_idx][0], dist) )
            appliance_idx += 1
        return matches


def recognize_signtaure(signal):
    global signal_signatures
   # plt.plot(signal.data)
   # plt.show()

    if not signal_signatures:
        new_appliance(signal.data)
    else:
        rospy.loginfo("Recognizing signature")
        signatures = [sig[1] for sig in signal_signatures]
        appliance_idx = 0
        dist_thresh = 1000
        best_match = 'NaN'

        #confusion = []

        # Recognize similarity using DTW
        for sig in signatures:
            dist,cost,path = dtw(signal.data, sig)
            #confusion.append(dist)
            if (dist < dist_thresh):
                best_match = signal_signatures[appliance_idx][0], dist
                dist_thresh = dist
            appliance_idx += 1

        #print "Confusion:"
        #print confusion
        #print best_match[1]
        if (best_match[1] > 50):
            rospy.loginfo("New appliance detected")
            new_appliance(signal.data)
        else:
            rospy.loginfo("Identified appliance: %s [dist: %d]" % (best_match[0], best_match[1]))

def new_appliance(signature):
    global signal_signatures
    resp = raw_input('New appliance?:')
    if (resp == 'y'):
        appliance_name = raw_input('Enter name for appliance:')
        signal_signatures.append([appliance_name, signature])
        print "Saved"

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.Subscriber("edge", Int16Array, recognize_signtaure)

    # spin() simply keeps python from exiting until this node is stopped
    while not rospy.is_shutdown():
        rospy.spin()

    # Node is shutting down. Save recorded signatures
    if signal_signatures:
        f = open('signatures.log', 'w')
        for item in signal_signatures:
            print >> f, item

        f.close()

if __name__ == '__main__':
    rospy.init_node('dtw', anonymous=True)

    if os.path.exists('signatures.log'):
        with open('signatures.log') as f:
            signal_signatures = [list(literal_eval(line)) for line in f]

    listener()
