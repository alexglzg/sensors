#!/usr/bin/env python

import os
import time
import rospy
#from geometry_msgs.msg import Pose2D
from std_msgs.msg import UInt16


class Test:
    def __init__(self):
        self.testing = True

        self.dt = 0

        #rospy.Subscriber("ref", Pose2D, self.ref_callback)

        self.d_thrust_pub = rospy.Publisher("rpwm", UInt16, queue_size=10)
        #self.d_heading_pub = rospy.Publisher("desired_heading", Float64, queue_size=10)

    #def ref_callback(self, refh):
        #self.reference_heading = refh.theta

    def desired(self, thrust):
        #while not rospy.is_shutdown():
        self.dt = thrust
        self.d_thrust_pub.publish(self.dt)

def main():
    rospy.init_node('rosserialtest', anonymous=True)
    rate = rospy.Rate(100) # 100hz
    t = Test()
    while not rospy.is_shutdown():
        if t.testing:
            start_time = rospy.Time.now().secs
            while (rospy.Time.now().secs - start_time) <= 1:
                t.desired(1500)
                rate.sleep()
            while (rospy.Time.now().secs - start_time) <= 3:
                t.desired(1570)
                rate.sleep()
            while (rospy.Time.now().secs - start_time) <= 6:
                t.desired(1540)
                rate.sleep()
            t.desired(1500)
            t.testing = False
            rospy.logwarn("Finished")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass