#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point32

def run():
    rospy.init_node("pid_setter", anonymous=True)

    pub = rospy.Publisher("set_pid", Point32, queue_size=10)
    gain_msg = Point32()

    while not rospy.is_shutdown():
        s = raw_input("Enter PID gain, i.e. Kp Ki Kd: ")
        gains = s.split(" ")

        if len(gains) > 3:
            continue

        gain_msg.x = float(gains[0])
        gain_msg.y = float(gains[1])
        gain_msg.z = float(gains[2])
        pub.publish(gain_msg)

if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
