#!/usr/bin/env

import rospy
from rosgraph_msgs.msg import Clock
from datetime import datetime

def publish_time():
    rospy.init_node('clock_publisher', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz

    clock_pub = rospy.Publisher('/clock', Clock, queue_size=10)

    while not rospy.is_shutdown():
        current_time = rospy.get_rostime()
        clock_msg = Clock()
        clock_msg.clock = current_time
        rospy.loginfo("Publishing current time: %s", current_time)
        clock_pub.publish(clock_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_time()
    except rospy.ROSInterruptException:
        pass

