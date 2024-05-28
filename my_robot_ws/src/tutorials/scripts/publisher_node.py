#!/usr/bin/env python

import rospy
from std_msgs.msg import String


def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1 Hz
    while not rospy.is_shutdown():
        # Get user input
        
        user_input = input("Enter a message: ")  # For Python 3

        # Publish the user input as a string message
        pub.publish(user_input)
        rospy.loginfo("Published: %s" % user_input)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
