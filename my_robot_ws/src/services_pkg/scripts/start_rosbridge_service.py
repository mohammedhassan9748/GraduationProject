#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerResponse

def start_rosbridge(req):
    # Execute command to start ROSBridge server (replace with actual command)
    # For example: subprocess.Popen(["roslaunch", "rosbridge_server", "rosbridge_websocket.launch"])
    rospy.loginfo("ROS Bridge Server started.")
    return TriggerResponse(success=True, message="ROS Bridge Server started successfully.")

def start_rosbridge_server():
    rospy.init_node('start_rosbridge_server')
    rospy.Service('/start_rosbridge', Trigger, start_rosbridge)
    rospy.loginfo("Ready to start ROSBridge server.")
    rospy.spin()

if __name__ == "__main__":
    start_rosbridge_server()

