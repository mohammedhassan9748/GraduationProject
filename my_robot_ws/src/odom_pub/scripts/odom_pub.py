#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Point32, Quaternion
from nav_msgs.msg import Odometry
import math

# Robot parameters
R = 0.07         # Wheel radius
L = 0.37         # Distance between wheels
N = 360.0        # Number of encoder ticks per revolution

# Initialize encoder values and pose
right_ticks = 0
left_ticks = 0
x = 0.0
y = 0.0
th = 0.0

# Previous encoder values
prev_right_ticks = 0
prev_left_ticks = 0

# Previously global, now initialized within main()
prev_time = None

def encoder_callback(msg):
    global right_ticks, left_ticks
    right_ticks = msg.x  # Right encoder ticks
    left_ticks = msg.y  # Left encoder ticks

def update_odometry(event):
    global prev_right_ticks, prev_left_ticks, prev_time, x, y, th

    current_time = rospy.Time.now()
    dt = (current_time - prev_time).to_sec()
    if dt == 0:
        return

    # Calculate tick differences
    delta_right = right_ticks - prev_right_ticks
    delta_left = left_ticks - prev_left_ticks

    # Calculate wheel distances
    dr = (2 * math.pi * R * delta_right) / N
    dl = (2 * math.pi * R * delta_left) / N

    # Calculate average distance and change in angle
    dc = (dr + dl) / 2.0
    dth = (dr - dl) / L

    # Update pose
    x += dc * math.cos(th)
    y += dc * math.sin(th)
    th += dth

    # Update previous values
    prev_right_ticks = right_ticks
    prev_left_ticks = left_ticks
    prev_time = current_time

    # Create and publish the odometry message
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'chassis'

    # Set position
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.position.z = 0.0
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
    odom.pose.pose.orientation = Quaternion(*odom_quat)

    # Set velocity
    odom.twist.twist.linear.x = dc / dt
    odom.twist.twist.linear.y = 0.0
    odom.twist.twist.angular.z = dth / dt

    # Publish the odometry message
    odom_pub.publish(odom)

    # Broadcast the transform
    odom_broadcaster.sendTransform(
        (x, y, 0),
        odom_quat,
        current_time,
        "chassis",
        "odom"
    )

def main():
    global odom_pub, odom_broadcaster, prev_time

    rospy.init_node('odometry_publisher')
    prev_time = rospy.Time.now()  # Initialize time after the node has started
    rospy.Subscriber('/encoder', Point32, encoder_callback)
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()

    # Use a Timer to call the update_odometry function at a fixed rate
    rospy.Timer(rospy.Duration(0.05), update_odometry)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass