#!/usr/bin/env python3
import rospy
import tf
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class EKFFusion:
    def __init__(self):
        rospy.init_node('ekf_fusion')

        # Publishers and subscribers
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.odom_pub = rospy.Publisher('/odom_filtered', Odometry, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()

        # EKF matrices
        self.x = np.zeros((3,1))  # state vector [x, y, theta]
        self.P = np.eye(3) * 0.01  # initial state covariance
        self.F = np.eye(3)  # state transition model
        self.H = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])  # measurement matrix
        self.R = np.diag([0.05, 0.05, 0.01])  # measurement noise covariance matrix (odom)
        self.Q = np.diag([0.01, 0.01, 0.03])  # process noise covariance matrix

        self.last_time = rospy.Time.now()
        self.last_imu_time = rospy.Time.now()

    def odom_callback(self, msg):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        th = self.x[2,0]
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z

        # Predict state based on odometry
        self.x[0] += v * np.cos(th) * dt
        self.x[1] += v * np.sin(th) * dt
        self.x[2] += omega * dt

        self.P = self.F @ self.P @ self.F.T + self.Q

        self.publish_odom()

    def imu_callback(self, msg):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_imu_time).to_sec()
        self.last_imu_time = current_time

        # IMU provides angular velocity
        imu_omega = msg.angular_velocity.z

        # Update orientation estimate from IMU
        self.x[2] += imu_omega * dt

        # Simplistic update assuming IMU is very reliable about rotation
        self.P[2,2] += 0.0001  # small increase in uncertainty

    def publish_odom(self):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'chassis'
        odom.pose.pose.position.x = self.x[0,0]
        odom.pose.pose.position.y = self.x[1,0]
        quaternion = quaternion_from_euler(0, 0, self.x[2,0])
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]
        self.odom_pub.publish(odom)

        # Broadcast the transformation
        self.tf_broadcaster.sendTransform(
            (self.x[0,0], self.x[1,0], 0),
            quaternion,
            rospy.Time.now(),
            'chassis',
            'odom'
        )

if __name__ == '__main__':
    try:
        ekf_node = EKFFusion()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
