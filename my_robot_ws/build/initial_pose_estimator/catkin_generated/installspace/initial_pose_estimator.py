#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.srv import GetMap
from fast_laser_scan_matcher import fast_match

class InitialPoseEstimator:
    def __init__(self):
        rospy.init_node('initial_pose_estimator', anonymous=True)
        
        self.map = None
        self.map_resolution = None
        self.map_origin = None
        
        self.get_map()
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        
        self.initialized = False

    def get_map(self):
        rospy.wait_for_service('static_map')
        try:
            map_service = rospy.ServiceProxy('static_map', GetMap)
            response = map_service()
            self.map = np.array(response.map.data).reshape((response.map.info.height, response.map.info.width))
            self.map_resolution = response.map.info.resolution
            self.map_origin = (response.map.info.origin.position.x, response.map.info.origin.position.y)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def laser_callback(self, scan):
        if self.initialized:
            return
        
        laser_ranges = np.array(scan.ranges)
        laser_ranges[laser_ranges == float('inf')] = scan.range_max
        angles = np.linspace(scan.angle_min, scan.angle_max, len(laser_ranges))
        
        x_coords = laser_ranges * np.cos(angles)
        y_coords = laser_ranges * np.sin(angles)
        
        # Convert to map coordinates
        x_coords_map = (x_coords - self.map_origin[0]) / self.map_resolution
        y_coords_map = (y_coords - self.map_origin[1]) / self.map_resolution
        
        # Filter valid points
        valid_indices = (x_coords_map >= 0) & (x_coords_map < self.map.shape[1]) & (y_coords_map >= 0) & (y_coords_map < self.map.shape[0])
        x_coords = x_coords[valid_indices]
        y_coords = y_coords[valid_indices]
        
        # Use fast_match to find the best match
        initial_x, initial_y, initial_yaw = fast_match(self.map, x_coords, y_coords, self.map_resolution, self.map_origin)
        
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.header.stamp = rospy.Time.now()
        
        initial_pose.pose.pose.position.x = initial_x
        initial_pose.pose.pose.position.y = initial_y
        
        quaternion = tf.transformations.quaternion_from_euler(0, 0, initial_yaw)
        initial_pose.pose.pose.orientation.x = quaternion[0]
        initial_pose.pose.pose.orientation.y = quaternion[1]
        initial_pose.pose.pose.orientation.z = quaternion[2]
        initial_pose.pose.pose.orientation.w = quaternion[3]
        
        self.initial_pose_pub.publish(initial_pose)
        rospy.loginfo("Initial pose published")
        
        self.initialized = True

if __name__ == '__main__':
    try:
        InitialPoseEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass