import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

class ImuAggregator:
    def __init__(self):
        self.imu_pub = rospy.Publisher('imu/data', Imu, queue_size=10)
        rospy.Subscriber('imu/accel', Vector3, self.accel_callback)
        rospy.Subscriber('imu/gyro', Vector3, self.gyro_callback)

        self.accel_data = Vector3()
        self.gyro_data = Vector3()

    def accel_callback(self, msg):
        self.accel_data = msg
        self.publish_imu()

    def gyro_callback(self, msg):
        self.gyro_data = msg
        self.publish_imu()

    def publish_imu(self):
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.linear_acceleration.x = self.accel_data.x
        imu_msg.linear_acceleration.y = self.accel_data.y
        imu_msg.linear_acceleration.z = self.accel_data.z
        imu_msg.angular_velocity.x = self.gyro_data.x
        imu_msg.angular_velocity.y = self.gyro_data.y
        imu_msg.angular_velocity.z = self.gyro_data.z

        self.imu_pub.publish(imu_msg)

if __name__ == '__main__':
    rospy.init_node('imu_aggregator')
    aggregator = ImuAggregator()
    rospy.spin()

