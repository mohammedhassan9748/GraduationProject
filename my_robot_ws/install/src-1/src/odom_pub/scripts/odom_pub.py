import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point32, Quaternion, TransformStamped
import tf
import math

R = 0.034         # Wheel radius
L = 0.133         # Distance between wheels
N = 360.0         # Number of encoder ticks

Dc = 0.0
RtickOld = 0.0
RtickNew = 0.0
LtickOld = 0.0
LtickNew = 0.0
x = 0.0
y = 0.0
th = 0.0

# Callback function for encoder messages
def encoder_cb(msg):
    global RtickNew, LtickNew, RtickOld, LtickOld, x, y, th, Dc

    RtickNew = msg.x  # Right encoder ticks
    LtickNew = msg.y  # Left encoder ticks

    DR = 2 * math.pi * R * (RtickNew - RtickOld) / N  # Right wheel distance
    DL = 2 * math.pi * R * (LtickNew - LtickOld) / N  # Left wheel distance

    Dc = (DR + DL) / 2.0
    th += (DR - DL) / L
    x += Dc * math.cos(th)
    y += Dc * math.sin(th)

    RtickOld = RtickNew
    LtickOld = LtickNew

def main():
    rospy.init_node('odometry_publisher')
    odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
    tf_broadcaster = tf.TransformBroadcaster()
    rospy.Subscriber('/encoder', Point32, encoder_cb)

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    r = rospy.Rate(20.0)  # 20 Hz

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        dt = (current_time - last_time).to_sec()

        vx = Dc * math.cos(th) / dt
        vy = Dc * math.sin(th) / dt
        vth = (2 * math.pi * R * (RtickNew - RtickOld) / N - 2 * math.pi * R * (LtickNew - LtickOld) / N) / (L * dt)

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        # Transform for broadcasting
        tf_broadcaster.sendTransform(
            (x, y, 0.0),
            odom_quat,
            current_time,
            "chassis",
            "odom"
        )

        # Populate odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'chassis'

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*odom_quat)

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth

        odom_pub.publish(odom)

        last_time = current_time
        r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
