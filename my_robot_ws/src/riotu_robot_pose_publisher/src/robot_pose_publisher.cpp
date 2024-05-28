#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_pose_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  std::string map_frame, base_frame;
  double publish_frequency;
  bool is_stamped;
  ros::Publisher pose_pub;

  nh_priv.param<std::string>("map_frame", map_frame, "/map");
  nh_priv.param<std::string>("base_frame", base_frame, "/chassis");
  nh_priv.param<double>("publish_frequency", publish_frequency, 10.0);
  nh_priv.param<bool>("is_stamped", is_stamped, false);

  if (is_stamped) {
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("robot_pose", 1);
  } else {
    pose_pub = nh.advertise<geometry_msgs::Pose>("robot_pose", 1);
  }

  tf::TransformListener listener;

  ros::Rate rate(publish_frequency);
  while (ros::ok()) {
    tf::StampedTransform transform;
    try {
      // Wait for up to 1 second for the transform to become available
      if (listener.waitForTransform(map_frame, base_frame, ros::Time(0), ros::Duration(1.0))) {
        listener.lookupTransform(map_frame, base_frame, ros::Time(0), transform);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = map_frame;
        pose_stamped.header.stamp = ros::Time::now();

        pose_stamped.pose.orientation.x = transform.getRotation().getX();
        pose_stamped.pose.orientation.y = transform.getRotation().getY();
        pose_stamped.pose.orientation.z = transform.getRotation().getZ();
        pose_stamped.pose.orientation.w = transform.getRotation().getW();

        pose_stamped.pose.position.x = transform.getOrigin().getX();
        pose_stamped.pose.position.y = transform.getOrigin().getY();
        pose_stamped.pose.position.z = transform.getOrigin().getZ();

        if (is_stamped) {
          pose_pub.publish(pose_stamped);
        } else {
          pose_pub.publish(pose_stamped.pose);
        }
      } else {
        ROS_WARN("Waiting for transform between %s and %s", map_frame.c_str(), base_frame.c_str());
      }
    } catch (tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep(); // Sleep to avoid spamming with errors
      continue;
    }
    rate.sleep();
  }

  return EXIT_SUCCESS;
}

