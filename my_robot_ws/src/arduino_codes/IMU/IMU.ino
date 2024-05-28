#include <Wire.h>
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <MPU6050.h>


ros::NodeHandle nh;

MPU6050 imu;

geometry_msgs::Vector3 accel_msg;
geometry_msgs::Vector3 gyro_msg;
ros::Publisher pub_accel("imu/accel", &accel_msg);
ros::Publisher pub_gyro("imu/gyro", &gyro_msg);

void setup() {
  Wire.begin();

  nh.initNode();
  
  nh.advertise(pub_accel);
  nh.advertise(pub_gyro);
  
  imu.initialize();
  if (!imu.testConnection()) {
    while (1) {
      // Loop forever if connection failed
    }
  }
}

void loop() {
  // Read raw IMU data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert accelerometer data to m/s² and publish
  accel_msg.x = ax * 9.81 / 16384.0;
  accel_msg.y = ay * 9.81 / 16384.0;
  accel_msg.z = az * 9.81 / 16384.0;
  pub_accel.publish(&accel_msg);

  // Convert gyro data to rad/s and publish
  gyro_msg.x = gx * 0.001064225;
  gyro_msg.y = gy * 0.001064225;
  gyro_msg.z = gz * 0.001064225;
  pub_gyro.publish(&gyro_msg);

  nh.spinOnce();
  delay(10);
}

