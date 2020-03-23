#include <ros/ros.h>
// #include <librealsense2/rs.hpp>
#include <mutex>
// #include "example.hpp"          // Include short list of convenience functions for rendering
#include <cstring>
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include <cmath>


ros::Subscriber gyro_subscriber;
ros::Subscriber accel_subscriber;

// geometry_msgs::Twist twist_msg;

geometry_msgs::Vector3 gyro_msg;
geometry_msgs::Vector3 accel_msg;

double new_time,last_time, time_diff = 0;

void gyroCallback(const sensor_msgs::Imu & gyro_message);
void accelCallback(const sensor_msgs::Imu & accel_message);
void complementary_filter();

int main( int argc, char** argv )
{
  ros::init(argc, argv, "gryo");
  ros::NodeHandle n;
  // new_time = ros::time_now()
  // gyro_subscriber = n.subscribe("/camera/gyro/sample", 1000, gyroCallback);
  // accel_subscriber = n.subscribe("/camera/accel/sample", 1000, accelCallback);
  complementary_filter();

  ros::spin();
  return 0;
}

void gyroCallback(const sensor_msgs::Imu & gyro_message){
	gyro_msg.x = gyro_message.angular_velocity.x;
	gyro_msg.y = gyro_message.angular_velocity.y;
	gyro_msg.z = gyro_message.angular_velocity.z;
  ROS_INFO("%f", gyro_msg.x);
}

void accelCallback(const sensor_msgs::Imu & accel_message){
	accel_msg.x = accel_message.linear_acceleration.x;
	accel_msg.y = accel_message.linear_acceleration.y;
	accel_msg.z = accel_message.linear_acceleration.z;
  ROS_INFO("%f", accel_msg.x);
}

void complementary_filter(){
  ros::NodeHandle n;
  new_time = ros::Time::now().toSec();
  gyro_subscriber = n.subscribe("/camera/gyro/sample", 1000, gyroCallback);
  accel_subscriber = n.subscribe("/camera/accel/sample", 1000, accelCallback);
  time_diff = last_time - new_time;
  double pitch, roll, acc_pitch, acc_roll = 0;
  acc_pitch = (atan2(accel_msg.y,accel_msg.z))*180.0/M_PI;
  acc_roll = (atan2(accel_msg.x,accel_msg.z))*180.0/M_PI;
  pitch = 0.98 * (pitch + gyro_msg.x * time_diff) + 0.02 * acc_pitch;
  roll = 0.98 * (roll + gyro_msg.y * time_diff) + 0.02 * acc_roll;
  last_time = new_time;
}
