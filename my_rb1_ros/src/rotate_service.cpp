#include <cmath>
#include <geometry_msgs/Twist.h>
#include <my_rb1_ros/Rotate.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

ros::Publisher vel_pub;
double current_yaw = 0;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  double s = msg->pose.pose.orientation.w;
  double x = msg->pose.pose.orientation.x;
  double y = msg->pose.pose.orientation.y;
  double z = msg->pose.pose.orientation.z;

  // Calculate yaw angle from the quaternion
  current_yaw = atan2(2.0 * (y * x + s * z), s * s + x * x - y * y - z * z);
}

bool rotateCallback(my_rb1_ros::Rotate::Request &req,
                    my_rb1_ros::Rotate::Response &res) {
  double target_yaw = current_yaw + (req.degrees * M_PI / 180.0);
  if (target_yaw > M_PI)
    target_yaw -= 2 * M_PI; // Normalize the angle

  const double kP = 0.5; // Proportional gain
  ros::Rate rate(10);
  while (ros::ok()) {
    // Calculate rotation angle error
    double error_yaw = target_yaw - current_yaw;

    // Stop if the target is reached
    if (fabs(error_yaw) < 0.01) {
      break;
    }

    geometry_msgs::Twist twist;
    twist.angular.z = kP * error_yaw; // Proportional controller
    vel_pub.publish(twist);

    ros::spinOnce();
    rate.sleep();
  }

  // Stop the robot
  geometry_msgs::Twist stop_twist;
  vel_pub.publish(stop_twist);

  res.result = "Rotation completed.";
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "rotate_service_server");
  ros::NodeHandle nh;

  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::Subscriber odom_sub = nh.subscribe("odom", 10, odomCallback);
  ros::ServiceServer service =
      nh.advertiseService("rotate_robot", rotateCallback);

  ROS_INFO("Rotate service server started.");

  ros::spin();

  return 0;
}
