#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"

#define WHEELRADIUS 0.07

ros::Publisher pub;


void printToScreen(const sensor_msgs::JointState::ConstPtr& msg)
{
  const sensor_msgs::JointState* write_joint = msg.get();
  //ROS_INFO("Front Left speed is %f", write_joint->velocity[0]);
  geometry_msgs::TwistStamped final;
  final.header.seq = write_joint->header.seq;
  final.header.stamp = write_joint->header.stamp;
  final.header.frame_id = write_joint->header.frame_id;

  double fl = write_joint->velocity[0];
  double fr = write_joint->velocity[1];
  double rl = write_joint->velocity[2];
  double rr = write_joint->velocity[3];


  final.twist.linear.x = (fl + fr + rl + rr) * (WHEELRADIUS / 4);
  final.twist.linear.y = (-fl + fr + rl - rr) * (WHEELRADIUS / 4);
  final.twist.linear.z = 0;

  final.twist.angular.x = 0;
  final.twist.angular.y = 0;
  final.twist.angular.z = (-fl + fr - rl + rr) * (WHEELRADIUS / 4);

  pub.publish(final);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "velocity_comp");
  ros::NodeHandle n;
  pub = n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);
  ros::Subscriber sub = n.subscribe("/wheel_states", 1000, printToScreen);
  ros::spin();

  return 0;
}
