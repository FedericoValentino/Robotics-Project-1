#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "project1/WheelSpeed.h"

#define R 0.064
#define L 0.2
#define W 0.169
#define CONV_FACTOR 9.549297

ros::Publisher pub;

void publishSpeed(const geometry_msgs::TwistStamped msg){
  /*ROS_INFO("data subscribed (%f %f %f), (%f %f %f)",
  msg.twist.linear.x,msg.twist.linear.y,msg.twist.linear.z,
  msg.twist.angular.x,msg.twist.angular.y,msg.twist.angular.z);*/
  project1::WheelSpeed message;
  message.Header=msg.header;
  double vx=msg.twist.linear.x;
  double vy=msg.twist.linear.y;
  double omega=msg.twist.angular.z;

  double vl1=CONV_FACTOR*(vx-vy-(L+W)*omega)/R;
  double vr1=CONV_FACTOR*(vx+vy+(L+W)*omega)/R;
  double vr2=CONV_FACTOR*(vx-vy+(L+W)*omega)/R;
  double vl2=CONV_FACTOR*(vx+vy-(L+W)*omega)/R;

  message.rpm_fl=vl1;
  message.rpm_fr=vr1;
  message.rpm_rl=vl2;
  message.rpm_rr=vr2;

  pub.publish(message);

}

int main(int argc, char**argv){
  ros :: init(argc,argv,"wheelSpeed_publisher");
  ros :: NodeHandle node;
  pub=node.advertise<project1::WheelSpeed>("wheels_rpm",1000);
  ros :: Subscriber sub=node.subscribe("/cmd_vel",1000,publishSpeed);
  ros :: spin();
}
