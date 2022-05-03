#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"
#include "VelocityPublisher.h"
#include "project1/WheelSpeed.h"

#define WHEELRADIUS 0.064
#define ENCODER_RES 39.8475
#define RATIO 5.0
#define L 0.2
#define W 0.169
#define PI 3.14
#define CONV_FACTOR 9.549297

//FILE* output = fopen("/home/feder/Desktop/Programmazione/robotics/src/project1/src/outputOriginal.txt", "w");
//FILE* output2 = fopen("/home/feder/Desktop/Programmazione/robotics/src/project1/src/outputObtained.txt", "w");


VelocityPublisher::VelocityPublisher()
{
  sub = this->n.subscribe("/wheel_states", 1000, &VelocityPublisher::wheelsCallback, this);
  pub = this->n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);
  pubTest = this->n.advertise<project1::WheelSpeed>("speedsTest", 1000);
  velocity.x = 0;
  velocity.y = 0;
  velocity.z = 0;
  omega.x = 0;
  omega.y = 0;
  omega.z = 0;
  totalMessage = 0;
}

void VelocityPublisher::computeVelocities()
{
  double fl = ((msg2Data.position[0] - msg1Data.position[0])/(timeDiff(msg2Data.header.stamp, msg1Data.header.stamp))) * (1/ENCODER_RES) * (1/RATIO) * 2*PI;
  double fr = ((msg2Data.position[1] - msg1Data.position[1])/(timeDiff(msg2Data.header.stamp, msg1Data.header.stamp))) * (1/ENCODER_RES) * (1/RATIO) * 2*PI;
  double rl = ((msg2Data.position[2] - msg1Data.position[2])/(timeDiff(msg2Data.header.stamp, msg1Data.header.stamp))) * (1/ENCODER_RES) * (1/RATIO) * 2*PI;
  double rr = ((msg2Data.position[3] - msg1Data.position[3])/(timeDiff(msg2Data.header.stamp, msg1Data.header.stamp))) * (1/ENCODER_RES) * (1/RATIO) * 2*PI;
  //fprintf(output2, "%f\n%f\n%f\n%f\n", fl * RATIO/CONV_FACTOR, fr * RATIO/CONV_FACTOR, rl * RATIO/CONV_FACTOR, rr * RATIO/CONV_FACTOR);

  project1::WheelSpeed message;
  message.rpm_fl = fl * CONV_FACTOR;
  message.rpm_fr = fr * CONV_FACTOR;
  message.rpm_rl = rl * CONV_FACTOR;
  message.rpm_rr = rr * CONV_FACTOR;
  pubTest.publish(message);

  velocity.x = (fl + fr + rl + rr) * (WHEELRADIUS / 4);
  velocity.y = (-fl + fr + rl - rr) * (WHEELRADIUS / 4);
  velocity.z = 0;

  omega.x = 0;
  omega.y = 0;
  omega.z = (-fl + fr - rl + rr) * (WHEELRADIUS / 4/(L+W));
  publishVelocities();
}


double VelocityPublisher::timeDiff(const ros::Time t1, const ros::Time t2)
{
  ros::Duration difference = t1 - t2;
  double diff = difference.toSec();
  return diff;
}

void VelocityPublisher::wheelsCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  totalMessage++;
  if(totalMessage == 1)
  {
    msg1Data = *msg;
  }
  else if(totalMessage == 2)
  {
    msg2Data = *msg;
  }
  else
  {
    msg1Data = msg2Data;
    msg2Data = *msg;
  }

  if(totalMessage >= 2)
  {
    //fprintf(output, "%f\n%f\n%f\n%f\n", msg.get()->velocity[0]/60, msg.get()->velocity[1]/60, msg.get()->velocity[2]/60, msg.get()->velocity[3]/60);
    computeVelocities();
  }
}

void VelocityPublisher::publishVelocities()
{
  geometry_msgs::TwistStamped final;
  final.header.frame_id = msg2Data.header.frame_id;
  final.header.seq = msg2Data.header.seq;
  final.header.stamp = msg2Data.header.stamp;


  final.twist.linear.x = velocity.x;
  final.twist.linear.y = velocity.y;
  final.twist.linear.z = velocity.z;

  final.twist.angular.x = omega.x;
  final.twist.angular.y = omega.y;
  final.twist.angular.z = omega.z;

  pub.publish(final);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "velocity_comp");
  VelocityPublisher node;
  ros::spin();

  return 0;
}
