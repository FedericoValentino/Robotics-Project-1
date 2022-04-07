#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

void printToScreen(const sensor_msgs::JointState::ConstPtr& msg)
{
  const sensor_msgs::JointState* write_joint = msg.get();
  ROS_INFO("Front Left speed is %f", write_joint->velocity[0]);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "sub");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/wheel_states", 1000, printToScreen);
  ros::spin();

  return 0;
}
