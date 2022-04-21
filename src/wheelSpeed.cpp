#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"

ros::Publisher pub;

void printHere(const geometry_msgs::TwistStamped msg){
  /*ROS_INFO("data subscribed (%f %f %f), (%f %f %f)",
  msg.twist.linear.x,msg.twist.linear.y,msg.twist.linear.z,
  msg.twist.angular.x,msg.twist.angular.y,msg.twist.angular.z);*/

}

int main(int argc, char**argv){
  ros :: init(argc,argv,"vector_print");
  ros :: NodeHandle node;
  ros :: Subscriber sub=node.subscribe("/cmd_vel",1000,printHere);
  ros :: spin();
}
