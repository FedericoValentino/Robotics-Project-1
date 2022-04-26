#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3.h"


class VelocityPublisher
{
public:
  VelocityPublisher();
  void wheelsCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void computeVelocities();
  void publishVelocities();
  double timeDiff(const ros::Time t1, const ros::Time t2);
private:
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Publisher pub;
  ros::Publisher pub2;

  sensor_msgs::JointState msg1Data;
  sensor_msgs::JointState msg2Data;
  geometry_msgs::Vector3 velocity;
  geometry_msgs::Vector3 omega;
  int totalMessage;
};
