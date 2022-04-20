#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H


#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/TransformStamped.h"

enum class IntegrationMethod {
    EULER, RUNGE_KUTTA
};

class OdometryPublisher {
public:
  OdometryPublisher(); 
  void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void eulerOdometry();
  void rungeKuttaOdometry();
  void publishOdometry(const std_msgs::Header header);
  void broadcastTFOdometry(const std_msgs::Header header);

private:
  ros::NodeHandle n; 
  ros::Subscriber sub;
  ros::Publisher pub;
  tf2_ros::TransformBroadcaster transformBroadcaster;
  geometry_msgs::TransformStamped transformStamped;

  double x_k;
  double y_k;
  double theta_k;
  ros::Time t_k;
  ros::Time t_k_new; //time k+1
  double v_x_k;
  double v_y_k;
  double omega_k;
  IntegrationMethod integrationMethod;
};

#endif