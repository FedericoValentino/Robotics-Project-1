#include "OdometryPublisher.h"
#include "math.h"

#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "tf2/LinearMath/Quaternion.h"

OdometryPublisher::OdometryPublisher() {
    sub = this->n.subscribe("cmd_vel", 1000, &OdometryPublisher::velocityCallback, this);
    pub = this->n.advertise<nav_msgs::Odometry>("odom", 1000);
    integrationMethod = IntegrationMethod::EULER;;
    //For these i need the inizializer
    x_k = 0.0;
    y_k = 0.0;
    theta_k = 0.0;
    t_k = ros::Time::now();
    t_k_new = ros::Time::now();
}

void OdometryPublisher::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    v_x_k = msg->twist.linear.x;
    v_y_k = msg->twist.linear.y;
    omega_k = msg->twist.angular.z;
    t_k = t_k_new;
    t_k_new = msg->header.stamp;

    switch (integrationMethod) {
        case IntegrationMethod::EULER :
            eulerOdometry();
            break;

        case IntegrationMethod::RUNGE_KUTTA :
            rungeKuttaOdometry();
            break;

        default:
            break;
    }

    publishOdometry(msg->header);
}

void OdometryPublisher::eulerOdometry() {
    double Ts = (t_k_new - t_k).toSec();

    //For euler integration, it's ok to use v_x and v_y (it don't use different angle from the initial velocity orientation)
    x_k = x_k + v_x_k * Ts;
    y_k = y_k + v_y_k * Ts;
    theta_k = theta_k + omega_k * Ts;
}

void OdometryPublisher::rungeKuttaOdometry() {
    double Ts = (t_k_new - t_k).toSec();

    //velocity module
    double velocity = sqrt(pow(v_x_k, 2.0) + pow(v_y_k, 2.0));

    //velocity orientation(at time k)
    double vel_angle = atan(v_y_k / v_x_k);

    //velocity mean orientation from time k to time k+1
    double vel_mean_angle = vel_angle + ((omega_k * Ts) / 2.0);

    //Runge-Kutta
    x_k = x_k + velocity * Ts * cos(vel_mean_angle);
    y_k = y_k + velocity * Ts * sin(vel_mean_angle);
    theta_k = theta_k + omega_k * Ts;
}

void OdometryPublisher::publishOdometry(const std_msgs::Header header) {
    nav_msgs::Odometry msg;

    msg.header.frame_id = header.frame_id;
    msg.header.seq = header.seq;
    msg.header.stamp = header.stamp;

    msg.pose.pose.position.x = x_k;
    msg.pose.pose.position.y = y_k;
    msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_k);
    msg.pose.pose.orientation.x = q.getX();
    msg.pose.pose.orientation.y = q.getY();
    msg.pose.pose.orientation.z = q.getZ();
    msg.pose.pose.orientation.w = q.getW();

    ROS_INFO("Robot pose : (%f, %f, %f)", msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z);

    pub.publish(msg);
}

void OdometryPublisher::broadcastTFOdometry(const std_msgs::Header header) {

    transformStamped.header.seq = header.seq;
    transformStamped.header.stamp = header.stamp;
    transformStamped.header.frame_id = "world"; //don't know what to put here
    transformStamped.child_frame_id = "base_link"; //don't know what to put here

    transformStamped.transform.translation.x = x_k;
    transformStamped.transform.translation.y = y_k;
    transformStamped.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_k);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    transformBroadcaster.sendTransform(transformStamped);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "callbacks_sub");
  
    OdometryPublisher odometryPublisher; // constructor is called

    ros::spin();

    return 0;
}