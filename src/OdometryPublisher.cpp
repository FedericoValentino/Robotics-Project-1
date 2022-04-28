#include "OdometryPublisher.h"
#include "math.h"
#include "project1/ResetOdometry.h"

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
    integrationMethod = IntegrationMethod::RUNGE_KUTTA;

    service = n.advertiseService("reset", &OdometryPublisher::resetOdometryCallback, this);

    f = boost::bind(&OdometryPublisher::integrationMethodReconfigureCallback, this, _1, _2);
    dynServer.setCallback(f);

    isInitialized = false;
}

void OdometryPublisher::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    if(!isInitialized) return;
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
    }

    broadcastTFOdometry(msg->header);
    publishOdometry(msg->header);
}

bool OdometryPublisher::resetOdometryCallback(project1::ResetOdometry::Request &req, project1::ResetOdometry::Response &res) {
    res.old_x = x_k;
    res.old_y = y_k;
    res.old_theta = theta_k;
    res.old_time_seconds = t_k.toSec();
    
    x_k = req.new_x;
    y_k = req.new_y;
    theta_k = req.new_theta;
    t_k_new = ros::Time(req.new_time_seconds);

    isInitialized = true;
    //ROS_INFO("Reset at pose (%f,%f,%f) and time %f sec", x_k, y_k, theta_k, req.new_time_seconds);

    return true;
}

void OdometryPublisher::integrationMethodReconfigureCallback(project1::integrationParameterConfig &config, uint32_t level) {
    switch (config.integrationMethod) {
    case 0:
        integrationMethod = IntegrationMethod::EULER;
        break;
    case 1:
        integrationMethod = IntegrationMethod::RUNGE_KUTTA;
        break;
    } 
}

void OdometryPublisher::eulerOdometry() {
    double Ts = (t_k_new - t_k).toSec();

    //v_x_k and v_y_k are in the robot frame, so the velocity orientation has to be adjusted by theta_k to match global frame
    double v_x = v_x_k * cos(theta_k) - v_y_k * sin(theta_k);
    double v_y = v_x_k * sin(theta_k) + v_y_k * cos(theta_k);

    x_k += v_x * Ts;
    y_k += v_y * Ts;
    theta_k += omega_k * Ts;
}

void OdometryPublisher::rungeKuttaOdometry() {
    double Ts = (t_k_new - t_k).toSec();

    //velocity module
    double velocity = sqrt(pow(v_x_k, 2.0) + pow(v_y_k, 2.0));

    double v_x = v_x_k * cos(theta_k) - v_y_k * sin(theta_k);
    double v_y = v_x_k * sin(theta_k) + v_y_k * cos(theta_k);

    //velocity orientation(at time k)
    double vel_angle = atan(v_y / v_x);

    //velocity mean orientation from time k to time k+1
    double vel_mean_angle = vel_angle + ((omega_k * Ts) / 2.0);

    //Runge-Kutta
    x_k = x_k + velocity * Ts * cos(vel_mean_angle);
    y_k = y_k + velocity * Ts * sin(vel_mean_angle);
    theta_k = theta_k + omega_k * Ts;
}

void OdometryPublisher::publishOdometry(const std_msgs::Header header) {
    nav_msgs::Odometry msg;

    msg.header.frame_id = "odom";
    msg.header.seq = header.seq;
    msg.header.stamp = header.stamp;
    msg.child_frame_id = "base_link";

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
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "base_link";

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
    ros::init(argc, argv, "odometry_publisher");
  
    OdometryPublisher odometryPublisher;

    ros::spin();

    return 0;
}