#include "PoseResetter.h"
#include "ros/ros.h"
#include "project1/ResetOdometry.h"
#include "geometry_msgs/Quaternion.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


PoseResetter::PoseResetter() {
    client = n.serviceClient<project1::ResetOdometry>("reset");
    sub = n.subscribe("robot/pose", 1000, &PoseResetter::initialPositionCallback, this);
    initializationDone = false;
}

void PoseResetter::initialPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!initializationDone) {
        project1::ResetOdometry resetSrv;
        resetSrv.request.new_x = msg->pose.position.x;
        resetSrv.request.new_y = msg->pose.position.y;

        tf2::Quaternion q;
        geometry_msgs::Quaternion q_msg = msg->pose.orientation;
        tf2::fromMsg(q_msg, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        resetSrv.request.new_theta = yaw;

        initializationDone = client.call(srv);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "reset");
  
    PoseResetter poseResetter;

    ros::spin();

    return 0;
}