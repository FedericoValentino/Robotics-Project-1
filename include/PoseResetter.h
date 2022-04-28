#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H

#include "ros/ros.h"
#include "project1/ResetOdometry.h"
#include "geometry_msgs/PoseStamped.h"

class PoseResetter {
    public:
        PoseResetter();
        void initialPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    private:
        ros::NodeHandle n;
        ros::ServiceClient client;
        project1::ResetOdometry srv;
        ros::Subscriber sub;

        bool initializationDone;
};

#endif