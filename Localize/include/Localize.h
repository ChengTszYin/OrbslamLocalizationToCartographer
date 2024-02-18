#ifndef LOCALIZE_H
#define LOCALIZE_H
#include <ros/ros.h>
#include "std_srvs/Trigger.h"
#include "std_srvs/Empty.h"
//#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <cartographer_ros_msgs/FinishTrajectory.h>
#include <cartographer_ros_msgs/StartTrajectory.h>

class Localize {
public:
    Localize();
    void angleCallback_(const std_msgs::Int16::ConstPtr& msgs);
    void featCallback_(const std_msgs::Int16::ConstPtr& msgs);
    bool callback_(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res);
    std_srvs::Trigger srv;
    void StartLocalize();
    void LocalMethodSelect(const std::string method);
    void cartoInit_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

private:
    ros::NodeHandle nh;
    ros::Publisher vel_pub;
    ros::ServiceClient orb_client_;
    ros::Subscriber pose_sub_;
    ros::Subscriber feature_sub_;
    ros::Subscriber euler_sub_;
    ros::ServiceServer service;
    ros::ServiceClient finishTrajectoryClient;
    ros::ServiceClient StartTrajectoryCliebt;
    std::string local_method;
    float xcoordinate = 0.0;
    float ycoordinate = 0.0;
    float xorient = 0.0;
    float yorient = 0.0;
    float zorient = 0.0;
    float worient = 0.0;
    bool pose_received = false;
    bool taskFinish = true;
    int nfeature;
    int angle;
    int startAngle = 0;
    int poseID = 0;
};

#endif