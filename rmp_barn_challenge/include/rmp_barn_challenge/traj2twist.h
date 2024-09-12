#pragma once
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>


class Traj_2_Twist
{
    public:
        Traj_2_Twist(ros::NodeHandle nh);
    
    private:
        void convertTrajToTwist(const trajectory_msgs::MultiDOFJointTrajectory& traj);
        void odomCallback(const nav_msgs::Odometry& odom);
        bool readParams();
        double wrapAngle(double angle);
        double current_yaw_;
        ros::Subscriber traj_sub_;
        ros::Subscriber odom_sub_;
        ros::Publisher cmd_pub_;
        ros::NodeHandle nh_;
        std::string traj_topic_;
        std::string twist_topic_;
        std::string odom_topic_;
};