#ifndef LASER_SCAN_TO_POINT_CLOUD_H
#define LASER_SCAN_TO_POINT_CLOUD_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <laser_geometry/laser_geometry.h>


class LaserScanToPointCloud
{
public:
    LaserScanToPointCloud(ros::NodeHandle& nh);

private:
    bool readParameters();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

    ros::NodeHandle nh_;
    ros::Publisher point_cloud_pub_;
    std::string laser_scan_topic;
    std::string point_cloud_topic;
    std::string target_frame_id;
    bool change_frame;
    laser_geometry::LaserProjection projector_;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> scan_sub_;
    std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan>> tf_filter_;
};

#endif // LASER_SCAN_TO_POINT_CLOUD_H
