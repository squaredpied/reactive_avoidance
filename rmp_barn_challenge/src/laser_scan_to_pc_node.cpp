#include <rmp_barn_challenge/laser_scan_to_pc_node.h>

LaserScanToPointCloud::LaserScanToPointCloud(ros::NodeHandle& nh)
    : nh_(nh), tfListener(tfBuffer)
{
    // Read parameters
    if (!readParameters())
    {
        ROS_WARN("Some/No parameter(s) file found. Using default values.");
        ros::requestShutdown();
    }

    if (change_frame && !nh_.getParam("target_frame_id", target_frame_id))
    {
        ROS_WARN("Target frame not found");
        ros::requestShutdown();
    }

    // Initialize subscribers using shared pointers
    scan_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::LaserScan>>(nh_, laser_scan_topic, 1);

    if (change_frame)
    {
        tf_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::LaserScan>>(*scan_sub_, tfBuffer, target_frame_id, 10, nh_);
        tf_filter_->registerCallback(boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    }
    else
    {
        scan_sub_->registerCallback(boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    }

    point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(point_cloud_topic, 1);
}

bool LaserScanToPointCloud::readParameters()
{
    nh_.param("laser_scan_topic", laser_scan_topic, std::string("/scan"));
    nh_.param("point_cloud_topic", point_cloud_topic, std::string("/cloud"));
    nh_.param("change_frame", change_frame, false);
    return true;
}

void LaserScanToPointCloud::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    sensor_msgs::PointCloud2 cloud;
    if (change_frame)
    {
        try
        {
            projector_.transformLaserScanToPointCloud(target_frame_id, *scan, cloud, tfBuffer);
            point_cloud_pub_.publish(cloud);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Transform failure: %s", ex.what());
        }
    }
    else
    {
        projector_.projectLaser(*scan, cloud);
        point_cloud_pub_.publish(cloud);
    }
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_scan_to_point_cloud");
    ros::NodeHandle nodeHandle("~");
    LaserScanToPointCloud converter (nodeHandle);

    ros::spin();
    return 0;
}
