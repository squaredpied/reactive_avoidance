#include <ros/ros.h>
#include <Eigen/Eigen>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

class SubgoalPlanner
{
    public:
    SubgoalPlanner(ros::NodeHandle nh);
    void laserScanCallback(const sensor_msgs::LaserScan& scan);
    void goalCallback(const geometry_msgs::PointStamped& goal);

    private:
    bool readParameters();
    ros::NodeHandle nh_;
    ros::Subscriber laser_scan_;
    ros::Subscriber goal_;
    ros::Publisher sub_goal_;
    Eigen::Vector3d goal_W_;
    std::string goal_frame_;
    bool goal_received_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    int goal_bearing_range_; // used because of discretization errors
    double threshold_; // threshold to check for candidate valleys
    double s_max_;
    double w_s_; // window size
    double alpha_; // angular resolution of the histogram
    double a_; // parameter for polar histogram building
    double b_; // parameter for polar histogram building
    double t_high_;
    double t_low_;
    Eigen::MatrixXf polar_histogram_;

};