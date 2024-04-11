#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <limits>
#include <memory>

namespace robot_sensor_model
{
class ScanToPointCloudConverter
{
public:
    ScanToPointCloudConverter();
    ~ScanToPointCloudConverter() = default;

private:
    /**
     * @brief Callback for laser scan messages
     *
     * @param[in] scan_msg Scan message
     */
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);

    /**
     * @brief Project 2D scan points given in range and angle to 3D coordinate
     *
     * @param[in] angles All angle values of the 2D scan
     * @param[in] ranges All range values of the 2D scan
     */
    void projectPoints(const std::vector<float> &angles, const std::vector<float> &ranges);

    ros::NodeHandle nh_;

    // Subscriber and publisher
    ros::Publisher cloud_pub_;
    ros::Subscriber scan_sub_;

    // Stores point cloud converted from laser scans
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

    // Range limits
    float range_min_{std::numeric_limits<float>::lowest()};
    float range_max_{std::numeric_limits<float>::max()};
};

};