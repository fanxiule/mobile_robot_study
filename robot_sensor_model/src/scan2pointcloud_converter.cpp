#include <robot_sensor_model/scan2pointcloud_converter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/shared_ptr.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace robot_sensor_model
{
ScanToPointCloudConverter::ScanToPointCloudConverter()
{
    cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("scan_cloud", 1, false);
    scan_sub_ = nh_.subscribe("/scan", 1, &ScanToPointCloudConverter::scanCallback, this);
}

void ScanToPointCloudConverter::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    // check if the scan dimension is consistent
    const auto angle_size = static_cast<int>(std::round((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment)) + 1;
    if (angle_size != scan_msg->ranges.size())
    {
        ROS_ERROR_STREAM("Mismatch data size with " << angle_size << " angles and " << scan_msg->ranges.size() << " ranges.");
        return;
    }

    // overwrite range limits
    range_min_ = scan_msg->range_min;
    range_max_ = scan_msg->range_max;

    // fill a vector of angles
    std::vector<float> angles(angle_size);
    std::iota(angles.begin(), angles.end(), 0); // fill with 0, 1, ..., angle_size - 1
    std::for_each(angles.begin(), angles.end(), [&](float &angle)
                  { angle *= scan_msg->angle_increment; });

    projectPoints(angles, scan_msg->ranges);

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_, cloud_msg);
    cloud_msg.header = scan_msg->header;
    cloud_pub_.publish(cloud_msg);
}

void ScanToPointCloudConverter::projectPoints(const std::vector<float> &angles, const std::vector<float> &ranges)
{
    // first clear the point cloud
    cloud_->clear();

    for (std::size_t i = 0; i < angles.size(); i++)
    {
        const auto angle = angles[i];
        const auto range = ranges[i];

        // out of range limits
        if (range > range_max_ || range < range_min_)
        {
            continue;
        }

        /*
         *    scan point *
         *              /|
         *             / |
         *            /  |
         *      range/   | y
         *          /    |
         *         /     |
         *        /angle |
         * robot * - - - -
         *           x
         */
        const auto x = range * std::cos(angle);
        const auto y = range * std::sin(angle);

        cloud_->push_back(pcl::PointXYZ(x, y, 0.1));
    }
}

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_to_pointcloud_converter");
    robot_sensor_model::ScanToPointCloudConverter converter;
    ros::spin();
    return 0;
}