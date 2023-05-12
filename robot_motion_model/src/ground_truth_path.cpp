#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

class GroundTruthPathPublisher
{
private:
    ros::NodeHandle nh;
    ros::Subscriber odom_sub;
    ros::Publisher path_pub;
    nav_msgs::Path gt_path_msgs;

public:
    GroundTruthPathPublisher()
    {
        // publish the path to the /ground_truth_path topic
        path_pub = nh.advertise<nav_msgs::Path>("/ground_truth_path", 1, true);
        // subscribe to the /odom topic to gather the robot's ground truth pose
        odom_sub = nh.subscribe("/odom", 1, &GroundTruthPathPublisher::odom_callback, this);
    }

private:
    void odom_callback(const nav_msgs::Odometry &odom_msgs)
    {
        geometry_msgs::PoseStamped stamped_pose_msgs;
        gt_path_msgs.header = odom_msgs.header;
        stamped_pose_msgs.header = odom_msgs.header;
        stamped_pose_msgs.pose = odom_msgs.pose.pose;
        gt_path_msgs.poses.push_back(stamped_pose_msgs);
        path_pub.publish(gt_path_msgs);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ground_truth_robot_path");

    GroundTruthPathPublisher path_pub;
    
    ros::Rate loop_rate(20);
    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}