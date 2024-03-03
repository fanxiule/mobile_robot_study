#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

class GroundTruthPathPublisher
{
public:
    GroundTruthPathPublisher()
    {
        path_pub_ = nh_.advertise<nav_msgs::Path>("/ground_truth_path", 1, true);
        odom_sub_ = nh_.subscribe("/odom", 1, &GroundTruthPathPublisher::odomCallback, this);
    }

    ~GroundTruthPathPublisher() = default;

private:
    ros::NodeHandle nh_;

    // Subscriber and publiher
    ros::Subscriber odom_sub_;
    ros::Publisher path_pub_;
    
    // Stores ground truth path
    nav_msgs::Path gt_path_msg_;

private:
    /* @brief Receives ground truth odometry and updates the ground truth path
     *
     * @param[in] odom_msg Odometry message
     */
    void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
    {
        geometry_msgs::PoseStamped stamped_pose_msg;
        stamped_pose_msg.header = odom_msg->header;
        stamped_pose_msg.pose = odom_msg->pose.pose;

        gt_path_msg_.header = odom_msg->header;
        gt_path_msg_.poses.push_back(stamped_pose_msg);
        
        path_pub_.publish(gt_path_msg_);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ground_truth_robot_path");
    GroundTruthPathPublisher path_pub;
    ros::spin();
    return 0;
}