#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/LinearMath/Quaternion.h>
#include <iostream>
#include <cmath>

class SimpleKinPathPublisher
{
private:
    ros::NodeHandle nh;
    ros::Subscriber control_sub;
    ros::Subscriber odom_sub; // for filling out header information
    ros::Publisher path_pub;
    nav_msgs::Path simple_kin_path_msgs;
    int seq_counter = 0;
    double period;
    double x = 0;
    double y = 0;
    double theta = 0;

public:
    SimpleKinPathPublisher(int freq_hz)
    {
        period = 1.0 / freq_hz; // convert frequency to the length of each period or sampling time
        simple_kin_path_msgs.header.frame_id = "/odom";
        path_pub = nh.advertise<nav_msgs::Path>("/simple_kinematic_path", 1, true);
        // subscribe to the /cmd_vel topic to access control actions
        control_sub = nh.subscribe("/cmd_vel", 1, &SimpleKinPathPublisher::control_callback, this);
        // odom_sub = nh.subscribe("/odom_vel", 1, &SimpleKinPathPublisher::odom_callback, this);
    }

private:
    void control_callback(const geometry_msgs::Twist &control_msgs)
    {
        double lin_vel = control_msgs.linear.x;
        double rot_vel = control_msgs.angular.z;
        std::cout << "Control received: " << lin_vel << " " << rot_vel << std::endl;
        // calculate new position based on the kinematic model
        x = x + lin_vel * cos(theta) * period;
        y = y + lin_vel * sin(theta) * period;
        theta = theta + rot_vel * period;

        // ignore header information for now
        geometry_msgs::PoseStamped stamped_pose_msgs;
        stamped_pose_msgs.header.frame_id = "/odom";
        stamped_pose_msgs.header.seq = seq_counter;
        seq_counter++;

        stamped_pose_msgs.pose.position.x = x;
        stamped_pose_msgs.pose.position.y = y;
        stamped_pose_msgs.pose.position.z = 0;

        tf::Quaternion orient(0, 0, theta);
        stamped_pose_msgs.pose.orientation.x = orient.getX();
        stamped_pose_msgs.pose.orientation.y = orient.getY();
        stamped_pose_msgs.pose.orientation.z = orient.getZ();
        stamped_pose_msgs.pose.orientation.w = orient.getW();

        simple_kin_path_msgs.poses.push_back(stamped_pose_msgs);
        path_pub.publish(simple_kin_path_msgs);
    }

    /*
    void odom_callback(const nav_msgs::Odometry &odom_msgs)
    {
        std::cout << "Odom received" << std::endl;
        simple_kin_path_msgs.header = odom_msgs.header;
        geometry_msgs::PoseStamped stamped_pose_msgs;
        stamped_pose_msgs.header = odom_msgs.header;
        stamped_pose_msgs.pose.position.x = x;
        stamped_pose_msgs.pose.position.y = y;
        stamped_pose_msgs.pose.position.z = 0;

        tf::Quaternion orient(0, 0, theta);
        stamped_pose_msgs.pose.orientation.x = orient.getX();
        stamped_pose_msgs.pose.orientation.y = orient.getY();
        stamped_pose_msgs.pose.orientation.z = orient.getZ();
        stamped_pose_msgs.pose.orientation.w = orient.getW();

        simple_kin_path_msgs.poses.push_back(stamped_pose_msgs);
        path_pub.publish(simple_kin_path_msgs);
    }
    */
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_kinematic_motion_model");

    int freq_hz = 9; // frequency of message publication, max_freq of /cmd_vel is only close to 10 Hz
    SimpleKinPathPublisher kin_path_pub(freq_hz);

    ros::Rate loop_rate(freq_hz);
    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }
}