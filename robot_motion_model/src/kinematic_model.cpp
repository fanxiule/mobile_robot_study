#include <robot_motion_model/kinematic_model.h>
#include <robot_motion_model/motion_model.h>

#include <angles/angles.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

KinematicModel::KinematicModel()
{
    ros::NodeHandle pnh;

    // get initial robot state and velocity
    double x_init{0.0};
    double y_init{0.0};
    double theta_init{0.0};
    double lin_vel_init{0.0};
    double rot_vel_init(0.0);

    pnh.param("x_init", x_init, x_init);
    pnh.param("y_init", y_init, y_init);
    pnh.param("theta_init", theta_init, theta_init);
    pnh.param("lin_vel_init", lin_vel_init, lin_vel_init);
    pnh.param("rot_vel_init", rot_vel_init, rot_vel_init);

    state_[0] = x_init;
    state_[1] = y_init;
    state_[2] = theta_init;

    vel_[0] = lin_vel_init;
    vel_[1] = rot_vel_init;

    ROS_INFO_STREAM("Initial state: x = " << state_[0] << " m, y = " << state_[1] << " m, theta = " << angles::to_degrees(state_[2]) << " deg.");
    ROS_INFO_STREAM("Initial velocity: lin_vel = " << vel_[0] << " m/s, ang_vel = " << angles::to_degrees(vel_[1]) << " deg/s.");

    // set up frame of the path
    pnh.param("path_frame", path_frame_, path_frame_);
    path_msg_.header.frame_id = path_frame_;

    // choose the appropriate motion model
    pnh.param("motion_model", motion_model_, motion_model_);

    if (motion_model_ == "forward_kinematics")
    {
        motion_model_func_ = robot_motion_model::forwardKinematicModel;
    }
    else if (motion_model_ == "velocity_model")
    {
        motion_model_func_ = robot_motion_model::velocityMotionModel;
    }
    else
    {
        ROS_FATAL_STREAM("Unsupported motion model: " << motion_model_ << ". Exiting...");
        ros::shutdown();
    }

    // set up publisher and subscriber
    path_pub_ = nh_.advertise<nav_msgs::Path>("kinematic_model_path", 1, true);
    control_sub_ = nh_.subscribe("/cmd_vel", 1, &KinematicModel::controllCallback, this);
}

void KinematicModel::controllCallback(const geometry_msgs::Twist::ConstPtr &control_msg)
{
    const auto current_time = ros::Time::now();

    // handle the first time when a msg is received
    if (!control_received_)
    {
        prev_control_time_ = current_time;
        recordVelocity(control_msg);

        control_received_ = true;
        return;
    }

    // calculate new state
    const auto period = (current_time - prev_control_time_).toSec();
    motion_model_func_(state_, vel_, period);

    ROS_INFO_STREAM_THROTTLE(10.0, "Current state: x = " << state_[0] << " m, y = " << state_[1] << " m, theta = " << angles::to_degrees(state_[2]) << " deg.");
    ROS_INFO_STREAM_THROTTLE(10.0, "Current velocity: lin_vel = " << vel_[0] << " m/s, ang_vel = " << angles::to_degrees(vel_[1]) << " deg/s.");

    // update path
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = path_frame_;
    pose_msg.header.seq = seq_;
    pose_msg.header.stamp = current_time;

    pose_msg.pose.position.x = state_[0];
    pose_msg.pose.position.y = state_[1];
    pose_msg.pose.position.z = 0.0;

    tf2::Quaternion orientation;
    orientation.setEuler(0.0, 0.0, state_[2]);
    pose_msg.pose.orientation.x = orientation.getX();
    pose_msg.pose.orientation.y = orientation.getY();
    pose_msg.pose.orientation.z = orientation.getZ();
    pose_msg.pose.orientation.w = orientation.getW();

    path_msg_.header.seq = seq_;
    path_msg_.header.stamp = current_time;
    path_msg_.poses.push_back(pose_msg);

    // publish path
    path_pub_.publish(path_msg_);

    // update variables for next iteration
    seq_++;
    prev_control_time_ = current_time;
    recordVelocity(control_msg);
}

void KinematicModel::recordVelocity(const geometry_msgs::Twist::ConstPtr &control_msg)
{
    vel_[0] = control_msg->linear.x;
    vel_[1] = control_msg->angular.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinematic_model");
    KinematicModel kinematic_model;
    ros::spin();
    return 0;
}