#pragma once

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <array>
#include <string>

class KinematicModel
{
public:
    KinematicModel();
    ~KinematicModel() = default;

private:
    /* @brief Receives control command, calculates and publihes path
     *
     * @param[in] control_msg Control command
     */
    void controllCallback(const geometry_msgs::Twist::ConstPtr &control_msg);

    void recordVelocity(const geometry_msgs::Twist::ConstPtr &control_msg);

    ros::NodeHandle nh_;

    // Subscriber and publisher
    ros::Subscriber control_sub_;
    ros::Publisher path_pub_;

    // Stores the path computed by the kinematic model
    nav_msgs::Path path_msg_;

    // Frame of the path msg
    std::string path_frame_{"/world"};

    // Robot state (x, y, theta) and velocity (linear, angular)
    std::array<double, 3> state_{0.0, 0.0, 0.0};
    std::array<double, 2> vel_{0.0, 0.0};

    // Time and sequence info
    int seq_{0};
    ros::Time prev_control_time_;

    // Flag to indicate if the a control signal has been received
    bool control_received_{false};

    // Motio model
    std::string motion_model_{"forward_kinematics"};
    void (*motion_model_func_)(std::array<double, 3> &, const std::array<double, 2> &, const double);
};