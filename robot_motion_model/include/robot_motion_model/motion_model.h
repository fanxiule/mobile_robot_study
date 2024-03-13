#pragma once

#include <angles/angles.h>

#include <array>
#include <cmath>

namespace robot_motion_model
{
    /* @brief Calculate a robot state with a simple forward kinematic model
     *
     * @param[in,out] state State of the robot including x, y, and theta
     * @param[in] vel Velocity of the robot with both linear and rotational components
     * @param[in] period Time delta of the control command
     */
    inline void forwardKinematicModel(std::array<double, 3> &state, const std::array<double, 2> &vel, const double period)
    {
        state[0] = state[0] + vel[0] * std::cos(state[2]) * period;
        state[1] = state[1] + vel[0] * std::sin(state[2]) * period;
        state[2] = state[2] + vel[1] * period;

        state[2] = angles::normalize_angle(state[2]);
    }

    /* @brief Calculate a robot state with a deterministic velocity motion model
     * The motion model is based on Probabilistic Robotics by Thrun et al.
     *
     * @param[in,out] state State of the robot including x, y, and theta
     * @param[in] vel Velocity of the robot with both linear and rotational components
     * @param[in] period Time delta of the control command
     */
    inline void velocityMotionModel(std::array<double, 3> &state, const std::array<double, 2> &vel, const double period)
    {
        state[0] = state[0] - vel[0] / vel[1] * std::sin(state[2]) + vel[0] / vel[1] * std::sin(state[2] + vel[1] * period);
        state[1] = state[1] + vel[0] / vel[1] * std::cos(state[2]) - vel[0] / vel[1] * std::cos(state[2] + vel[1] * period);
        state[2] = state[2] + vel[1] * period;

        state[2] = angles::normalize_angle(state[2]);
    }
};