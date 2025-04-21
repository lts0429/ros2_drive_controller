// differential_kinematics.hpp
#ifndef DIFFERENTIAL_KINEMATICS_HPP
#define DIFFERENTIAL_KINEMATICS_HPP

#include "geometry_msgs/msg/twist.hpp"

namespace differential_kinematics
{
    // Function that converts a Twist message to left and right wheel speeds
    // based on differential kinematics
    void calculate_wheel_speeds(const geometry_msgs::msg::Twist::SharedPtr& twist,
                                 double wheelbase, double& left_speed, double& right_speed)
    {
        // Extract linear and angular velocities from the Twist message
        double v_linear = twist->linear.x;  // linear velocity along the x-axis
        double omega_angular = twist->angular.z;  // angular velocity around the z-axis
        
        // Differential drive kinematics equations
        left_speed = v_linear - (wheelbase / 2.0) * omega_angular;
        right_speed = v_linear + (wheelbase / 2.0) * omega_angular;
    }
}  // namespace differential_kinematics

#endif  // DIFFERENTIAL_KINEMATICS_HPP
