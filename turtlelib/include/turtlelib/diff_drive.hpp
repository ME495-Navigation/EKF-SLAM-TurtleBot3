#ifndef TURTLELIB_DIFF_DRIVE_INCLUDE_GUARD_HPP
#define TURTLELIB_DIFF_DRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Differential drive kinematics for the turtlebot.

#include<iosfwd> // contains forward definitions for iostream objects

#include"turtlelib/geometry2d.hpp"
#include"turtlelib/se2d.hpp"

namespace turtlelib
{   
    /// \brief represent the wheel configurations
    struct WheelConfig
    {   
        /// \brief the left wheel configuration
        double lw = 0.0;

        /// \brief the right wheel configuration
        double rw = 0.0;
    };

    /// \brief represent the wheel velocities
    struct WheelVelocities
    {   
        /// \brief the left wheel velocity
        double lw = 0.0;

        /// \brief the right wheel velocity
        double rw = 0.0;
    };
    
    /// \brief differential drive kinematics for the turtlebot
    class DiffDrive
    {
    private:
        // Create a wheel configuration initialised to default values
        WheelConfig wheel_config {};

        // Create a robot configuration initialised to default values
        Transform2D robot_config {};

        // Physical dimensions of the turtlebot in m
        double half_trackwidth, wheel_radius;

    public:
        /// \brief The default DiffDrive class constructor
        /// \param half_track - the distance from the body frame to the wheel frame
        /// \param radius - the wheel radius
        DiffDrive(double half_track, double radius);

        /// \brief The DiffDrive class constructor
        /// \param half_track - the distance from the body frame to the wheel frame
        /// \param radius - the wheel radius
        /// \param wheel_position - the wheel configuration
        /// \param robot_pose - the robot configuration
        DiffDrive(double half_track, double radius, WheelConfig wheel_position, Transform2D robot_pose);


        /// \brief Fetches the robot configuration
        /// \return the robot configuration
        Transform2D get_robot_config() const;

        /// \brief Modifies the robot configuration
        /// \param robot_pose - the new robot configuration
        void set_robot_config(Transform2D robot_pose);

        /// \brief Fetches the wheel configuration
        /// \return the wheel configuration
        WheelConfig get_wheel_config() const;

        /// \brief Modifies the wheel configuration
        /// \param wheel_position - the new wheel configuration
        void set_wheel_config(WheelConfig wheel_position);

        /// \brief new wheel positions, calculate body twist
        /// \param wheel_position - the new wheel positions
        /// \return updated robot body twist
        Twist2D robot_body_twist(WheelConfig wheel_position) const;

        /// \brief using fk calculate twist between two wheel positions
        /// \param new_wheel_position - the new wheel positions
        /// \param prev_wheel_position - the previous wheel positions
        /// \return the twist between the two wheel positions
        Twist2D wheel_twist(WheelConfig new_wheel_position, WheelConfig prev_wheel_position) const;

        /// \brief using ik given body twist, calculate wheel velocities
        /// \param body_twist - the body twist
        /// \return the wheel velocities
        WheelVelocities inverse_kinematics(Twist2D body_twist);

        /// \brief using fk given new wheel positions, update robot pose
        /// \param wheel_position - the new wheel positions
        /// \return updated robot pose
        Transform2D forward_kinematics(WheelConfig wheel_position);
    };
}
#endif
