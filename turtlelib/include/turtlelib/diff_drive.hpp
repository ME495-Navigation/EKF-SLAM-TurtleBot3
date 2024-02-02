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
        /// \brief Create a wheel configuration initialised to default values
        WheelConfig wheel_config {};

        /// \brief Create a robot configuration initialised to default values
        Transform2D robot_config {};

    public:
        /// \brief The default DiffDrive class constructor
        DiffDrive();

        /// \brief The DiffDrive class constructor
        /// \param wheel_config - the wheel configuration
        /// \param robot_config - the robot configuration
        DiffDrive(WheelConfig wheel_position, Transform2D robot_pose);

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
