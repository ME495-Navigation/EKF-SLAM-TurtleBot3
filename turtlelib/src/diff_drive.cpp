#include <turtlelib/se2d.hpp>
#include <turtlelib/geometry2d.hpp>
#include <turtlelib/diff_drive.hpp>
#include <iostream>
#include <vector>
#include <cmath>

namespace turtlelib{
    
    DiffDrive::DiffDrive(double half_track, double radius):
    wheel_config(),
    robot_config(),
    half_trackwidth(half_track),
    wheel_radius(radius)
    {}

    DiffDrive::DiffDrive(double half_track, double radius, WheelConfig wheel_position, Transform2D robot_pose):
    wheel_config(wheel_position),
    robot_config(robot_pose),
    half_trackwidth(half_track),
    wheel_radius(radius)
    {}

    Transform2D DiffDrive::get_robot_config() const{
        return robot_config;
    }

    void DiffDrive::set_robot_config(Transform2D robot_pose){
        robot_config = robot_pose;
    }

    WheelConfig DiffDrive::get_wheel_config() const{
        return wheel_config;
    }

    void DiffDrive::set_wheel_config(WheelConfig wheel_position){
        wheel_config = wheel_position;
    }

    Twist2D DiffDrive::robot_body_twist(WheelConfig wheel_position) const{
        // assume absolute encoder readings (from initial wheel position)
        // this won't matter as we are working with difference
        // in readings anyways (from initial wheel positions)
        WheelVelocities wheel_vel;
        // assume unit time
        wheel_vel.lw = wheel_position.lw - this->wheel_config.lw;
        wheel_vel.rw = wheel_position.rw - this->wheel_config.rw;

        // fk calculation, refer to eq in doc
        Twist2D body_twist;
        body_twist.omega =  ((wheel_vel.rw - wheel_vel.lw)*wheel_radius)/(2*half_trackwidth);
        body_twist.x = 0.5*wheel_radius*cos(robot_config.rotation())*(wheel_vel.rw + wheel_vel.lw);
        body_twist.y = 0.5*wheel_radius*sin(robot_config.rotation())*(wheel_vel.rw + wheel_vel.lw);

        return body_twist;
    }

    WheelVelocities DiffDrive::inverse_kinematics(Twist2D body_twist){
        WheelVelocities wheel_vel;
        if (!(almost_equal(body_twist.y,0.0))){
            throw std::logic_error("Invalid twist. Y component must be zero!");
        }
        else{
            // ik calculations, refer to eq in doc
            wheel_vel.lw = (body_twist.x - half_trackwidth*body_twist.omega)/wheel_radius;
            wheel_vel.rw = (body_twist.x + half_trackwidth*body_twist.omega)/wheel_radius;
        }
        return wheel_vel;
    }

    Transform2D DiffDrive::forward_kinematics(WheelConfig wheel_position){
        // assume absolute encoder readings (from initial wheel position)
        // this won't matter as we are working with difference
        // in readings anyways (from initial wheel positions)
        WheelVelocities wheel_vel;
        // assume unit time
        wheel_vel.lw = wheel_position.lw - this->wheel_config.lw;
        wheel_vel.rw = wheel_position.rw - this->wheel_config.rw;

        //update the wheel position
        this->wheel_config = wheel_position;

        // fk calculation, refer to eq in doc
        Twist2D body_twist;
        body_twist.omega =  ((wheel_vel.rw - wheel_vel.lw)*wheel_radius)/(2*half_trackwidth);
        body_twist.x = 0.5*wheel_radius*cos(robot_config.rotation())*(wheel_vel.rw + wheel_vel.lw);
        body_twist.y = 0.5*wheel_radius*sin(robot_config.rotation())*(wheel_vel.rw + wheel_vel.lw);

        // update robot configuration in the world frame
        robot_config *= integrate_twist(body_twist);

        return robot_config;
    }




}