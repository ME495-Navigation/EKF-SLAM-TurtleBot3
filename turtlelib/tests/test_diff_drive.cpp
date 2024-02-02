#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <turtlelib/diff_drive.hpp>
#include <iostream>
#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath>
#include <sstream>

namespace turtlelib{
TEST_CASE( "pure translation forward", "[kinematics]" )
{   
    // radius is set to 1 for ease in calculations
    DiffDrive diff_drive(0.8, 1.0);
    Twist2D t = Twist2D{0.0, 1.0, 0.0};

    WheelVelocities wheel_vel = diff_drive.inverse_kinematics(t);
    REQUIRE_THAT(wheel_vel.lw, Catch::Matchers::WithinRel(1.0));
    REQUIRE_THAT(wheel_vel.rw, Catch::Matchers::WithinRel(1.0));

    WheelConfig wheel_config {1.0,1.0};
    Transform2D new_config = diff_drive.forward_kinematics(wheel_config);
    REQUIRE_THAT(new_config.translation().x, Catch::Matchers::WithinRel(1.0));
    REQUIRE_THAT(new_config.translation().y, Catch::Matchers::WithinRel(0.0));
    REQUIRE_THAT(new_config.rotation(), Catch::Matchers::WithinRel(0.0));
}

TEST_CASE( "pure rotation", "[kinematics]" )
{   
    // radius is set to 1 for ease in calculations
    DiffDrive diff_drive(1.0, 1.0);
    Twist2D t = Twist2D{1.0, 0.0, 0.0};

    WheelVelocities wheel_vel = diff_drive.inverse_kinematics(t);
    REQUIRE_THAT(wheel_vel.lw, Catch::Matchers::WithinRel(-1.0));
    REQUIRE_THAT(wheel_vel.rw, Catch::Matchers::WithinRel(1.0));

    WheelConfig wheel_config {-1.0,1.0};
    Transform2D new_config = diff_drive.forward_kinematics(wheel_config);
    REQUIRE_THAT(new_config.translation().x, Catch::Matchers::WithinRel(0.0));
    REQUIRE_THAT(new_config.translation().y, Catch::Matchers::WithinRel(0.0));
    REQUIRE_THAT(new_config.rotation(), Catch::Matchers::WithinRel(1.0));
}

TEST_CASE( "circular arc", "[kinematics]" )
{   
    // radius is set to 1 for ease in calculations
    DiffDrive diff_drive(1.0, 1.0);
    Twist2D t = Twist2D{2*PI, 2.0, 0.0};

    WheelVelocities wheel_vel = diff_drive.inverse_kinematics(t);
    REQUIRE_THAT(wheel_vel.lw, Catch::Matchers::WithinRel(-2*PI + 2.0));
    REQUIRE_THAT(wheel_vel.rw, Catch::Matchers::WithinRel(2*PI + 2.0));

    WheelConfig wheel_config {-2*PI + 2.0, 2*PI + 2.0};
    Transform2D new_config = diff_drive.forward_kinematics(wheel_config);
    REQUIRE_THAT(new_config.translation().x, Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(new_config.translation().y, Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(new_config.rotation(), Catch::Matchers::WithinAbs(2*PI, 1e-5));
}

TEST_CASE( "invalid twist", "[kinematics]" )
{   
    // radius is set to 1 for ease in calculations
    DiffDrive diff_drive(1.0, 1.0);
    Twist2D t = Twist2D{2*PI, 2.0, 1.0};

    REQUIRE_THROWS_AS(diff_drive.inverse_kinematics(t), std::logic_error);
}
}