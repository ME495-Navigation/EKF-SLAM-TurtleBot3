#include <catch2/catch_test_macros.hpp>
#include <turtlelib/geometry2d.hpp>
#include <iostream>

TEST_CASE( "angle is normalized", "[normalize]" ) {
    REQUIRE( turtlelib::normalize_angle(5.0) == 0.0 );
}