#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <turtlelib/geometry2d.hpp>
#include <iostream>
#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath>
#include <sstream>

namespace turtlelib{
TEST_CASE( "angle is normalized", "[normalize]" ) {
    REQUIRE( normalize_angle(PI) == PI );
    REQUIRE( normalize_angle(-PI) == -PI );
    REQUIRE( normalize_angle(0.0) == 0.0 );
    REQUIRE( normalize_angle(-PI/4.0) == -PI/4.0 );
    REQUIRE( normalize_angle(3*PI/2.0) == -PI/2.0 );
    REQUIRE( normalize_angle(-5*PI/2.0) == -PI/2.0 );
    REQUIRE_THAT(normalize_angle(-5*PI/2.0), Catch::Matchers::WithinRel(-PI/2.0));
}

TEST_CASE( "output stream 2d point", "[2d point output stream]" ) {
    Point2D point;
    point.x = 5.7;
    point.y = 6.3;
    std::stringstream os;
    os << point;
    REQUIRE( os.str() == "[5.7 6.3]" );
}
}