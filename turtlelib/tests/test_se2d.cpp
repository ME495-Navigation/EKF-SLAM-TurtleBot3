#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <turtlelib/se2d.hpp>
#include <iostream>
#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath>
#include <sstream>


namespace turtlelib{
TEST_CASE( "output stream twist", "[twist]" ) {
    Twist2D twist;
    twist.omega = 7.7;
    twist.x = 8.12;
    twist.y = 1.93;
    std::stringstream os;
    os << twist;
    REQUIRE( os.str() == "[7.7 8.12 1.93]" );
}

TEST_CASE( "input stream twist", "[twist]" ) {
    Twist2D twist_1,twist_2;
    std::stringstream is_1,is_2;

    is_1 << "3.2 8.3 5.65";
    is_1 >> twist_1;

    is_2 << "[2.5 2.14 86.0]";
    is_2 >> twist_2;

    REQUIRE( twist_1.omega == 3.2 );
    REQUIRE( twist_1.x == 8.3 );
    REQUIRE( twist_1.y == 5.65 );
    REQUIRE( twist_2.omega == 2.5 );
    REQUIRE( twist_2.x == 2.14 );
    REQUIRE( twist_2.y == 86.0 );
}
}