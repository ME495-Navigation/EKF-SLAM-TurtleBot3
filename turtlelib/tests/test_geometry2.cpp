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

TEST_CASE( "output stream 2d point", "[point]" ) {
    Point2D point;
    point.x = 5.7;
    point.y = 6.3;
    std::stringstream os;
    os << point;
    REQUIRE( os.str() == "[5.7 6.3]" );
}

TEST_CASE( "output stream vector", "[vector]" ) {
    Vector2D vector;
    vector.x = 2.1;
    vector.y = 0.93;
    std::stringstream os;
    os << vector;
    REQUIRE( os.str() == "[2.1 0.93]" );
}

TEST_CASE( "input stream 2d point", "[point]" ) {
    Point2D point_1, point_2;
    std::stringstream is_1,is_2;

    is_1 << "0.7 4.3";
    is_1 >> point_1;

    is_2 << "[9.5 1.32]";
    is_2 >> point_2;

    REQUIRE( point_1.x == 0.7 );
    REQUIRE( point_1.y == 4.3 );
    REQUIRE( point_2.x == 9.5 );
    REQUIRE( point_2.y == 1.32 );
}

TEST_CASE( "input stream vector", "[vector]" ) {
    Vector2D vector_1, vector_2;
    std::stringstream is_1,is_2;

    is_1 << "2.0 0.93";
    is_1 >> vector_1;

    is_2 << "[2.5 0.97]";
    is_2 >> vector_2;

    REQUIRE( vector_1.x == 2.0 );
    REQUIRE( vector_1.y == 0.93 );
    REQUIRE( vector_2.x == 2.5 );
    REQUIRE( vector_2.y == 0.97 );
}

TEST_CASE( "vector formation", "[vector]" ) {
    Point2D head, tail;
    Vector2D vec;

    head.x = 4.0;
    head.y = 4.0;
    tail.x = 2.0;
    tail.y = 1.0;
    vec = head - tail;

    REQUIRE( vec.x == 2.0 );
    REQUIRE( vec.y == 3.0 );
}

TEST_CASE( "point displacement", "[point]" ) {
    Point2D p, tail;
    Vector2D vec;

    vec.x = 4.0;
    vec.y = 4.0;
    tail.x = 2.0;
    tail.y = 1.0;
    p = tail + vec;

    REQUIRE( p.x == 6.0 );
    REQUIRE( p.y == 5.0 );
}
}