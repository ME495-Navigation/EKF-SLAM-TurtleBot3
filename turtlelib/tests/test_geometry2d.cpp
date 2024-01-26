#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <turtlelib/geometry2d.hpp>
#include <iostream>
#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath>
#include <sstream>

namespace turtlelib{
TEST_CASE( "angle is normalized", "[normalize]" ) {
    
    REQUIRE_THAT(normalize_angle(PI), Catch::Matchers::WithinRel(PI));
    REQUIRE_THAT(normalize_angle(-PI), Catch::Matchers::WithinRel(-PI));
    REQUIRE_THAT(normalize_angle(0.0), Catch::Matchers::WithinRel(0.0));
    REQUIRE_THAT(normalize_angle(-PI/4.0), Catch::Matchers::WithinRel(-PI/4.0));
    REQUIRE_THAT(normalize_angle(3*PI/2.0), Catch::Matchers::WithinRel(-PI/2.0));
    REQUIRE_THAT(normalize_angle(-5*PI/2.0), Catch::Matchers::WithinRel(-PI/2.0));
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

    REQUIRE_THAT(point_1.x, Catch::Matchers::WithinRel(0.7));
    REQUIRE_THAT(point_1.y, Catch::Matchers::WithinRel(4.3));
    REQUIRE_THAT(point_2.x, Catch::Matchers::WithinRel(9.5));
    REQUIRE_THAT(point_2.y, Catch::Matchers::WithinRel(1.32));
}

TEST_CASE( "input stream vector", "[vector]" ) {
    Vector2D vector_1, vector_2;
    std::stringstream is_1,is_2;

    is_1 << "2.0 0.93";
    is_1 >> vector_1;

    is_2 << "[2.5 0.97]";
    is_2 >> vector_2;

    REQUIRE_THAT(vector_1.x, Catch::Matchers::WithinRel(2.0));
    REQUIRE_THAT(vector_1.y, Catch::Matchers::WithinRel(0.93));
    REQUIRE_THAT(vector_2.x, Catch::Matchers::WithinRel(2.5));
    REQUIRE_THAT(vector_2.y, Catch::Matchers::WithinRel(0.97));
}

TEST_CASE( "vector formation", "[vector]" ) {
    Point2D head, tail;
    Vector2D vec;

    head.x = 4.0;
    head.y = 4.0;
    tail.x = 2.0;
    tail.y = 1.0;
    vec = head - tail;

    REQUIRE_THAT(vec.x, Catch::Matchers::WithinRel(2.0));
    REQUIRE_THAT(vec.y, Catch::Matchers::WithinRel(3.0));
}

TEST_CASE( "vector normalize", "[vector]" ) {
    Vector2D vec,vec_n;

    vec.x = 1.0;
    vec.y = 1.0;
    vec_n = normalize(vec);

    REQUIRE_THAT(vec_n.x, Catch::Matchers::WithinAbs(0.7071, 0.01));
    REQUIRE_THAT(vec_n.y, Catch::Matchers::WithinAbs(0.7071, 0.01));
}

TEST_CASE( "point displacement", "[point]" ) {
    Point2D p, tail;
    Vector2D vec;

    vec.x = 4.0;
    vec.y = 4.0;
    tail.x = 2.0;
    tail.y = 1.0;
    p = tail + vec;

    REQUIRE_THAT(p.x, Catch::Matchers::WithinRel(6.0));
    REQUIRE_THAT(p.y, Catch::Matchers::WithinRel(5.0));
}

TEST_CASE( "vector addition", "[vector]" ) {
    Vector2D vec,vec_1,vec_2;

    vec_1.x = 7.3;
    vec_1.y = 5.3;
    vec_2.x = 4.2;
    vec_2.y = -9.7;

    vec = vec_1 + vec_2;

    REQUIRE_THAT(vec.x, Catch::Matchers::WithinRel(11.5));
    REQUIRE_THAT(vec.y, Catch::Matchers::WithinRel(-4.4));
}

TEST_CASE( "vector addition to itself", "[vector]" ) {
    Vector2D vec,vec_1;

    vec.x = 4.0;
    vec.y = -9.2;
    vec_1.x = 7.3;
    vec_1.y = 5.2;

    vec += vec_1;

    REQUIRE_THAT(vec.x, Catch::Matchers::WithinRel(11.3));
    REQUIRE_THAT(vec.y, Catch::Matchers::WithinRel(-4.0));
}

TEST_CASE( "vector subtraction", "[vector]" ) {
    Vector2D vec,vec_1,vec_2;

    vec_1.x = 7.3;
    vec_1.y = 5.3;
    vec_2.x = 4.2;
    vec_2.y = -9.7;

    vec = vec_1 - vec_2;

    REQUIRE_THAT(vec.x, Catch::Matchers::WithinRel(3.1));
    REQUIRE_THAT(vec.y, Catch::Matchers::WithinRel(15.0));
}

TEST_CASE( "vector subtraction to itself", "[vector]" ) {
    Vector2D vec,vec_1;

    vec.x = 4.0;
    vec.y = -9.2;
    vec_1.x = 7.3;
    vec_1.y = 5.2;

    vec -= vec_1;

    REQUIRE_THAT(vec.x, Catch::Matchers::WithinRel(-3.3));
    REQUIRE_THAT(vec.y, Catch::Matchers::WithinRel(-14.4));
}

TEST_CASE( "vector multiplication by scalar on lhs", "[vector]" ) {
    Vector2D vec,vec_1;
    double sr = 5.0;

    vec_1.x = 4.0;
    vec_1.y = -9.2;
    
    vec = sr*vec_1;

    REQUIRE_THAT(vec.x, Catch::Matchers::WithinRel(20.0));
    REQUIRE_THAT(vec.y, Catch::Matchers::WithinRel(-46.0));
}

TEST_CASE( "vector multiplication by scalar on rhs", "[vector]" ) {
    Vector2D vec,vec_1;
    double sr = 5.0;

    vec_1.x = 4.0;
    vec_1.y = -9.2;
    
    vec = vec_1*sr;

    REQUIRE_THAT(vec.x, Catch::Matchers::WithinRel(20.0));
    REQUIRE_THAT(vec.y, Catch::Matchers::WithinRel(-46.0));
}

TEST_CASE( "vector dot product", "[vector]" ) {
    Vector2D vec_1,vec_2;
    double dtp;

    vec_1.x = 7.3;
    vec_1.y = 5.3;
    vec_2.x = 4.2;
    vec_2.y = -9.7;

    dtp = dot(vec_1,vec_2);

    REQUIRE_THAT(dtp, Catch::Matchers::WithinRel(-20.75));
}

TEST_CASE( "angle between two vectors", "[vector]" ) {
    Vector2D vec_1,vec_2;
    double ang;

    vec_1.x = 1.0;
    vec_1.y = 1.0;
    vec_2.x = -1.0;
    vec_2.y = 0.0;

    ang = angle(vec_1,vec_2);

    REQUIRE_THAT(ang, Catch::Matchers::WithinAbs(2.35619, 1e-5));
}
}
