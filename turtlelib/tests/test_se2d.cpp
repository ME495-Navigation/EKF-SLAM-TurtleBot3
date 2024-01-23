#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <turtlelib/se2d.hpp>
#include <iostream>
#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath>
#include <sstream>


namespace turtlelib{
TEST_CASE( "output stream twist", "[twist]" ) //Abhishek, Sankar
{
    Twist2D twist;
    twist.omega = 7.7;
    twist.x = 8.12;
    twist.y = 1.93;
    std::stringstream os;
    os << twist;
    REQUIRE( os.str() == "[7.7 8.12 1.93]" );
}

TEST_CASE( "input stream twist", "[twist]" ) //Abhishek, Sankar
{
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

TEST_CASE( "transform constructor()", "[transform]" ) //Abhishek, Sankar
{
    Transform2D transform;

    REQUIRE_THAT(transform.rotation(), Catch::Matchers::WithinRel(0.0));
    REQUIRE_THAT(transform.translation().x, Catch::Matchers::WithinRel(0.0));
    REQUIRE_THAT(transform.translation().y, Catch::Matchers::WithinRel(0.0));
}

TEST_CASE( "transform constructor(Vector2D)", "[transform]" ) //Abhishek, Sankar
{
    Transform2D transform(Vector2D {2.3,3.1});

    REQUIRE_THAT(transform.rotation(), Catch::Matchers::WithinRel(0.0));
    REQUIRE_THAT(transform.translation().x, Catch::Matchers::WithinRel(2.3));
    REQUIRE_THAT(transform.translation().y, Catch::Matchers::WithinRel(3.1));
}

TEST_CASE( "transform constructor(double)", "[transform]" ) //Abhishek, Sankar
{
    Transform2D transform(PI);

    REQUIRE_THAT(transform.rotation(), Catch::Matchers::WithinRel(PI));
    REQUIRE_THAT(transform.translation().x, Catch::Matchers::WithinRel(0.0));
    REQUIRE_THAT(transform.translation().y, Catch::Matchers::WithinRel(0.0));
}

TEST_CASE( "transform constructor(Vector2D, double)", "[transform]" ) //Abhishek, Sankar
{
    Transform2D transform(Vector2D{2.3,3.1}, PI);

    REQUIRE_THAT(transform.rotation(), Catch::Matchers::WithinRel(PI));
    REQUIRE_THAT(transform.translation().x, Catch::Matchers::WithinRel(2.3));
    REQUIRE_THAT(transform.translation().y, Catch::Matchers::WithinRel(3.1));
}

TEST_CASE( "output stream transform", "[transform]" ) //Abhishek, Sankar
{
    Transform2D transform(Vector2D{2.3,3.1}, PI);
    std::stringstream os;
    os << transform;
    REQUIRE( os.str() == "deg: 180 x: 2.3 y: 3.1" );
}

TEST_CASE( "input stream transform", "[transform]" ) //Abhishek, Sankar
{
    Transform2D transform_1,transform_2,transform_3;

    std::stringstream is_1,is_2,is_3;

    is_1 << "deg: 180 x: 2.3 y: 3.1"; //test type output of << operator
    is_1 >> transform_1;

    is_2 << "180 2.3 3.1"; //test type separated by spaces
    is_2 >> transform_2;

    is_3 << "180\n2.3\n3.1"; //test type separated by new line
    is_3 >> transform_3;

    REQUIRE_THAT(transform_1.rotation(), Catch::Matchers::WithinRel(PI));
    REQUIRE_THAT(transform_1.translation().x, Catch::Matchers::WithinRel(2.3));
    REQUIRE_THAT(transform_1.translation().y, Catch::Matchers::WithinRel(3.1));
    REQUIRE_THAT(transform_2.rotation(), Catch::Matchers::WithinRel(PI));
    REQUIRE_THAT(transform_2.translation().x, Catch::Matchers::WithinRel(2.3));
    REQUIRE_THAT(transform_2.translation().y, Catch::Matchers::WithinRel(3.1));
    REQUIRE_THAT(transform_3.rotation(), Catch::Matchers::WithinRel(PI));
    REQUIRE_THAT(transform_3.translation().x, Catch::Matchers::WithinRel(2.3));
    REQUIRE_THAT(transform_3.translation().y, Catch::Matchers::WithinRel(3.1));
}

TEST_CASE( "transform multiplication", "[transform]" ) //Abhishek, Sankar
{
    Transform2D transform_1(Vector2D{1.0,1.0}, 0.0);
    Transform2D transform_2(Vector2D{2.0,2.0}, PI/2.0);
    Transform2D resulting_transform;

    resulting_transform = transform_1*transform_2;

    REQUIRE_THAT(resulting_transform.rotation(), Catch::Matchers::WithinRel(PI/2.0));
    REQUIRE_THAT(resulting_transform.translation().x, Catch::Matchers::WithinRel(3.0));
    REQUIRE_THAT(resulting_transform.translation().y, Catch::Matchers::WithinRel(3.0));
}

TEST_CASE("inv() Transform" "[se2d]") // Rahul,Roy
{
    double a = turtlelib::PI/2.0;
    Vector2D vec ={0.0,1.0};
    Transform2D trans = {{vec},a};
    REQUIRE_THAT(trans.inv().rotation(), Catch::Matchers::WithinAbs(-a, 1e-5));
    REQUIRE_THAT(trans.inv().translation().x, Catch::Matchers::WithinAbs(-1.0, 1e-5));
    REQUIRE_THAT(trans.inv().translation().y, Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("rotation()" "[se2d]") // Rahul,Roy
{
    double angle_test= 5.0;
    Transform2D test{angle_test};
    REQUIRE_THAT(test.rotation(), Catch::Matchers::WithinAbs(angle_test, 1e-5));
}

TEST_CASE("translation()" "[se2d]") // Rahul,Roy
{
    double x_test= 5.0;
    double y_test= 3.0;
    Transform2D test{{x_test,y_test}};
    REQUIRE_THAT(test.translation().x, Catch::Matchers::WithinAbs(x_test, 1e-5));
    REQUIRE_THAT(test.translation().y, Catch::Matchers::WithinAbs(y_test, 1e-5));
}

TEST_CASE("matrix mult operator - operator=", "[transform]") // Srikanth,Schelbert
{
    Vector2D test_trans_ab = {1.0, 2.0};
    double test_rot_ab = 0.0;
    Vector2D test_trans_bc = {2.0, 3.0};
    double test_rot_bc = PI/2;
    Transform2D T_ab1 = {test_trans_ab, test_rot_ab};
    Transform2D T_ab2 = {test_trans_ab, test_rot_ab};
    Transform2D T_ab3 = {test_trans_ab, test_rot_ab}; // made 3 since fcn returns overwritten tf
    Transform2D T_bc = {test_trans_bc, test_rot_bc};
    REQUIRE_THAT((T_ab1*=T_bc).translation().x, Catch::Matchers::WithinAbs(3.0, 1e-5));
    REQUIRE_THAT((T_ab2*=T_bc).translation().y, Catch::Matchers::WithinAbs(5.0, 1e-5));
    REQUIRE_THAT((T_ab3*=T_bc).rotation(), Catch::Matchers::WithinAbs(PI/2.0, 1e-5));
}

TEST_CASE("operator()(Vector2D v)","[transform]") // Srikanth,Schelbert
{
    double test_rot = -PI/2.0;
    double test_x = 1.0;
    double test_y = 1.0;
    Transform2D T_ab{{test_x,test_y}, test_rot};
    Vector2D v_b{1, 1};
    Vector2D v_a = T_ab(v_b);
    REQUIRE_THAT(v_a.x, Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(v_a.y, Catch::Matchers::WithinAbs(-1.0, 1e-5));
}

TEST_CASE("operator()(Point2D v)","[transform]") // Srikanth,Schelbert
{
    double test_rot = -PI/2.0;
    double test_x = 1.0;
    double test_y = 1.0;
    Transform2D T_ab{{test_x,test_y}, test_rot};
    Point2D p_b{1, 1};
    Point2D p_a = T_ab(p_b);
    REQUIRE_THAT(p_a.x, Catch::Matchers::WithinAbs(2.0, 1e-5));
    REQUIRE_THAT(p_a.y, Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("operator()(Twist2D t)","[transform]") // Srikanth,Schelbert
{
    double test_rot = PI/2.0;
    double test_x = 0.0;
    double test_y = 1.0;
    Transform2D T_ab{{test_x,test_y}, test_rot};
    Twist2D T_b{1, 1, 1};
    Twist2D T_a = T_ab(T_b);
    REQUIRE_THAT(T_a.omega, Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(T_a.x, Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(T_a.y, Catch::Matchers::WithinAbs(1.0, 1e-5));
}
}