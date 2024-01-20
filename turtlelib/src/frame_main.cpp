#include<iostream>
#include <turtlelib/se2d.hpp>
#include <turtlelib/geometry2d.hpp>
#include <turtlelib/svg.hpp>


using turtlelib::Transform2D;
using turtlelib::Vector2D;
using turtlelib::Point2D;
using turtlelib::Twist2D;
using turtlelib::Svg;


int main() {
    
    Transform2D t_ab, t_bc;
    Point2D p_a;
    Vector2D v_b;
    Twist2D V_b;
    Svg svg("frames.svg");

    std::cout << "Enter transform T_{a,b}:" << std::endl;
    std::cin >> t_ab;
    std::cout << "Enter transform T_{b,c}:" << std::endl;
    std::cin >> t_bc;

    std::cout << "T_{a,b}: " << t_ab << std::endl;
    std::cout << "T_{b,a}: " << t_ab.inv() << std::endl;
    std::cout << "T_{b,c}: " << t_bc << std::endl;
    std::cout << "T_{c,b}: " << t_bc.inv() << std::endl;
    std::cout << "T_{a,c}: " << t_ab*t_bc << std::endl;
    std::cout << "T_{c,a}: " << (t_ab*t_bc).inv() << std::endl;

    std::cout << "Enter point p_a:" << std::endl;
    std::cin >> p_a;
    std::cout << "p_a: " << p_a << std::endl;
    std::cout << "p_b: " << (t_ab.inv())(p_a) << std::endl;
    std::cout << "p_c: " << ((t_ab*t_bc).inv())(p_a) << std::endl;

    std::cout << "Enter vector v_b:" << std::endl;
    std::cin >> v_b;
    std::cout << "v_bhat: " << normalize(v_b) << std::endl;
    std::cout << "v_a: " << t_ab(v_b) << std::endl;
    std::cout << "v_b: " << v_b << std::endl;
    std::cout << "v_c: " << (t_bc.inv())(v_b) << std::endl;

    std::cout << "Enter twist V_b: " << std::endl;
    std::cin >> V_b;
    std::cout << "V_a: " << t_ab(V_b) << std::endl;
    std::cout << "V_b: " << V_b << std::endl;
    std::cout << "V_c: " << (t_bc.inv())(V_b) << std::endl;

    return 0;
}