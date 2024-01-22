#include<iostream>
#include <turtlelib/se2d.hpp>
#include <turtlelib/geometry2d.hpp>
#include <turtlelib/svg.hpp>

using turtlelib::Transform2D;
using turtlelib::Vector2D;
using turtlelib::Point2D;
using turtlelib::Twist2D;
using turtlelib::Svg;
using turtlelib::SvgPoint;
using turtlelib::SvgLine;

int main() {

    Transform2D t_ab, t_bc;
    Point2D p_a;
    Vector2D v_b;
    Twist2D V_b;
    Svg svg("/tmp/frames.svg");
    SvgPoint p_1,p_2,p_3;
    SvgLine l_a,l_b,l_c;

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

    Transform2D A,B,C;
    B = (t_ab.inv())*A;

    svg.draw_frame(A,"{A}");
    svg.draw_frame(t_ab,"{B}");
    svg.draw_frame(t_ab*t_bc,"{C}");
    
    std::cout << "Enter point p_a:" << std::endl;
    std::cin >> p_a;
    std::cout << "p_a: " << p_a << std::endl;
    std::cout << "p_b: " << (t_ab.inv())(p_a) << std::endl;
    std::cout << "p_c: " << ((t_ab*t_bc).inv())(p_a) << std::endl;

    p_1.point = p_a;
    p_2.point = (t_ab.inv())(p_a);
    p_2.fill = "brown";
    p_3.point = ((t_ab*t_bc).inv())(p_a);
    p_3.fill = "orange";
    svg.draw_point(p_1);
    svg.draw_point(p_2);
    svg.draw_point(p_3);

    std::cout << "Enter vector v_b:" << std::endl;
    std::cin >> v_b;
    std::cout << "v_bhat: " << normalize(v_b) << std::endl;
    std::cout << "v_a: " << t_ab(v_b) << std::endl;
    std::cout << "v_b: " << v_b << std::endl;
    std::cout << "v_c: " << (t_bc.inv())(v_b) << std::endl;

    l_b.x2 = t_ab.translation().x;
    l_b.y2 = t_ab.translation().y;
    l_b.x1 = t_ab(normalize(v_b)).x + l_b.x2;
    l_b.y1 = t_ab(normalize(v_b)).y + l_b.y2;
    l_b.stroke = "brown";
    svg.draw_line(l_b);

    l_a.x2 = 0.0;
    l_a.y2 = 0.0;
    l_a.x1 = t_ab(v_b).x + l_a.x2 ;
    l_a.y1 = t_ab(v_b).y + l_a.y2;
    l_a.stroke = "purple";
    svg.draw_line(l_a);

    l_c.x2 = (t_ab*t_bc).translation().x;
    l_c.y2 = (t_ab*t_bc).translation().y;
    l_c.x1 = (t_ab(v_b)).x + (t_ab*t_bc).translation().x; 
    l_c.y1 = (t_ab(v_b)).y + (t_ab*t_bc).translation().y;
    l_c.stroke = "orange";
    svg.draw_line(l_c);

    std::cout << "Enter twist V_b: " << std::endl;
    std::cin >> V_b;
    std::cout << "V_a: " << t_ab(V_b) << std::endl;
    std::cout << "V_b: " << V_b << std::endl;
    std::cout << "V_c: " << (t_bc.inv())(V_b) << std::endl;

    return 0;
}