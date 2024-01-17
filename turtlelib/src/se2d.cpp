#include <turtlelib/se2d.hpp>
#include <iostream>
#include <vector>
#include <cmath>

namespace turtlelib{
std::ostream & operator<<(std::ostream & os, const Twist2D & tw)
{
    return os << "[" << tw.omega <<" "<< tw.x << " " << tw.y << "]";
}

std::istream & operator>>(std::istream & is, Twist2D & tw)
{
    char next = is.peek(); //look at next character

    if(next == '[')
    {
        is.get(); //remove next character if it is '['
        is >> tw.omega;        
        is >> tw.x;
        is >> tw.y;
        is.get(); //remove ']'
    }
    else
    {
        is >> tw.omega >> tw.x >> tw.y;
    }

    return is;
}

Transform2D::Transform2D(): trans2d(), rot2d()
{}

Transform2D::Transform2D(Vector2D trans): trans2d(trans), rot2d()
{}

Transform2D::Transform2D(double radians): trans2d(), rot2d(radians)
{}

Transform2D::Transform2D(Vector2D trans, double radians): trans2d(trans), rot2d(radians)
{}

Point2D Transform2D::operator()(Point2D p) const
{
    Point2D p_new;
    p_new.x = trans2d.x + p.x;
    p_new.y = trans2d.y + p.y;
    return p_new;
}

Vector2D Transform2D::operator()(Vector2D v) const
{
    return {v.x*cos(rot2d)-v.y*sin(rot2d)+trans2d.x, 
    v.x*sin(rot2d)+v.y*cos(rot2d)+trans2d.y};
}

Twist2D Transform2D::operator()(Twist2D v) const
{
    return {v.omega,
            v.omega*trans2d.y+v.x*cos(rot2d)-v.y*sin(rot2d),
            -v.omega*trans2d.x+v.x*sin(rot2d)+v.y*cos(rot2d)};
}

Transform2D Transform2D::inv() const
{
    return {{-trans2d.x*cos(rot2d)-trans2d.y*sin(rot2d), -trans2d.y*cos(rot2d)+trans2d.x*sin(rot2d)}, -rot2d};
}

}