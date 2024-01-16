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
}