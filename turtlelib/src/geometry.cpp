#include <turtlelib/geometry2d.hpp>
#include <iostream>

namespace turtlelib{
double normalize_angle(double rad){

    // double epsilon=1.0e-12;
    while(rad > PI)
    {
        rad -= 2.0*PI;
    }
    while(rad < - PI)
    {
        rad += 2.0*PI;
    }
    return rad;
}

std::ostream & operator<<(std::ostream & os, const Point2D & p)
{
    return os << "[" << p.x << " " << p.y << "]";
}

std::istream & operator>>(std::istream & is, Point2D & p)
{
    char next = is.peek();

    if(next == '[')
    {
        is.get();
        is >> p.x;
        is >> p.y;
        is.get();
    }
    else
    {
        is >> p.x >> p.y;
    }

    return is;
}


std::ostream & operator<<(std::ostream & os, const Vector2D & v)
{
    return os << "[" << v.x << " " << v.y << "]";
}

std::istream & operator>>(std::istream & is, Vector2D & v)
{
    char next = is.peek();

    if(next == '[')
    {
        is.get();
        is >> v.x;
        is >> v.y;
        is.get();
    }
    else
    {
        is >> v.x >> v.y;
    }

    return is;
}
}