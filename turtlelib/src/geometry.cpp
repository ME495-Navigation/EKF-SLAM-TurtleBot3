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

}