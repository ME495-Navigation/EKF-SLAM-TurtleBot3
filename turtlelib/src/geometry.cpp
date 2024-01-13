#include <turtlelib/geometry2d.hpp>

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
}