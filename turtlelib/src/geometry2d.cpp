#include <turtlelib/geometry2d.hpp>
#include <iostream>

namespace turtlelib{
double normalize_angle(double rad)
{
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
    char next = is.peek(); //look at next character

    if(next == '[')
    {
        is.get(); //remove next character if it is '['
        is >> p.x;
        is >> p.y;
        is.get(); //remove ']'
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

    if(next == '[') //look at next character
    {
        is.get(); //remove next character if it is '['
        is >> v.x;
        is >> v.y;
        is.get(); //remove ']'
    }
    else
    {
        is >> v.x >> v.y;
    }

    return is;
}

Vector2D operator-(const Point2D & head, const Point2D & tail)
{
    Vector2D vec;
    vec.x = head.x - tail.x;
    vec.y = head.y - tail.y;
    return vec;
}

double magnitude(const Vector2D & v)
{
    return std::sqrt(std::pow(v.x,2) + std::pow(v.y,2));
}

Vector2D normalize(const Vector2D & v)
{
    double mag;
    mag =magnitude(v);

    return {v.x/mag, v.y/mag};
}

Point2D operator+(const Point2D & tail, const Vector2D & disp)
{
    Point2D p;
    p.x = tail.x + disp.x;
    p.y = tail.y + disp.y;
    return p;
}

}