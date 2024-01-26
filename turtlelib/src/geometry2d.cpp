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

Vector2D operator+(const Vector2D & lhs, const Vector2D & rhs)
{
    Vector2D v;
    v.x = lhs.x + rhs.x;
    v.y = lhs.y + rhs.y;
    return v;
}

Vector2D & Vector2D::operator+=(const Vector2D & rhs)
{
    x = x + rhs.x;
    y = y + rhs.y;
    return *this;
}

Vector2D operator-(const Vector2D & lhs, const Vector2D & rhs)
{
    Vector2D v;
    v.x = lhs.x - rhs.x;
    v.y = lhs.y - rhs.y;
    return v;
}   

Vector2D & Vector2D::operator-=(const Vector2D & rhs)
{
    x = x - rhs.x;
    y = y - rhs.y;
    return *this;
}

Vector2D operator*(const Vector2D & v, const double & s)
{
    Vector2D vec;
    vec.x = v.x * s;
    vec.y = v.y * s;
    return vec;
}

Vector2D operator*(const double & s, const Vector2D & v)
{
    Vector2D vec;
    vec.x = v.x * s;
    vec.y = v.y * s;
    return vec;
}

double dot(Vector2D lhs, Vector2D rhs)
{
    double dt;
    dt = lhs.x*rhs.x + lhs.y*rhs.y;
    return dt;
}

double angle(Vector2D lhs, Vector2D rhs)
{
    double dotprod = dot(lhs, rhs);
    double mag_lhs = magnitude(lhs);
    double mag_rhs = magnitude(rhs);
    
    // catch division by zero
    if (mag_lhs == 0.0 || mag_rhs == 0.0) {
        std::cerr << "Magnitudes of both vectors are zero." << std::endl;
        return 0.0;
    }

    return std::acos(dotprod / (mag_lhs * mag_rhs));
}
}