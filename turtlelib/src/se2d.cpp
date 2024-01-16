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

Transform2D::Transform2D() 
{
    // Declare a 2D vector of integers
    std::vector<std::vector<double>> trans2d;
    trans2d = {{1.0,0.0,0.0},{0.0,1.0,0.0},{0.0,0.0,1.0}};
}

Transform2D::Transform2D(Vector2D trans)
{
    // Declare a 2D vector of integers
    std::vector<std::vector<double>> trans2d;
    trans2d = {{1.0,0.0,trans.x},{0.0,1.0,trans.y},{0.0,0.0,1.0}};

}

Transform2D::Transform2D(double radians)
{
    // Declare a 2D vector of integers
    std::vector<std::vector<double>> trans2d;

    // Calculate sine and cosine
    double sine = sin(radians);
    double cosine = cos(radians);

    trans2d = {{cosine,-sine,0.0},{sine,cosine,0.0},{0.0,0.0,1.0}};
}

Transform2D::Transform2D(Vector2D trans, double radians)
{
    // Declare a 2D vector of integers
    std::vector<std::vector<double>> trans2d;

    // Calculate sine and cosine
    double sine = sin(radians);
    double cosine = cos(radians);

    trans2d = {{cosine,-sine,trans.x},{sine,cosine,trans.y},{0.0,0.0,1.0}};
}
}