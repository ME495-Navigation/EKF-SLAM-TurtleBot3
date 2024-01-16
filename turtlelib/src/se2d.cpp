#include <turtlelib/se2d.hpp>
#include <iostream>


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
}