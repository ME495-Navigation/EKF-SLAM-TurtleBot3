#include <turtlelib/se2d.hpp>
#include <turtlelib/geometry2d.hpp>
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
    p_new.x = p.x*cos(rot2d)-p.y*sin(rot2d)+trans2d.x;
    p_new.y = p.x*sin(rot2d)+p.y*cos(rot2d)+trans2d.y;
    return p_new;
}

Vector2D Transform2D::operator()(Vector2D v) const
{
    return {v.x*cos(rot2d)-v.y*sin(rot2d), 
    v.x*sin(rot2d)+v.y*cos(rot2d)};
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

Transform2D & Transform2D::operator*=(const Transform2D & rhs)
{
    trans2d.x = rhs.trans2d.x*cos(rot2d)-rhs.trans2d.y*sin(rot2d) + trans2d.x;
    trans2d.y = rhs.trans2d.x*sin(rot2d)+rhs.trans2d.y*cos(rot2d) + trans2d.y;
    rot2d = rot2d + rhs.rot2d;
    return *this;
}

Vector2D Transform2D::translation() const
{
    return {trans2d.x , trans2d.y};
}

double Transform2D::rotation() const
{
    return rot2d;
}

std::ostream & operator<<(std::ostream & os, const Transform2D & tf)
{
    return os << "deg: " << rad2deg(tf.rot2d) <<" x: "<< tf.trans2d.x << " y: " << tf.trans2d.y;
}

std::istream & operator>>(std::istream & is, Transform2D & tf)
{   
    std::string trash_1, trash_2, trash_3;
    double rotation {};
    Vector2D translation {};
    char next = is.peek(); //look at next character

    if(next == 'd')
    {
        is >> trash_1; //remove deg:
        is >> rotation; //assign deg value to rotation         
        is >> trash_2; //remove x:
        is >> translation.x; //assign x value to translation.x
        is >> trash_3; //remove y:
        is >> translation.y; //assign y value to translation.y
    }
    else
    {
        is >> rotation >> translation.x >> translation.y;
    }
    tf = Transform2D(translation, deg2rad(rotation));
    return is;
}

Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
{
    lhs *= rhs;
    return lhs;
}

Transform2D integrate_twist(Twist2D twist)
{
    if(twist.omega == 0.0){
        return Transform2D {{twist.x,twist.y}, 0.0};
    }
    else{

        double x = twist.y / twist.omega;
        double y = - twist.x / twist.omega;
        Transform2D Tsb {{x,y}, 0.0};
        Transform2D Tbs = Tsb.inv();
        Transform2D Tss_p {{0.0,0.0}, twist.omega};
        
        return Tbs*Tss_p*Tsb;
    }
}
}