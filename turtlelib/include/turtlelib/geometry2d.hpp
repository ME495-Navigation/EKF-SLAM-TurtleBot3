#ifndef TURTLELIB_GEOMETRY2D_HPP_INCLUDE_GUARD
#define TURTLELIB_GEOMETRY2D_HPP_INCLUDE_GUARD
/// \file
/// \brief Two-dimensional geometric primitives.


#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath>
namespace turtlelib
{
    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double PI=3.14159265358979323846;

    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    /// NOTE: implement this in the header file
    /// constexpr means that the function can be computed at compile time
    /// if given a compile-time constant as input
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
    {
        return std::abs(d1-d2)<epsilon;
    }

    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \returns radians
    constexpr double deg2rad(double deg)
    {
        return deg*(PI/180.0);
    }

    /// \brief convert radians to degrees
    /// \param rad - angle in radians
    /// \returns the angle in degrees
    constexpr double rad2deg(double rad)
    {
        return rad*(180.0/PI);
    }

    /// \brief wrap an angle to (-PI, PI]
    /// \param rad (angle in radians)
    /// \return an angle equivalent to rad but in the range (-PI, PI]
    double normalize_angle(double rad);

    /// static_assertions test compile time assumptions.
    /// You should write at least one more test for each function
    /// You should also purposely (and temporarily) make one of these tests fail
    /// just to see what happens
    static_assert(almost_equal(0, 0), "is_zero failed");
    static_assert(almost_equal(1.2, 1.2), "is_zero failed");
    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");
    static_assert(almost_equal(deg2rad(180.0), PI), "deg2rad failed");
    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg) failed");
    static_assert(almost_equal(rad2deg(PI), 180.0), "rad2deg) failed");
    static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");

    /// \brief a 2-Dimensional Point
    struct Point2D
    {
        /// \brief the x coordinate
        double x = 0.0;

        /// \brief the y coordinate
        double y = 0.0;
    };

    /// \brief output a 2 dimensional point as [xcomponent ycomponent]
    /// \param os - stream to output to
    /// \param p - the point to print
    std::ostream & operator<<(std::ostream & os, const Point2D & p);

    /// \brief input a 2 dimensional point
    ///   You should be able to read vectors entered as follows:
    ///   [x y] or x y
    /// \param is - stream from which to read
    /// \param p [out] - output vector
    /// HINT: See operator>> for Vector2D
    std::istream & operator>>(std::istream & is, Point2D & p);

    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
        /// \brief the x coordinate
        double x = 0.0;

        /// \brief the y coordinate
        double y = 0.0;

        /// \brief add a vector to another
        /// \param rhs - the vector to add
        Vector2D & operator+=(const Vector2D & rhs);

        /// \brief subtract a vector from another
        /// \param rhs - the vector to subtract
        Vector2D & operator-=(const Vector2D & rhs);

    };

    /// \brief Subtracting one point from another yields a vector
    /// \param head point corresponding to the head of the vector
    /// \param tail point corresponding to the tail of the vector
    /// \return a vector that points from p1 to p2
    /// NOTE: this is not implemented in terms of -= because subtracting two Point2D yields a Vector2D
    Vector2D operator-(const Point2D & head, const Point2D & tail);

    /// \brief Adding a vector to a point yields a new point displaced by the vector
    /// \param tail The origin of the vector's tail
    /// \param disp The displacement vector
    /// \return the point reached by displacing by disp from tail
    /// NOTE: this is not implemented in terms of += because of the different types
    Point2D operator+(const Point2D & tail, const Vector2D & disp);

    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// \param os - stream to output to
    /// \param v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as follows:
    ///   [x y] or x y
    /// \param is - stream from which to read
    /// \param v [out] - output vector
    /// Hint: The following may be useful:
    /// https://en.cppreference.com/w/cpp/io/basic_istream/peek
    /// https://en.cppreference.com/w/cpp/io/basic_istream/get
    ///
    /// The way input works is (more or less): what the user types is stored in a buffer until the user enters
    /// a newline (by pressing enter).  The iostream methods then process the data in this buffer character by character.
    /// Typically, each character is examined and then removed from the buffer automatically.
    /// If the characters don't match what is expected (e.g., we are expecting an int but the letter 'q' is encountered)
    /// an error flag is set on the stream object (e.g., std::cin).
    ///
    /// We have lower level control however. For example:
    /// peek looks at the next unprocessed character in the buffer without removing it
    /// get removes the next unprocessed character from the buffer.
    std::istream & operator>>(std::istream & is, Vector2D & v);

    /// \brief magnitude of a 2d vector
    /// \param v - vector v
    double magnitude(const Vector2D & v);

    /// \brief normalize a 2d vector
    /// \param v - the vector to normalize
    Vector2D normalize(const Vector2D & v);

    /// \brief addition of 2 vectors to return new vector
    /// \param lhs - the first vector to add
    /// \param rhs - the second vector to add
    Vector2D operator+(const Vector2D & lhs, const Vector2D & rhs);

    /// \brief subtraction of 2 vectors to return new vector
    /// \param lhs - the first vector
    /// \param rhs - the second vector (subtracts from first)
    Vector2D operator-(const Vector2D & lhs, const Vector2D & rhs);

    /// \brief multiply a vector by a scalar
    /// \param v - the vector
    /// \param s - the scalar
    Vector2D operator*(const Vector2D & v, const double & s);

    /// \brief multiply a scalar by a vector
    /// \param s - the scalar
    /// \param v - the vector
    Vector2D operator*(const double & s, const Vector2D & v);

    /// \brief the dot product of 2 vectors
    /// \param lhs - the first vector
    /// \param rhs - the second vector
    double dot(Vector2D lhs, Vector2D rhs);

    /// \brief the angle between 2 vectors
    /// \param lhs - the first vector
    /// \param rhs - the second vector
    double angle(Vector2D lhs, Vector2D rhs);
}

#endif
