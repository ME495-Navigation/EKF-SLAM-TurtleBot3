#ifndef TURTLELIB_SVG_INCLUDE_GUARD_HPP
#define TURTLELIB_SVG_INCLUDE_GUARD_HPP
/// \file
/// \brief SVG visualisation.

#include <fstream>
#include <string>
#include <iostream>
#include <turtlelib/se2d.hpp>
#include <turtlelib/geometry2d.hpp>

namespace turtlelib
{

    /// \brief represent a point (svg)
    struct SvgPoint
    {
        /// \brief the x and y positions
        Point2D point {0.0,0.0};

        /// \brief the radius
        double r = 3.0;

        /// \brief the fill colour
        std::string fill = "purple";

        /// \brief the stroke width
        double stroke_width = 1.0;
    };

    /// \brief represent a line (svg)
    struct SvgLine
    {
        /// \brief the x1 position
        double x1= 0.0;

        /// \brief the x2 position
        double x2 = 0.0;

        /// \brief the y1 position
        double y1= 0.0;

        /// \brief the y2 position
        double y2= 0.0;

        /// \brief the stroke color
        std::string stroke = "purple";

        /// \brief the stroke width
        double stroke_width = 5.0;

        /// \brief the arrow start tag
        std::string marker_start= "url(#Arrow1Sstart)";
    };


    /// \brief svg visualisation class.
    class Svg
    {
    private:
        /// \brief output file
        std::ofstream outputFile;

    public:
        /// \brief create an SVG object with a specific output filepath
        Svg(const std::string& filepath);

        /// \brief destructor to close the file
        ~Svg();

        /// \brief draw a point
        void draw_point(SvgPoint svg_point);

        /// \brief draw a line
        void draw_line(SvgLine svg_line);

        /// \brief draw text
        /// \param x - the x position of the text
        /// \param y - the y position of the text
        /// \param text - the text to be printed
        void draw_text(double x, double y, std::string text);

        /// \brief draw a coordinate frame
        void draw_frame(Transform2D frame, std::string name);
    };
}

#endif