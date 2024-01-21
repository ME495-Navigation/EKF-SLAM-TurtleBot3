#include <turtlelib/svg.hpp>
#include <turtlelib/se2d.hpp>
#include <turtlelib/geometry2d.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>

namespace turtlelib
{
Svg::Svg(const std::string & filepath)
{
    outputFile.open(filepath);
    outputFile << "<svg width=\"8.500000in\" height=\"11.000000in\" " 
    << "viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">" << std::endl;

    outputFile << R"delimiter(
<defs>
<marker
    style="overflow:visible"
    id="Arrow1Sstart"
    refX="0.0"
    refY="0.0"
    orient="auto">
    <path
        transform="scale(0.2) translate(6,0)"
        style="fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt"
        d="M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z "
        />
    </marker>
</defs>
)delimiter" << std::endl;
}

Svg::~Svg() 
{
    if (outputFile.is_open())
    {
        outputFile << "</svg>" << std::endl; 
        outputFile.close();
    }
}

void Svg::draw_point(SvgPoint svg_point)
{
    double new_cx = (svg_point.point.x * 96) + 408;
    double new_cy = (svg_point.point.y * -96) + 528;
    outputFile << "<circle cx=\"" << new_cx <<"\" cy=\"" << new_cy << 
    "\" r=\"" << svg_point.r << "\" stroke=\"" << svg_point.fill << 
    "\" fill=\"" << svg_point.fill << "\" stroke-width=\"" << svg_point.stroke_width << "\" />" << std::endl;
}

void Svg::draw_line(SvgLine svg_line)
{
    double new_x1 = (svg_line.x1 * 96) + 408;
    double new_x2 = (svg_line.x2 * 96) + 408;
    double new_y1 = (svg_line.y1 * -96) + 528;
    double new_y2 = (svg_line.y2 * -96) + 528;
    outputFile << "<line x1=\"" << new_x1 << "\" x2=\"" << new_x2 << "\" y1=\"" << new_y1
    << "\" y2=\"" << new_y2 << "\" stroke=\"" << svg_line.stroke << "\" stroke-width=\"" << svg_line.stroke_width 
    << "\" marker-start=\"" << svg_line.marker_start << "\" /> />" << std::endl;
}

void Svg::draw_text(double x, double y, std::string text)
{
    double new_x = (x * 96) + 408;
    double new_y = (y * -96) + 528;
    outputFile << "<text x=\"" << new_x << "\" y=\"" << new_y << "\">" << text << "</text>" << std::endl;
}

void Svg::draw_frame(Transform2D frame, std::string name = "")
{   
    Point2D x {1.0,0.0}, y {0.0,1.0}, new_x, new_y;
    
    new_x = frame(x);
    new_y = frame(y);

    SvgLine x_axis,y_axis;

    x_axis.x2 = frame.translation().x;
    x_axis.y2 = frame.translation().y;
    y_axis.x2 = frame.translation().x;
    y_axis.y2 = frame.translation().y;

    x_axis.x1 = new_x.x;
    x_axis.y1 = new_x.y;
    y_axis.x1 = new_y.x;
    y_axis.y1 = new_y.y;

    x_axis.stroke = "red";
    y_axis.stroke = "green";

    outputFile << "<g>" << std::endl;
    draw_line(x_axis);
    draw_line(y_axis);
    draw_text(x_axis.x2 + 0.1, x_axis.y2 + 0.1, name);
    outputFile << "</g>" << std::endl;
}

}
