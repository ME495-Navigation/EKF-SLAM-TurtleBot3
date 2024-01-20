#include <turtlelib/svg.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>

namespace turtlelib
{
Svg::Svg(const std::string & filepath)
{
    std::cout << "opening file" << std::endl;
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

    outputFile << "</svg>" << std::endl; 
}

Svg::~Svg() 
{
    if (outputFile.is_open())
    {
        outputFile.close();
    }
}

void Svg::draw_point(SvgPoint svg_point)
{
    outputFile << "<circle cx=\"" << svg_point.cx <<"\" cy=\"" << svg_point.cy << 
    "\" r=\"" << svg_point.r << "\" stroke=\"" << svg_point.stroke << 
    "\" fill=\"" << svg_point.fill << "\" stroke-width=\"" << svg_point.stroke_width << "\" />" << std::endl;
}

void Svg::draw_line(SvgLine svg_line)
{
    outputFile << "<line x1=\"" << svg_line.x1 << "\" x2=\"" << svg_line.x2 << "\" y1=\"" << svg_line.y1
    << "\" y2=\"" << svg_line.y2 << "\" stroke=\"" << svg_line.stroke << "\" stroke-width=\"" << svg_line.stroke_width 
    << "\" marker-start=\"" << svg_line.marker_start << "\" /> />" << std::endl;
}

void Svg::draw_text(double x, double y, std::string text)
{
    outputFile << "<text x=\"" << x << "\" y=\"" << y << "\">" << text << "</text>" << std::endl;
}

void Svg::draw_frame(SvgLine line_1, SvgLine line_2, std::string name = "")
{
    outputFile << "<g>" << std::endl;
    draw_line(line_1);
    draw_line(line_2);
    draw_text(line_1.x2 + 0.2, line_1.y2 + 0.2, name);
    outputFile << "</g>" << std::endl;
}

}
