#include <turtlelib/svg.hpp>
#include <iostream>
#include <vector>
#include <cmath>

namespace turtlelib
{
Svg::Svg(const std::string& filepath) 
{
    outputFile.open(filepath);
}

Svg::~Svg() 
{
    if (outputFile.is_open())
    {
        outputFile.close();
    }
}


}
