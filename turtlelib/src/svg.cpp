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
    outputFile << "<svg width=\"8.500000in\" height=\"11.000000in\"" 
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

    // if (outputFile.is_open())
    // {
    //     outputFile.close();
    //     std::cout << "closing file" << std::endl;
    // }
}

// Svg::~Svg() 
// {
//     if (outputFile.is_open())
//     {
//         outputFile.close();
//     }
// }
}
