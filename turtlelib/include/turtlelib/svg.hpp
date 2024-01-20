#ifndef TURTLELIB_SVG_INCLUDE_GUARD_HPP
#define TURTLELIB_SVG_INCLUDE_GUARD_HPP
/// \file
/// \brief SVG visualisation.

#include <fstream>
#include <string>
#include <iostream>

namespace turtlelib
{
    /// \brief svg visualisation class.
    class Svg
    {
    private:
        /// \brief output file
        std::ofstream outputFile;
    public:
        /// \brief create an SVG object
        Svg(const std::string& filepath);

        /// \brief destructor to close the file
        ~Svg();
    };
}

#endif