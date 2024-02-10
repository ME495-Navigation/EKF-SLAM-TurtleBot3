cmake_minimum_required(VERSION 3.22)

project(turtlelib)

add_library(turtlelib src/geometry2d.cpp src/se2d.cpp src/svg.cpp src/diff_drive.cpp)

target_include_directories(turtlelib
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)

target_compile_options(turtlelib PUBLIC -Wall -Wextra -pedantic)

target_compile_features(turtlelib PUBLIC cxx_std_17)

add_executable(frame_main src/frame_main.cpp)

target_link_libraries(frame_main turtlelib)

install(DIRECTORY include/turtlelib DESTINATION include)

install(TARGETS turtlelib frame_main EXPORT turtlelib-targets)

install(EXPORT turtlelib-targets
        FILE turtlelib-config.cmake
        NAMESPACE turtlelib::
        DESTINATION lib/cmake/${PROJECT_NAME})

find_package(Doxygen)

option(BUILD_DOCS "Build the documentation" ON)

if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})
    # Turn the README.md into the homepage of the doxygen docs
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

    # Tell Doxygen where to find the documentation
    doxygen_add_docs(doxygen include/ src/ README.md ALL)

    # The documentation will be in the build/html directory
    # The main page is build/html/index.html
endif()

include(CTest)

if(BUILD_TESTING)
    # Find the Unit testing framework. In this example, Catch2
    find_package(Catch2 3 REQUIRED)

    # A test is just an executable that is linked against the unit testing library
    add_executable(geometry2d_test tests/test_geometry2d.cpp)
    add_executable(se2d_test tests/test_se2d.cpp)
    add_executable(svg_test tests/test_svg.cpp)
    add_executable(diff_drive_test tests/test_diff_drive.cpp)

    target_link_libraries(geometry2d_test turtlelib Catch2::Catch2WithMain)
    target_link_libraries(se2d_test turtlelib Catch2::Catch2WithMain)
    target_link_libraries(svg_test turtlelib Catch2::Catch2WithMain)
    target_link_libraries(diff_drive_test turtlelib Catch2::Catch2WithMain)

    # register the test with CTest, telling it what executable to run
    add_test(NAME geometry2d_test COMMAND geometry2d_test)
    add_test(NAME se2d_test COMMAND se2d_test)
    add_test(NAME svg_test COMMAND svg_test)
    add_test(NAME diff_drive_test COMMAND diff_drive_test)

endif()