cmake_minimum_required(VERSION 3.8)
project(nuslam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Doxygen)
find_package(Armadillo)

rosidl_generate_interfaces(
${PROJECT_NAME}_int 
"msg/Landmarks.msg"
"srv/InitialPose.srv"
LIBRARY_NAME ${PROJECT_NAME}
DEPENDENCIES
geometry_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_int "rosidl_typesupport_cpp")

include_directories(${ARMADILLO_INCLUDE_DIRS})

add_executable(slam src/slam.cpp)
ament_target_dependencies(slam rclcpp std_msgs std_srvs geometry_msgs sensor_msgs
nuturtlebot_msgs nav_msgs tf2_ros tf2 visualization_msgs turtlelib)
target_link_libraries(slam turtlelib::turtlelib ${ARMADILLO_LIBRARIES})
target_link_libraries(slam "${cpp_typesupport_target}")

add_executable(landmarks src/landmarks.cpp)
ament_target_dependencies(landmarks rclcpp std_msgs std_srvs geometry_msgs sensor_msgs
nuturtlebot_msgs nav_msgs tf2_ros tf2 visualization_msgs turtlelib)
target_link_libraries(landmarks turtlelib::turtlelib ${ARMADILLO_LIBRARIES})
target_link_libraries(landmarks "${cpp_typesupport_target}")


install(TARGETS
slam landmarks
DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Building documentation should be optional.
# To build documentation pass -DBUILD_DOCS=ON when generating the build system
option(BUILD_DOCS "Build the documentation" ON)

# build just because Doxygen is missing
if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})
    # Turn the README.md into the homepage of the doxygen docs
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

    # Tell Doxygen where to find the documentation
    doxygen_add_docs(doxygen include/ src/ README.md ALL)

    # The documentation will be in the build/html directory
    # The main page is build/html/index.html
endif()

install(DIRECTORY
        config
        launch
        srv
        msg
        tests
        DESTINATION share/${PROJECT_NAME}/)

if(NOT CMAKE_BUILD_TYPE)        # If no build type is set, set one
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Type of build." FORCE)
endif()

include(CTest)

if(BUILD_TESTING)
    # Find the Unit testing framework. In this example, Catch2
    find_package(Catch2 3 REQUIRED)

    # A test is just an executable that is linked against the unit testing library
    add_executable(circle_fit_test tests/circle_tests.cpp)

    target_link_libraries(circle_fit_test ${ARMADILLO_LIBRARIES} Catch2::Catch2WithMain)

    # register the test with CTest, telling it what executable to run
    add_test(NAME circle_fit_test COMMAND circle_fit_test)

endif()

ament_package()
