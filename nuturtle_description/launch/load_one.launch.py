from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, Shutdown, SetLaunchConfiguration
from launch.substitutions import Command, PathJoinSubstitution, TextSubstitution, LaunchConfiguration, EqualsSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(name="use_jsp", default_value="true",
                              description="true (default): use jsp to publish joint states, otherwise no joint states published"),

        DeclareLaunchArgument(name="use_rviz", default_value="true",
                              description="true (default): start rviz, otherwise don't start rviz"),

        DeclareLaunchArgument(name="color", default_value="purple",
                              description="purple (default): set the color of the robot",
                              choices = ["red", "green", "blue", "purple"]),
        
        SetLaunchConfiguration(name="rviz_config",
                               value = [TextSubstitution(text="basic_"), LaunchConfiguration("color"), TextSubstitution(text=".rviz")]),

        Node(package="joint_state_publisher",
             executable="joint_state_publisher",
             namespace= LaunchConfiguration("color"),
             condition= IfCondition(
                    EqualsSubstitution(LaunchConfiguration("use_jsp"), "true"))
             ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace= LaunchConfiguration("color"),
            parameters=[
                {"robot_description" :
                 Command([TextSubstitution(text="xacro "),
                          PathJoinSubstitution(
                              [FindPackageShare("nuturtle_description"), "urdf", "turtlebot3_burger.urdf.xacro"]), TextSubstitution(text=" color:="), LaunchConfiguration("color")]),
                 "frame_prefix" : [LaunchConfiguration("color"), TextSubstitution(text="/")]
                }
            ]
            ),
        Node(
            package="rviz2",
            executable="rviz2",
            namespace= LaunchConfiguration("color"),
            arguments=["-d",
                       PathJoinSubstitution(
                           [FindPackageShare("nuturtle_description"), "config", LaunchConfiguration("rviz_config")]),
                        "-f", [LaunchConfiguration("color"), TextSubstitution(text="/"), TextSubstitution(text="base_footprint")]   
                           ],
            condition=IfCondition(
                    EqualsSubstitution(LaunchConfiguration("use_rviz"), "true")),
            on_exit=Shutdown()
            )
        ])