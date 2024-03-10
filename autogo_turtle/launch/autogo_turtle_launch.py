from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    autogo_turtle_geometry_node = Node(
        package='autogo_turtle',
        executable='autogo_turtle_geometry',
        name='autogo_turtle_geometry_node',
    )
    autogo_turtle_waypoints_node = Node(
        package='autogo_turtle',
        executable='autogo_turtle_waypoints',
        name='autogo_turtle_waypoints_node',
    )
    kill_turtle1 = ExecuteProcess(
        cmd=[[
            'ros2 service call ',
            '/kill ',
            'turtlesim/srv/Kill ',
            '"name: turtle1"'
        ]],
        shell=True
    )
    kill_turtle2 = ExecuteProcess(
        cmd=[[
            'ros2 service call ',
            '/kill ',
            'turtlesim/srv/Kill ',
            '"name: turtle2"'
        ]],
        shell=True
    )
    spawn_turtle2 = ExecuteProcess(
        cmd=[[
            'ros2 service call ',
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 1.5, y: 5.5, theta: 0, name: "turtle2"}"'
        ]],
        shell=True
    )
    move_turtle1 = ExecuteProcess(
        cmd=[[
            'ros2 service call ',
            '/turtle1/teleport_absolute ',
            'turtlesim/srv/TeleportAbsolute ',
            '"{x: 1.5, y: 8.5, theta: 0}"'
        ]],
        shell=True
    )
    clear_turtlesim = ExecuteProcess(
        cmd=[[
            'ros2 service call ',
            '/clear ',
            'std_srvs/srv/Empty'
        ]],
        shell=True
    )
    return LaunchDescription([
        move_turtle1,
        spawn_turtle2,
        TimerAction(
            period=1.0,
            actions=[clear_turtlesim],
        ),
        TimerAction(
            period=2.0,
            actions=[autogo_turtle_waypoints_node,
                     autogo_turtle_geometry_node],
        ),
        TimerAction(
            period=65.0,  # Adjust this period as needed
            actions=[kill_turtle2],
        ),
        TimerAction(
            period=565.0,  # Adjust this period as needed
            actions=[kill_turtle1],
        ),
    ])