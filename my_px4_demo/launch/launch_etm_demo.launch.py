from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # 声明可配置参数
    # declare_x_arg = DeclareLaunchArgument(
    #     'i', default_value=1, description='Follower index, starting from 1'
    # )

    node_list = []
    for i in range(1,5):
        node_list.append(
            Node(
                package='my_px4_demo',
                executable='formation_node',
                name=f'Follower_{i}',
                output='screen',
                parameters=[
                    {'i': i},
                    ]
                )
        )

    return LaunchDescription(node_list)

