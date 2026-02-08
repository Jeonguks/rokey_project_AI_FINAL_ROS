# launch/fight_fire.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

'''
ros2 launch fight_fire fight_fire.launch.py \
  --ros-args -r __ns:=/robot2

ros2 launch fight_fire fight_fire.launch.py \
  --ros-args -r __ns:=/robot6
'''
def generate_launch_description():
    return LaunchDescription([
        Node(
            package="fight_fire",
            executable="task_controller",
            name="task_controller",
            output="screen",
            remappings=[
                ("/tf", "tf"),
                ("/tf_static", "tf_static"),
            ],
        ),
        Node(
            package="fight_fire",
            executable="perception_node",
            name="perception_node",
            output="screen",
            remappings=[
                ("/tf", "tf"),
                ("/tf_static", "tf_static"),
            ],
        ),
    ])
