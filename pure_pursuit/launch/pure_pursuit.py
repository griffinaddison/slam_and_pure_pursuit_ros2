from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    node1 = Node(
            package='safety_node',
            namespace='',
            executable='safety_node',
            name='safety_node',
            parameters=[
            {"thresh": 0.5},
        ]
        )
    
    node = Node(
        package='pure_pursuit',
        namespace='',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        parameters=[
            {"lookahead_distance": 2.00}, #levine 1.25
            {"velocity": 5.5}, #levine 9.0 6.0 4.0
            {"speed_lookahead_distance": 3.00}, #levine 2.0
            {"brake_gain": 3.3}, #levine 12.0 
            {"visualize": True},
            {"wheel_base": 0.33},# levine 0.33 0.40
        ]
        )



    ld = LaunchDescription()

    #ld.add_action(node1)
    ld.add_action(node)
    return ld
