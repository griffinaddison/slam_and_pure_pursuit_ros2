from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # node1 = Node(
    #         package='safety_node',
    #         namespace='',
    #         executable='safety_node',
    #         name='safety_node',
    #         parameters=[
    #         {"thresh": 0.5},
    #     ]
    #     )
    
    node = Node(
        package='pure_pursuit',
        namespace='',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        parameters=[
            {"lookahead_distance": 2.0},
            {"velocity": 5.0},
        ]
        )



    ld = LaunchDescription()

    #ld.add_action(node1)
    ld.add_action(node)
    return ld
