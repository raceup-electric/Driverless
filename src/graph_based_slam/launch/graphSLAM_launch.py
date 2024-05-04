# from launch import LaunchDescription
# from launch_ros.actions import Node


# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package="graph_based_slam",
#             executable="graph_SLAM",
#             output="screen",
#             emulate_tty=True,
#             prefix=['stdbuf -o L'],
#             parameters=[
#                 {"use_sim_time": True}
#             ]
#         )
#     ])
