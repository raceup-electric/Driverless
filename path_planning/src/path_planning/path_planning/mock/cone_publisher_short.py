import rclpy
from rclpy.node import Node

import csv
import logging
import os


from fs_msgs.msg import Cone, Track
from geometry_msgs.msg import Point
from path_planning.model.tag import Tag
from rclpy.node import Node
from graph_based_slam.msg import PoseGraph, GraphNode



class ConePublisher(Node):
    LOG_LEVEL = logging.INFO

    cones = []
    blue_cones = []
    yellow_cones = []
    orange_cones = []
    big_orange_cones = []
    unknown_cones = []

    i = 0
    last_cone = 0


    def __init__(self):
        super().__init__('cone_publisher')

        logging.basicConfig(level=ConePublisher.LOG_LEVEL,
                            format='%(levelname)s:%(message)s')

        self.subscription = self.create_subscription(
            PoseGraph,
            'pose_graph',
            self.pose_graph_callback,
            10  # QoS profile depth
        )
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(
            Cone,
            'cone',
            200)




    def pose_graph_callback(self, msg):
        logging.info(f'Received PoseGraph message')
        last_cone = len(self.cones)

        for cone in msg.graph_nodes[last_cone:]:
            if cone.color == "blue":
                self.cones.append(
                    [Tag.BLUE.value, cone.x, cone.y])
                self.blue_cones.append(
                    [Tag.BLUE.value, cone.x, cone.y])
            elif cone.color == "yellow":
                self.cones.append(
                    [Tag.YELLOW.value, cone.x, cone.y])
                self.yellow_cones.append(
                    [Tag.YELLOW.value, cone.x, cone.y])
            elif cone.color == "orange":
                self.cones.append(
                    [Tag.BIG_ORANGE.value, cone.x, cone.y])
                self.big_orange_cones.append(
                    [Tag.BIG_ORANGE.value, cone.x, cone.y])
            else:
                self.cones.append(
                    [Tag.UNKNOWN.value, cone.x, cone.y])
                self.unknown_cones.append(
                    [Tag.UNKNOWN.value, cone.x, cone.y])

            logging.info(f'Blue Cones: {len(self.blue_cones)}\n\
                Yellow Cones: {len(self.yellow_cones)}\n\
                Big Orange Cones: {len(self.big_orange_cones)}\n\
                Unknown Cones: {len(self.unknown_cones)}')
            
        if len(self.cones) > 0 and self.i < len(self.cones):
            self.__publish_cone(self.cones[self.i])
            self.i += 1
        last_cone = len(self.cones)
            
    def __publish_cone(self, cone):
        color = map_tag_to_color(cone[0])
        x = cone[1]
        y = cone[2]
        z = 0.0

        logging.debug(f'Publishing {self.i}: {color}, {x}, {y}')
        self.publisher_.publish(
            Cone(color=color, location=Point(x=x, y=y, z=z)))
        
        

def map_tag_to_color(tag):
    """
    Map a tag to it's corresponding cone color.

    :param tag: Tag of the cone or coordinate.
    """
    if tag == Tag.BLUE.value:
        return Cone.BLUE
    elif tag == Tag.YELLOW.value:
        return Cone.YELLOW
    elif tag == Tag.ORANGE.value:
        return Cone.ORANGE_SMALL
    elif tag == Tag.BIG_ORANGE.value:
        return Cone.ORANGE_BIG
    else:
        return Cone.UNKNOWN
    
def __reset_indexes(self):
        """Reset indexes."""
        self.i = 0


def main(args=None):
    rclpy.init(args=args)
    cone_subscriber = ConePublisher()
    rclpy.spin(cone_subscriber)
    cone_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
