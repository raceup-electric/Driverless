"""Path Planner module."""
import logging
import time
from asyncio import Future
from typing import List

import matplotlib.pyplot as plt
import rclpy
from fs_msgs.msg import Cone
from fszhaw_msgs.msg import CurrentPosition, PlannedTrajectory
from fs_msgs.msg import ConeArrayWithCovariance 				####
from interfaces.srv import OptimizePath
from rclpy.node import Node
from scipy.spatial.distance import cdist

from path_planning.algorithm.exploration.exploration import Exploration
from path_planning.model.coordinate import Coordinate
from path_planning.model.mode import Mode
from path_planning.model.racetrajectory import RaceTrajectory
from path_planning.model.setup import Setup
from path_planning.planned_path_filter import PlannedPathFilter
from path_planning.start_finish_detector import StartFinishDetector
from path_planning.track_config import TrackConfig
from path_planning.util.path_planning_helpers import get_distance

SHOW_PLOT_EXPLORATION = False

class PathPlanner(Node):
    """
    The Path Planner.

    The path planner receives the current position from Localisation and the perceived cones from Perception
    and calculates the path the vehicle should take, and optimizes the path once the full track is known.
    """

    # default parameters
    LAPS = 2
    SHOW_CALC_TIMES = False
    SHOW_PLOT_EXPLORATION = False
    MOCK_CURRENT_POSITION = True
    TRACK_NAME = "sim_tool"

    # logging
    LOG_LEVEL = logging.INFO

    # set constants
    EXPLORATION_VELOCITY = 5.0
    MAX_CONES = 40
    START_FINISH_CONES_NEEDED = 2

    # set class variables
    mode = Mode.EXPLORATION
    index = 0
    laps_completed = 0
    exploration_cycle_times = []
    track_config = TrackConfig.Default
    setup = Setup.TRACK_DRIVE
    current_position = None
    all_cones = []
    blue_cones = []
    yellow_cones = []
    orange_cones = []
    big_orange_cones = []
    unknown_cones = []
    calculated_path = []
    optimized_path = []

    def __init__(self):
        """
        Initialize the path planner.

        Creates the subscribtion for the current position and the receiving cones,
        and creates a publisher for the planned path.
        """
        super().__init__('path_planner')

        # set logging level and format
        logging.basicConfig(level=PathPlanner.LOG_LEVEL,
                            format='%(levelname)s:%(message)s')

        # initialize ROS parameters
        self.__initialize_parameters()

        # set track config
        self.__set_track_config(self.track_name)
	
        # initialize cone subscriber
        self.cone_subscription = self.create_subscription(
            Cone,
            'cone',
            self.__cone_listener_callback,
            200)
        self.cone_subscription  # prevent unused variable warning

        # initialize Current Position subscriber
        self.current_position_subscription = self.create_subscription(
            CurrentPosition,
            'current_position',
            self.__current_position_listener_callback,
            200)
        self.current_position_subscription  # prevent unused variable warning
												
										

        # initialize Planned Trajectory publisher        
        self.planned_trajectory_publisher = self.create_publisher(
            PlannedTrajectory, 'planned_trajectory', 200)

        # init start finish detector
        self.start_finish_detector = StartFinishDetector(
            self.current_position, self.track_config)

        # init planned path filter
        self.planned_path_filter = PlannedPathFilter(self.current_position)

        logging.info('-----------------------')
        logging.info('Path Planner initialized!')


    def __initialize_parameters(self):
        """
        Initialize the ROS parameters.

        Initializes the ROS parameters with the passed values
        or with the default values.
        """
        # laps ros parameter
        self.declare_parameter('laps', PathPlanner.LAPS)
        self.laps = self.get_parameter(
            'laps').get_parameter_value().integer_value

        # show plot exploration ros parameter
        self.declare_parameter('show_calc_times',
                               PathPlanner.SHOW_CALC_TIMES)
        self.show_calc_times = self.get_parameter(
            'show_calc_times').get_parameter_value().bool_value

        # show plot exploration ros parameter
        self.declare_parameter('show_plot_exploration',
                               PathPlanner.SHOW_PLOT_EXPLORATION)
        self.show_plot_exploration = self.get_parameter(
            'show_plot_exploration').get_parameter_value().bool_value

        # mock current position ros parameter
        self.declare_parameter('mock_current_position',
                               PathPlanner.MOCK_CURRENT_POSITION)
        self.mock_current_position = self.get_parameter(
            'mock_current_position').get_parameter_value().bool_value

        # track name ros parameter
        self.declare_parameter('track_name',
                               PathPlanner.TRACK_NAME)
        self.track_name = self.get_parameter(
            'track_name').get_parameter_value().string_value

        logging.info('ROS Parameters set!')

    def __set_track_config(self, track_name: str):
        """
        Set the track configuration.

        Load and set the configurations for the specific track
        specified in the track config class.
        """
        if track_name == 'acceleration.csv':
            self.track_config = TrackConfig.Acceleration
            self.setup = Setup.ACCELERATION
        elif track_name == 'skidpad.csv':
            self.track_config = TrackConfig.Skidpad
            self.setup = Setup.SKID_PAD
        else:
            self.setup = Setup.TRACK_DRIVE
            if track_name == 'small_track.csv':
                self.track_config = TrackConfig.SmallTrack
            elif track_name == 'rand.csv':
                self.track_config = TrackConfig.Rand
            elif track_name == 'comp_2021.csv':
                self.track_config = TrackConfig.Comp2021
            elif track_name == 'garden_light.csv':
                self.track_config = TrackConfig.GardenLight
            else:
                self.track_config = TrackConfig.Default

        self.current_position = self.track_config.START_CURRENT_POSITION

        logging.info(
            f'Track Configuration loaded!\n\
            Track Config: {self.track_config.NAME}\n\
            Setup: {self.setup.name}')

    def __current_position_listener_callback(self, current_position: CurrentPosition):
        """
        Execute callback function when receiving an update for the current position.

        :param current_position: The updated current position of the vehicle.
        """
        if self.mock_current_position != True:
            logging.debug(
                f'Set new Position: x:{current_position.vehicle_position_x} y:{current_position.vehicle_position_y}')
            self.current_position = current_position

    def __cone_listener_callback(self, next_cone: Cone):
        """
        Execute callback function when receiving the next cone.

        :param next_cone: The received next cone.
        """
        logging.debug(
            f'Current Position: {self.current_position.vehicle_position_x} {self.current_position.vehicle_position_y}\n\
                Next Cone: Tag={next_cone.color}, Coordinates=({next_cone.location.x}, {next_cone.location.y})')

        # add next cone to its corresponding list regarding it's color
        if SHOW_PLOT_EXPLORATION == False:						###############
        	self.__add_to_received_cones(next_cone)

        # handle start finish detection
        start_finish_detected = False
        if self.setup == Setup.TRACK_DRIVE:
            if self.index >= 20:
                start_finish_detected = self.start_finish_detector.detect_by_starting_position(
                    current_position=self.current_position)
        else:
            if self.index >= 20 and next_cone.color == Cone.ORANGE_BIG:
                # handle start finish detection if a big orange cone is detected
                start_finish_detected = self.start_finish_detector.detect_by_cones(index=self.index,
                                                                                   current_position=self.current_position,
                                                                                   next_cone=next_cone)
        if start_finish_detected:
            self.laps_completed += 1  # add counter laps completed

        if self.mode == Mode.EXPLORATION:

            if self.show_calc_times:
                # save start time
                t_start = time.perf_counter()

            # calculate the planned path with the Exploration Algorithm
            last_midpoint, planned_path = Exploration.calculate_path(
                self.current_position,
                next_cone,
                self.blue_cones[-PathPlanner.MAX_CONES:],
                self.yellow_cones[-PathPlanner.MAX_CONES:],
                self.orange_cones,
                self.big_orange_cones,
                self.track_config,
                self.show_plot_exploration)

            if self.show_calc_times:
                # add runtime of cycle
                self.exploration_cycle_times.append(
                    time.perf_counter() - t_start)

            if planned_path:
                # save the planned path for the optimization request            ################
                self.calculated_path.extend(planned_path)

            # filter the planned path
            filtered_path = self.planned_path_filter.filter_path(planned_path)

            if self.mock_current_position:
                # set last midpoint as new current position
                self.current_position = last_midpoint

            if filtered_path:
                # publish the filtered path
                self.publish_planned_path(filtered_path)

                if self.show_plot_exploration:
                    plt.ion()
                    filtered_path_x, filtered_path_y = zip(*filtered_path)
                    plt.plot(filtered_path_x, filtered_path_y, 'o', c='green')
                    plt.show()
                    plt.pause(0.0001)

        else:
            if self.show_plot_exploration:
                plt.ioff()  # deactivate interactive mode (from exploration algorithm)

    def __add_to_received_cones(self, next_cone: Cone):
        """
        Add receiving cone to a list corresponding to its color.

        :param next_cone: The received next cone.
        """
        self.all_cones.append([next_cone.location.x, next_cone.location.y])
        if next_cone.color == Cone.BLUE:
            self.blue_cones.append(
                [next_cone.location.x, next_cone.location.y])
        elif next_cone.color == Cone.YELLOW:
            self.yellow_cones.append(
                [next_cone.location.x, next_cone.location.y])
        elif next_cone.color == Cone.ORANGE_SMALL:
            self.orange_cones.append(
                [next_cone.location.x, next_cone.location.y])
        elif next_cone.color == Cone.ORANGE_BIG:
            self.big_orange_cones.append(
                [next_cone.location.x, next_cone.location.y])
        else:
            self.unknown_cones.append(
                [next_cone.location.x, next_cone.location.y])


    def publish_planned_path(self, planned_path: List[Coordinate]):
        """
        Publish the planned path.

        :param planned_path: The planned path to be published.
        """
        logging.info('-----------------------')
        logging.info(
            f'Publishing Planned Path: indexes {self.index} - {self.index + len(planned_path) - 1}')

        for point in planned_path:
            x = point[0]
            y = point[1]
            v = PathPlanner.EXPLORATION_VELOCITY

            logging.debug('-----------------------')
            logging.debug(
                f'Publishing Planned Path: i:{self.index} x:{x} y:{y} velocity:{v}')
            self.planned_trajectory_publisher.publish(PlannedTrajectory(
                index=self.index, target_x=x, target_y=y, target_velocity=v))
            self.index += 1

 

def main(args=None):
    """
    Run the path planner.

    :param args: Additional arguments (Default value = None).
    """
    rclpy.init(args=args)

    path_planner = PathPlanner()
    optimized_laps_published = False

    while rclpy.ok():
        rclpy.spin_once(path_planner)
        

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    path_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
