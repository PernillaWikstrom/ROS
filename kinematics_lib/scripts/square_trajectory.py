#!/usr/bin/python3

import rospy
import rospkg

rospack = rospkg.RosPack()
from geometry_msgs.msg import Pose, PoseArray
import numpy as np


class SquareTrajectory:
    """
    Computes a squares trajectory depending on sampling rate, corner position and desired speed of motion.
    """

    def __init__(
        self,
        hz: float = 10.0,
        vertices: np.array = np.array(
            [
                [0.27, -0.15, 0.0],
                [0.57, -0.15, 0.1],
                [0.57, 0.15, 0.1],
                [0.27, 0.15, 0.0],
            ]
        ),
        base_frame: str = "base",
    ):
        self._path_publisher = rospy.Publisher("desired_path", PoseArray, queue_size=10)
        self._path = PoseArray()
        self._path.header.frame_id = base_frame
        self._dt = 1.0 / hz
        # the 4 vertices of the square
        self._vertices = vertices
        self._current_segment = 0
        self._current_idx = 0
        speed = 0.05  # [m/s]
        self._waypoints = self.compute_waypoints_wrapper(speed)

    def compute_waypoints(
        self,
        distance_to_take: float,
        n_samples_per_dimension: float,
        p1: np.ndarray,
        p2: np.ndarray,
    ):
        """
        Computes a list of wayppoints from start to end position.

        Args:
            distance_to_take (float): [m]
            n_samples_per_dimension (float): distance to take divided by how far we can travel in one sample
            p1 (np.ndarray): start position [x, y, z]
            p2 (np.ndarray): end position [x, y, z]

        Returns:
            waypoints (list): list of waypoints between p1 and p2
        """

        max_number_samples = abs(
            n_samples_per_dimension[(np.absolute(n_samples_per_dimension)).argmax()]
        )
        if max_number_samples < 1.0:
            return []

        max_sampling_distance = distance_to_take / max_number_samples
        waypoints = [p1]
        for i in range(int(max_number_samples)):
            p = Pose()
            p.position.x = waypoints[i][0]
            p.position.y = waypoints[i][1]
            p.position.z = waypoints[i][2]
            p.orientation.w = 0.707
            p.orientation.y = 0.707
            self._path.poses.append(p)
            waypoints.append(waypoints[i] + max_sampling_distance)
        waypoints.append(p2)
        return waypoints

    def compute_waypoints_wrapper(self, speed: float):
        """Computes all the 4 segments of the square, given the initial vertices.

        Args:
            speed (float): [m/s] Constant speed of motion

        Returns:
            waypoints (list): list of waypoints for the full square pattern
        """
        # Distance in one sampling with dt
        dx = speed * self._dt
        # Looping over vertices
        waypoints = []
        for i in range(self._vertices.shape[0]):
            if i < 3:
                position_end = self._vertices[i + 1]
            else:
                position_end = self._vertices[0]

            distance_to_take = position_end - self._vertices[i]
            n_samples_per_dimension = distance_to_take / dx
            waypoints.append(
                self.compute_waypoints(
                    distance_to_take,
                    n_samples_per_dimension,
                    self._vertices[i],
                    position_end,
                )
            )

        return waypoints

    def publish_path(self):
        """
        Publish the path in rviz
        """
        self._path_publisher.publish(self._path)

    def next_segment(self):
        """
        Update next segment of the square (-1 if the path ended) and reset current index.
        """
        if self._current_segment == 3:
            self._current_segment = -1
        else:
            self._current_segment += 1

        self._current_idx = 0

    def get_point(self):
        """
        Returns the next point in the path. None if the path ended
        """
        if self._current_idx >= len(self._waypoints[self._current_segment]):
            self.next_segment()

        if self._current_segment == -1:
            return None

        current_point = (self._waypoints[self._current_segment])[self._current_idx]
        self._current_idx += 1
        return current_point

    def restart(self):
        """
        Resets the current segment and index to go through the path once again.
        """
        self._current_segment = 0
        self._current_idx = 0
