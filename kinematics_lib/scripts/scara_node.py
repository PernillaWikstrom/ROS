#!/usr/bin/python3


import rospy
import square_trajectory as st
from sensor_msgs.msg import JointState
import numpy as np


def inverse_kinematics_2D(x: float, y: float, z: float):
    """Computes the inverse kinematic to be in a specific position.
    Robotics: Modelling, Planning and Control, ISBN: 978-1-84996-634-4

    Args:
        x (float): [m] position
        y (float): [m] position
        z (float): [m] position

    Returns:
        (list): the inverse kinematic per joint
    """

    # Fixed lengths of the Scara robot arm
    l0 = 0.07
    l1 = 0.3
    l2 = 0.35
    # Analytical solution
    x0 = x - l0
    c2 = (x0**2.0 + y**2.0 - l1**2.0 - l2**2.0) / (2.0 * l1 * l2)
    theta = np.arctan2(np.sqrt(1 - c2**2), c2)
    alpha = np.arctan2((l2 * np.sin(theta)), (l1 + l2 * np.cos(theta)))
    beta = np.arctan2(y, x0)
    return [beta - alpha, theta, -z]


def main():
    rospy.init_node("scara_node")
    hz = 10
    rate = rospy.Rate(hz)

    trajectory_publisher = st.SquareTrajectory(hz)

    # define the ros message for publishing the joint positions
    joint_msg = JointState()
    joint_msg.name = ["rotational1", "rotational2", "translation"]

    # define the ros topic where to publish the joints values
    topic_name = rospy.get_param("~topic_name", "controller/joint_states")
    publisher = rospy.Publisher(topic_name, JointState, queue_size=10)

    while not rospy.is_shutdown():
        # get the current point in the trajectory
        point = trajectory_publisher.get_point()
        if point is not None:
            # get the inverse kinematic solution for this point
            q = inverse_kinematics_2D(*point)
            # publish this solution
            joint_msg.position = q
            publisher.publish(joint_msg)
        else:
            trajectory_publisher.restart()
        # publish the path to be visualized in rviz
        trajectory_publisher.publish_path()
        rate.sleep()


if __name__ == "__main__":
    main()
