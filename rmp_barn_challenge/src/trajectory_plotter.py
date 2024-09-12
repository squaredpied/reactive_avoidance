#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
import tf.transformations as tft

class RobotTrajectoryPlotter:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('trajectory_plotter', anonymous=True)

        # Parameters
        self.trajectory_threshold_distance = 0.5  # meters
        self.trajectory_threshold_angle = 15  # degrees
        self.robot_size = 0.2  # Size of the robot in meters for the plot

        # Variables to store the robot's position and orientation
        self.previous_position = None
        self.previous_orientation = None
        self.trajectory = []
        self.transformed_laser_scans = []

        # Setup TF listener
        self.tf_listener = tf.TransformListener()

        # Setup subscribers
        self.odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('/front/scan', LaserScan, self.scan_callback)

        # Register shutdown hook
        rospy.on_shutdown(self.on_shutdown)

    def odom_callback(self, msg):
        # Extract position and orientation
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # Convert orientation from quaternion to Euler angles
        _, _, yaw = tft.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        if self.previous_position is not None:
            # Calculate distance and angle difference
            distance = np.sqrt((position.x - self.previous_position.x) ** 2 + (position.y - self.previous_position.y) ** 2)
            angle_diff = abs(np.degrees(yaw - self.previous_orientation))

            if distance > self.trajectory_threshold_distance or angle_diff > self.trajectory_threshold_angle:
                self.trajectory.append((position.x, position.y, yaw))
                self.previous_position = position
                self.previous_orientation = yaw

        else:
            # Initialize previous position and orientation
            self.previous_position = position
            self.previous_orientation = yaw

    def scan_callback(self, msg):
        try:
            # Wait for the transform between laser frame and odom frame
            self.tf_listener.waitForTransform('/odom', msg.header.frame_id, msg.header.stamp, rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform('/odom', msg.header.frame_id, msg.header.stamp)

            # Convert rotation to Euler angles
            _, _, yaw = tft.euler_from_quaternion(rot)

            # Transformation matrix from laser frame to odom frame
            transform_matrix = tft.concatenate_matrices(
                tft.translation_matrix(trans),
                tft.euler_matrix(0, 0, yaw)
            )

            # Transform laser scan data to odom frame
            angles = np.arange(msg.angle_min, msg.angle_max+ msg.angle_increment, msg.angle_increment)
            ranges = np.array(msg.ranges)

            # Filter out invalid ranges
            valid_indices = np.isfinite(ranges)
            ranges = ranges[valid_indices]
            angles = angles[valid_indices]

            x = ranges * np.cos(angles)
            y = ranges * np.sin(angles)
            laser_points = np.vstack((x, y, np.zeros_like(x), np.ones_like(x)))

            # Apply the transformation to each point
            transformed_points = np.dot(transform_matrix, laser_points)

            # Store the transformed points
            self.transformed_laser_scans.append(transformed_points[:2, :])

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"TF Exception: {e}")

    def on_shutdown(self):
        # Plot the trajectory and laser scans at shutdown
        plt.figure()
        ax = plt.gca()

        # Plot all transformed laser scans
        for scan in self.transformed_laser_scans:
            ax.plot(scan[0, :], scan[1, :], 'g.', alpha=0.1)  # Reduce alpha for better visualization of multiple scans

        # Plot the trajectory and robot at each point
        if self.trajectory:
            trajectory_x = [p[0] for p in self.trajectory]
            trajectory_y = [p[1] for p in self.trajectory]
            ax.plot(trajectory_x, trajectory_y, 'b-', lw=2, label='Trajectory')

            for robot_position in self.trajectory:
                robot_x, robot_y, yaw = robot_position
                triangle = self.get_robot_triangle(robot_x, robot_y, yaw)
                ax.plot(triangle[0], triangle[1], 'r-')

        ax.set_aspect('equal', 'box')
        ax.set_xlim(-10, 10)
        ax.set_ylim(-10, 10)
        plt.legend()
        plt.show()

    def get_robot_triangle(self, x, y, yaw):
        # Create a triangle representing the robot
        half_size = self.robot_size / 2
        front_x = x + half_size * np.cos(yaw)
        front_y = y + half_size * np.sin(yaw)
        left_x = x + half_size * np.cos(yaw + 2 * np.pi / 3)
        left_y = y + half_size * np.sin(yaw + 2 * np.pi / 3)
        right_x = x + half_size * np.cos(yaw - 2 * np.pi / 3)
        right_y = y + half_size * np.sin(yaw - 2 * np.pi / 3)

        triangle_x = [front_x, left_x, right_x, front_x]
        triangle_y = [front_y, left_y, right_y, front_y]

        return triangle_x, triangle_y

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        plotter = RobotTrajectoryPlotter()
        plotter.spin()
    except rospy.ROSInterruptException:
        pass