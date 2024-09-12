#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import numpy as np
from gazebo_msgs.msg import ModelStates
import tf.transformations as tft

class RobotTrajectoryPlotter:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('trajectory_plotter', anonymous=True)

        # Parameters
        self.trajectory_threshold_distance = 0.5  # meters
        self.trajectory_threshold_angle = 15  # degrees
        self.robot_size = 0.5  # Size of the robot in meters for the plot
        self.obstacle_radius = 0.075  # Radius of the obstacle circles

        # Variables to store the robot's position and orientation
        self.previous_position = None
        self.previous_orientation = None
        self.trajectory = []
        self.obstacles = []

        # Setup subscribers
        self.model_state_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)

        # Register shutdown hook
        rospy.on_shutdown(self.on_shutdown)

    def model_states_callback(self, msg):
        robot_name = "jackal"
        obstacle_prefix = "unit_cylinder_"

        if robot_name in msg.name:
            robot_index = msg.name.index(robot_name)
            robot_pose = msg.pose[robot_index]

            position = robot_pose.position
            orientation = robot_pose.orientation

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

        # Get the positions of the obstacles, only once
        if not self.obstacles:
            for i, model_name in enumerate(msg.name):
                if model_name.startswith(obstacle_prefix):
                    obstacle_pose = msg.pose[i]
                    obstacle_position = obstacle_pose.position
                    self.obstacles.append((obstacle_position.x, obstacle_position.y))

    def on_shutdown(self):
        # Plot the trajectory and obstacles at shutdown
        plt.figure()
        ax = plt.gca()

        # Plot obstacles as circles
        for obstacle in self.obstacles:
            circle = plt.Circle(obstacle, self.obstacle_radius, color='gray', fill=True)
            ax.add_patch(circle)

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
