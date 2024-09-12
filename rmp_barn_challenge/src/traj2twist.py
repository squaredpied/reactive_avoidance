#!/usr/bin/python

import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion
import time

# Publisher for velocity commands
cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
current_yaw = 0.0

# PID parameters
k_p = 3.0  # Proportional gain before 5
k_i = 0.01  # Integral gain
k_d = 0.1  # Derivative gain

integral = 0.0
previous_error = 0.0
previous_time = None

def wrap_angle(angle):
    # Normalize angle to be within the range [-pi, pi]
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def sigmoid(x):
    # Sigmoid function for modulating the linear velocity
    return 1 / (1 + math.exp(-x))

def VelConverter(trajectory):
    global current_yaw, integral, previous_error, previous_time
    twist = Twist()
    if trajectory.points:
        first_point = trajectory.points[0]
        if first_point.velocities:
            first_velocity = first_point.velocities[0]
            new_velocity_x = first_velocity.linear.x
            new_velocity_y = first_velocity.linear.y
            
            # Set linear and angular velocities to zero initially
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            new_direction = wrap_angle(math.atan2(new_velocity_y, new_velocity_x))
            delta_angle = wrap_angle(new_direction - current_yaw)


            # Get the current time for PID calculations
            current_time = time.time()
            if previous_time is None:
                previous_time = current_time

            # Calculate the time difference
            dt = current_time - previous_time


            # Calculate PID components
            error = delta_angle
            integral += error * dt
            derivative = (error - previous_error) / dt if dt > 0 else 0.0

            # PID control output
            twist.angular.z = k_p * error + k_i * integral + k_d * derivative

            # Modulate the linear velocity using a sigmoid function
            sigmoid_factor = sigmoid(-10 * abs(delta_angle) + 5)  # Adjust the constants for desired behavior
            twist.linear.x = math.sqrt(new_velocity_x**2 + new_velocity_y**2) * sigmoid_factor

            # Update previous error and time for the next iteration
            previous_error = error
            previous_time = current_time

    cmd_vel_pub.publish(twist)

def ReadOdom(data):
    global current_yaw
    orientation = data.pose.pose.orientation
    orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
    (_, _, current_yaw) = euler_from_quaternion(orientation_list)

if __name__ == "__main__":
    rospy.init_node("cmdvelPublisher")
    rospy.Subscriber("/jackal/command/trajectory", MultiDOFJointTrajectory, VelConverter)
    rospy.Subscriber("/odometry/filtered", Odometry, ReadOdom)

    rospy.spin()
