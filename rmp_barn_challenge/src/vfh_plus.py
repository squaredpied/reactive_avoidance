#!/usr/bin/python

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, PointStamped
import tf2_ros, tf2_geometry_msgs


def isfinite(value):
    return not (math.isinf(value) or math.isnan(value))

class SubgoalPlanner:
    def __init__(self):
        # Initialize the node handle and other necessary components
        rospy.init_node('subgoal_planner_node', anonymous=True)
        self.goal_received_ = False

        # Params
        self.smoothing_window = 5
        self.range = 0.8
        self.alpha  = math.pi / 36.0 
        self.r = 0.3 # size of the robot 0.3
        self.lidar_robot_heading = 0
        self.required_free_bins = 20

        self.angular_diff_weight = 3
        self.previous_bin_diff_weight = 1
        self.robot_heading_weight = 1.5


        self.num_sectors = int((2 * math.pi)/self.alpha)

        # Initialize histograms
        self.polar_histogram = None
        self.smooth_histogram = None
        self.binary_histogram = None

        self.previous_bin_index = None

        self.tf_buffer_ = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_)

        # Read parameters
        if not self.readParameters():
            rospy.logwarn("Some/No parameter(s) found. Using default values.")

        # Subscribe to laser scan and goal topics
        self.laser_scan_ = rospy.Subscriber("/front/scan", LaserScan, self.laserScanCallback)
        self.goal_ = rospy.Subscriber("goal_point", PointStamped, self.goalCallback)

        # Publisher for the sub goal
        self.sub_goal_ = rospy.Publisher("sub_goal", Point, queue_size=1)
    

    def readParameters(self):
        # Read parameters from the parameter server
        self.goal_bearing_range_ = rospy.get_param("~goal_bearing_range", 8)
        self.threshold_ = rospy.get_param("~threshold", 2.5)
        self.s_max_ = rospy.get_param("~s_max", 3.0)
        self.t_high_ = rospy.get_param("~t_high", 15000) #15000
        self.t_low_ = rospy.get_param("~t_low", 0)
        return True

        

    def getPointInAnotherFrame(self, point, point_frame, stamp, dest_frame):
        # Create a stamped pose for the goal
        pose = tf2_geometry_msgs.PointStamped()
        pose.header.frame_id = point_frame
        pose.header.stamp = stamp
        pose.point.x = point[0]
        pose.point.y = point[1]
        pose.point.z = point[2]

        # Transform goal to laser frame
        try:
            pose_in_dest_frame = self.tf_buffer_.transform(pose, dest_frame, rospy.Duration(1.0))
        except tf2_ros.TransformException as ex:
            # rospy.logwarn(f"{ex}")
            return
        
        return pose_in_dest_frame
        

    def getPolarHistogram(self, scan):

        # Define the resolution of each sector (self.alpha)
        num_sectors = int(2 * math.pi / self.alpha) 

        # Step 1: Create the polar histogram with object density calculation
        polar_histogram = np.zeros(num_sectors)

        for i in range(len(scan.ranges)):
            if isfinite(scan.ranges[i]) and scan.range_min < scan.ranges[i] < scan.range_max:
                # Determine which sector this laser scan point falls into
                angle = scan.angle_min + i * scan.angle_increment

                # Normalize angle to be within 0 to 2*PI
                if angle < 0:
                    angle += 2 * math.pi

                # Calculate the width of the arc (g_ij) based on the range
                try:
                    g_ij = math.asin(self.r / scan.ranges[i])
                except:
                    g_ij = 0

                # Calculate the sector limits
                sector_left_lim = int((angle - g_ij) / self.alpha) % num_sectors
                sector_right_lim = int((angle + g_ij) / self.alpha) % num_sectors

                a = 1 + ((scan.range_max -1)/2)**2
                # Add the contribution to each sector within the limits
                if sector_left_lim <= sector_right_lim:
                    for j in range(sector_left_lim, sector_right_lim + 1):
                        polar_histogram[j] += (a - (scan.ranges[i])**2) 
                        # (1 / (scan.ranges[i] + 0.00001))
                else:
                    # Handle wrapping around the 0-degree mark
                    for j in range(sector_left_lim, num_sectors):
                        polar_histogram[j] += (a - (scan.ranges[i])**2)
                    for j in range(0, sector_right_lim + 1):
                        polar_histogram[j] += (a - (scan.ranges[i])**2)

        # Set bins outside the angular range to the maximum value
        min_angle = scan.angle_min
        max_angle = scan.angle_max

        if min_angle < 0:
            min_angle += 2 * math.pi
        if max_angle < 0:
            max_angle += 2 * math.pi

        min_sector = int(min_angle / self.alpha) % num_sectors
        max_sector = int(max_angle / self.alpha) % num_sectors

        # If min_sector is greater than max_sector, the scan spans the 0 mark
        if min_sector > max_sector:
            # Set all sectors outside the range to the maximum value
            for i in range(max_sector + 1, min_sector):
                polar_histogram[i] = np.inf  #indicating maximum obstacle density
        else:
            # If the scan does not wrap around, set the bins before min_sector and after max_sector to the maximum value
            for i in range(0, min_sector):
                polar_histogram[i] = np.inf
            for i in range(max_sector + 1, num_sectors):
                polar_histogram[i] = np.inf

        return polar_histogram
    
    
    def smoothenPolarHistogram(self, polar_histogram):

        half_window = self.smoothing_window // 2
        smooth_histogram = np.copy(polar_histogram)

        for i in range(polar_histogram.shape[0]):
            # Determine the start and end indices of the smoothing window
            start_index = (i - half_window) % polar_histogram.shape[0]
            end_index = (i + half_window) % polar_histogram.shape[0]
            
            # Handle the wrap-around if needed
            if start_index < end_index:
                smooth_histogram[i] = np.mean(polar_histogram[start_index:end_index + 1])
            else:
                # When the window wraps around the 0-degree mark
                window_values = np.concatenate((polar_histogram[start_index:], polar_histogram[:end_index + 1]))
                smooth_histogram[i] = np.mean(window_values)
        
        return smooth_histogram
    
    def createBinaryHistogram(self, histogram):
        binary_histogram = np.zeros(histogram.shape[0], dtype=int)
        for i in range(histogram.shape[0]):
            if histogram[i] > self.t_high_:
                binary_histogram[i] = 1  # High obstacle density
            elif histogram[i] < self.t_low_:
                binary_histogram[i] = 0  # Low obstacle density

        return binary_histogram


    def laserScanCallback(self, scan):
        # Check if the goal has been received
        if not self.goal_received_:
            rospy.logwarn("Goal not received yet. Ignoring laser scan data.")
            return
        

        # Transform goal to laser frame
        try:
            goal_in_laser_frame = self.getPointInAnotherFrame([self.goal_W_.x, self.goal_W_.y, 
                                                               self.goal_W_.z], self.goal_frame_, scan.header.stamp, scan.header.frame_id)
        except tf2_ros.TransformException as ex:
            # rospy.logwarn(f"{ex}")
            return

        # Extract the transformed goal position
        goal_L = np.array([goal_in_laser_frame.point.x,
                        goal_in_laser_frame.point.y,
                        goal_in_laser_frame.point.z])

        goal_bearing = math.atan2(goal_L[1], goal_L[0])


        # Convert goal_bearing to range [0, 2pi] from [-pi, pi]
        if goal_bearing < 0:
            goal_bearing += 2 * math.pi


        goal_range = np.sqrt(goal_L[0] ** 2 + goal_L[1] ** 2)


        # Step 1 Get the polar histogram
        polar_histogram = self.getPolarHistogram(scan)

        # Step 2: Smoothen the histogram using a low pass filter
        # smooth_histogram = self.smoothenPolarHistogram(polar_histogram)

        # Step 3: Create a binary polar histogram using t_high and t_low
        binary_histogram = self.createBinaryHistogram(polar_histogram)


        # Calculate the sector index that corresponds to the goal bearing
        goal_sector_index = int(goal_bearing / self.alpha) % self.num_sectors

        

        # Scan for contiguous free bins around the goal
        contiguous_free_bins = 0

        for direction in [-1, 1]:
            for offset in range(self.num_sectors):
                check_sector_index = (goal_sector_index + direction * offset) % self.num_sectors
                if binary_histogram[check_sector_index] == 0:  # Free bin
                    contiguous_free_bins += 1
                    if contiguous_free_bins >= self.required_free_bins:
                        break
                else:
                    break

            if contiguous_free_bins >= self.required_free_bins:
                break

        # If there are enough contiguous free bins near the goal, publish the goal as the subgoal
        if contiguous_free_bins >= self.required_free_bins:
            self.sub_goal_.publish(self.goal_W_)
            return


        # Step 4: Select the steering direction using a cost function
        # Calculate the steering direction based on the binary histogram and goal direction
        best_direction = -1.0
        min_cost = float('inf')

        for i in range(binary_histogram.shape[0]):
            if binary_histogram[i] == 0:  # Free direction
                # Calculate the cost based on the difference between goal direction and this direction
                
                cost = self.calculateCost(i, goal_bearing)

                if cost < min_cost:
                    min_cost = cost
                    best_direction = i

        if best_direction != -1.0:
            # If a valid direction is found, send it to the robot's control system
            self.previous_bin_index = best_direction 
            self.publishSubgoal(best_direction*self.alpha, scan.header)
        else:
            rospy.logwarn("No valid steering direction found. Stopping the robot.")
            self.stopRobot()

            

    def publishSubgoal(self,angle, scan_header):

        # Create a stamped pose for the goal
        goal_pose = tf2_geometry_msgs.PointStamped()
        goal_pose.header.frame_id = scan_header.frame_id
        goal_pose.header.stamp = scan_header.stamp
        goal_pose.point.x = self.range * np.cos(angle)
        goal_pose.point.y = self.range * np.sin(angle)
        goal_pose.point.z = 0

        # Transform goal to laser frame
        try:
            goal_in_odom_frame = self.tf_buffer_.transform(goal_pose, self.goal_frame_, rospy.Duration(1.0))
        except tf2_ros.TransformException as ex:
            # rospy.logwarn(f"{ex}")
            return


        pt = Point()
        pt.x = goal_in_odom_frame.point.x
        pt.y = goal_in_odom_frame.point.y
        pt.z = 0

        self.sub_goal_.publish(pt)



    def goalCallback(self, goal):
        self.goal_W_ = Point(goal.point.x, goal.point.y, goal.point.z)
        self.goal_frame_ = goal.header.frame_id

        self.goal_received_ = True


    def calculateCost(self, direction_bin, goal_angle):
        totalCost = 0
        # Wrap the angles to the range [0, 2*pi)
        direction_angle = (direction_bin * self.alpha) % (2 * math.pi)
        goal_angle = goal_angle % (2 * math.pi)

        goal_bin = int(goal_angle/ self.alpha) % self.num_sectors

        # Calculate the angular difference
        angular_difference = self.diffinSectors(direction_bin, goal_bin) 
       
        diff3 = 0

        if self.previous_bin_index is not None:
            diff3 = self.diffinSectors(self.previous_bin_index, direction_bin)

        diff2 = self.diffinSectors(direction_bin, self.lidar_robot_heading)

        totalCost= (self.angular_diff_weight*angular_difference) + (self.previous_bin_diff_weight*diff3) + (self.robot_heading_weight * diff2)

        return totalCost  # You can add more complex terms to the cost function if needed


    def stopRobot(self):
        # Implement the function to stop the robot
        rospy.loginfo("Stopping the robot.")


    def diffinSectors(self, sec1, sec2):
        return min(abs(sec1-sec2), abs(sec1-sec2- self.num_sectors), abs(sec1-sec2 + self.num_sectors))


if __name__ == '__main__':
    try:
        subgoal_planner = SubgoalPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
