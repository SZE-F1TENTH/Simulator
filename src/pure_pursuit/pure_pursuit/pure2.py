# code adapted from https://github.com/ladavis4/F1Tenth_Final_Project_and_ICRA2022/tree/main/pure_pursuit_pkg
import os

import rclpy
from rclpy.node import Node
from scipy import interpolate
import scipy.ndimage
from scipy.spatial.transform import Rotation as R
import numpy as np
import csv
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import LaserScan  # Add for obstacle detection
from sensor_msgs.msg import PointCloud2, PointField  # Add for pointcloud
import std_msgs.msg
import pdb 

class PurePursuit(Node):
    """
    Implement Pure Pursuit on the car
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # declare parameters
        self.declare_parameter("trajectory_csv", "/sim_ws/src/f1tenth_gym_ros/racelines/t5.csv")
        self.declare_parameter("pp_steer_L_fast", 2.5)
        self.declare_parameter("pp_steer_L_slow", 1.8)
        self.declare_parameter("kp_fast", 0.35)
        self.declare_parameter("kp_slow", 0.5)
        self.declare_parameter("L_threshold_speed", 4.0)
        self.declare_parameter("odom_topic", "ego_racecar/odom")
        self.declare_parameter("pure_pursuit_velocity_topic", "pure_pursuit_velocity")
        self.declare_parameter("drive_topic", "ego_racecar/drive")
        self.declare_parameter("use_obs_avoid_topic", "use_obs_avoid")
        self.declare_parameter("laser_topic", "/scan")
        self.declare_parameter("obstacle_check_distance", 3.0)
        self.declare_parameter("obstacle_stop_distance", 1.0)
        self.declare_parameter("obstacle_detection_threshold", 0.5)
        self.declare_parameter("min_obstacle_points", 2)
        self.declare_parameter("raceline_pointcloud_topic", "raceline_pointcloud")
        
        traj_csv = self.get_parameter("trajectory_csv").value
        self.pp_steer_L_fast = self.get_parameter("pp_steer_L_fast").value
        self.pp_steer_L_slow = self.get_parameter("pp_steer_L_slow").value
        self.kp_fast = self.get_parameter("kp_fast").value
        self.kp_slow = self.get_parameter("kp_slow").value
        self.L_threshold_speed = self.get_parameter("L_threshold_speed").value
        odom_topic = self.get_parameter("odom_topic").value
        pure_pursuit_velocity_topic = self.get_parameter("pure_pursuit_velocity_topic").value
        drive_topic = self.get_parameter("drive_topic").value
        use_obs_avoid_topic = self.get_parameter("use_obs_avoid_topic").value
        laser_topic = self.get_parameter("laser_topic").value
        self.obstacle_check_distance = self.get_parameter("obstacle_check_distance").value
        self.obstacle_stop_distance = self.get_parameter("obstacle_stop_distance").value
        self.obstacle_detection_threshold = self.get_parameter("obstacle_detection_threshold").value
        self.min_obstacle_points = self.get_parameter("min_obstacle_points").value
        raceline_pointcloud_topic = self.get_parameter("raceline_pointcloud_topic").value

        self.spline_index_car = 0  # Index of the car on the spline

        # Load raceline data including all available columns (for state)
        self.raw_csv_data = self.load_raw_csv_data(traj_csv)
        self.pp_waypoints, self.drive_velocity = load_from_csv(traj_csv)
        self.pp_x_spline = self.pp_waypoints[:, 0]
        self.pp_y_spline = self.pp_waypoints[:, 1]
        self.pp_spline_points = np.vstack((self.pp_x_spline, self.pp_y_spline, np.zeros((len(self.pp_y_spline)))))

        #### Obstacle Avoidance ###
        self.use_obs_avoid = False
        self.obstacle_detected = False
        self.laser_data = None

        ### ROS PUB/SUB ###
        self.pose_subscriber = self.create_subscription(Odometry, odom_topic, self.pose_callback, 1)
        self.velocity_publisher = self.create_publisher(Float64, pure_pursuit_velocity_topic, 1)
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, drive_topic, 1)
        self.use_obs_avoid_subscriber = self.create_subscription(Bool, use_obs_avoid_topic, self.use_obs_avoid_callback, 1)
        self.laser_subscriber = self.create_subscription(LaserScan, laser_topic, self.laser_callback, 1)
        self.raceline_pointcloud_publisher = self.create_publisher(PointCloud2, raceline_pointcloud_topic, 10)
        
        # Publish pointcloud initially (2 times)
        self.create_and_publish_pointcloud()
        self.get_logger().info("Published raceline pointcloud (1st time)")
        
        # Store the timer reference so we can destroy it later
        self.second_publish_timer = self.create_timer(1.0, self.publish_second_pointcloud)

    def publish_second_pointcloud(self):
        self.create_and_publish_pointcloud()
        self.get_logger().info("Published raceline pointcloud (2nd time)")
        # Destroy the timer to prevent further callbacks
        self.destroy_timer(self.second_publish_timer)
        # Clear the reference
        self.second_publish_timer = None

    def load_raw_csv_data(self, traj_csv):
        """Load the raw CSV data to get all columns including state if available"""
        with open(traj_csv, 'r') as f:
            lines = (line for line in f if not line.startswith('#'))
            data = np.loadtxt(lines, delimiter=',')
        return data
    
    def create_and_publish_pointcloud(self):
        """Create and publish the raceline as a PointCloud2 message"""
        # Determine the fields based on the data shape
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='velocity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Add state field if available (4th column)
        if self.raw_csv_data.shape[1] >= 4:
            fields.append(PointField(name='state', offset=16, datatype=PointField.FLOAT32, count=1))
            point_step = 20  # 5 fields * 4 bytes
        else:
            point_step = 16  # 4 fields * 4 bytes
            
        # Create point cloud data
        cloud_data = []
        for i in range(len(self.pp_waypoints)):
            # X, Y coordinates
            cloud_data.extend([self.pp_waypoints[i, 0], self.pp_waypoints[i, 1], 0.0])
            
            # Velocity
            cloud_data.append(self.drive_velocity[i])
            
            # State if available
            if self.raw_csv_data.shape[1] >= 4:
                cloud_data.append(self.raw_csv_data[i, 3])
                
        # Convert to bytes
        cloud_data = np.array(cloud_data, dtype=np.float32).tobytes()
        
        # Create PointCloud2 message
        cloud_msg = PointCloud2()
        cloud_msg.header.stamp = self.get_clock().now().to_msg()
        cloud_msg.header.frame_id = "map"
        cloud_msg.fields = fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = point_step
        cloud_msg.row_step = point_step * len(self.pp_waypoints)
        cloud_msg.data = cloud_data
        cloud_msg.height = 1
        cloud_msg.width = len(self.pp_waypoints)
        cloud_msg.is_dense = True
        
        # Publish the message
        self.raceline_pointcloud_publisher.publish(cloud_msg)
    
    def laser_callback(self, laser_msg):
        """Store laser scan data for obstacle detection"""
        self.laser_data = laser_msg

    def check_obstacle_on_trajectory(self, current_position, current_quat):
        """
        Check for obstacles 5 meters ahead on the trajectory
        Returns True if obstacle detected, False otherwise
        """
        if self.laser_data is None:
            return False
        
        # Get trajectory points ahead of current position
        trajectory_ahead = self.get_trajectory_points_ahead(current_position, self.obstacle_check_distance)
        
        if len(trajectory_ahead) == 0:
            return False
        
        # Convert trajectory points to laser scan coordinate system
        obstacle_points_count = 0
        
        for point in trajectory_ahead:
            # Transform global trajectory point to car frame
            point_global = np.array([point[0], point[1], 0])
            point_car = self.global_2_local(current_quat, current_position, point_global)
            
            # Convert to polar coordinates for laser scan comparison
            distance = np.sqrt(point_car[0]**2 + point_car[1]**2)
            angle = np.arctan2(point_car[1], point_car[0])
            
            # Check if this trajectory point has an obstacle nearby in laser scan
            if self.is_obstacle_at_point(distance, angle):
                obstacle_points_count += 1
        
        # Print obstacle count to terminal
        print(f"Akadály pontok száma: {obstacle_points_count}")
        
        # Only consider it an obstacle if we have enough points
        if obstacle_points_count < self.min_obstacle_points:
            print(f"Kevesebb mint {self.min_obstacle_points} pont - figyelmen kívül hagyva")
            return False
        
        return True

    def get_trajectory_points_ahead(self, current_position, check_distance):
        """Get trajectory points within check_distance ahead of current position"""
        current_pos = np.array([current_position.x, current_position.y])
        
        # Find points ahead on trajectory
        points_ahead = []
        cumulative_distance = 0
        
        for i in range(self.spline_index_car, len(self.pp_waypoints)):
            if i == self.spline_index_car:
                continue
                
            prev_point = self.pp_waypoints[i-1, :2]
            curr_point = self.pp_waypoints[i, :2]
            segment_distance = np.linalg.norm(curr_point - prev_point)
            cumulative_distance += segment_distance
            
            if cumulative_distance <= check_distance:
                points_ahead.append(curr_point)
            else:
                break
        
        return np.array(points_ahead)

    def is_obstacle_at_point(self, distance, angle):
        """Check if there's an obstacle at given distance and angle using laser scan"""
        if self.laser_data is None:
            return False
        
        # Convert angle to laser scan index
        angle_min = self.laser_data.angle_min
        angle_max = self.laser_data.angle_max
        angle_increment = self.laser_data.angle_increment
        
        if angle < angle_min or angle > angle_max:
            return False
        
        laser_index = int((angle - angle_min) / angle_increment)
        laser_index = max(0, min(laser_index, len(self.laser_data.ranges) - 1))
        
        # Check nearby laser points for obstacles
        search_range = 5  # Check nearby laser points
        for i in range(max(0, laser_index - search_range), 
                      min(len(self.laser_data.ranges), laser_index + search_range + 1)):
            laser_distance = self.laser_data.ranges[i]
            
            # Skip invalid readings
            if laser_distance < self.laser_data.range_min or laser_distance > self.laser_data.range_max:
                continue
            
            # Check if laser reading indicates obstacle near trajectory point
            if abs(laser_distance - distance) < self.obstacle_detection_threshold:
                return True
        
        return False

    def pose_callback(self, pose_msg):
        """
        This is the main pure pursuit callback loop
        All parameters are set in the init function
        """
        # parse information from pose_msg
        current_position = pose_msg.pose.pose.position
        current_quat = pose_msg.pose.pose.orientation

        # get the current spline index of the car and goal point
        self.spline_index_car = self.get_closest_point_to_car(current_position, self.pp_spline_points)

        # Check for obstacles on trajectory
        self.obstacle_detected = self.check_obstacle_on_trajectory(current_position, current_quat)

        #Determine the speed from the velocity profile
        drive_speed = self.drive_velocity[self.spline_index_car]
        
        # Stop if obstacle detected
        if self.obstacle_detected:
            drive_speed = 0.0
            self.get_logger().warn("Obstacle detected ahead! Stopping vehicle.")
        
        msg = Float64()
        msg.data = drive_speed
        self.velocity_publisher.publish(msg)

        # Calculate goal point
        if drive_speed > self.L_threshold_speed:
            global_goal_point = self.find_goal_point(self.pp_steer_L_fast)
        else:
            global_goal_point = self.find_goal_point(self.pp_steer_L_slow)
        local_goal_point = self.global_2_local(current_quat, current_position, global_goal_point)
        
        # Calculate steer angle
        if drive_speed > self.L_threshold_speed:
            steering_angle = self.calc_steer(local_goal_point, self.kp_fast, self.pp_steer_L_fast)
        else:
            steering_angle = self.calc_steer(local_goal_point, self.kp_slow, self.pp_steer_L_slow)        
        
        if not self.use_obs_avoid:
            msg = AckermannDriveStamped()
            msg.drive.steering_angle = float(steering_angle)
            msg.drive.speed = float(drive_speed)
            self.drive_publisher.publish(msg)

    def calc_steer(self, goal_point_car, kp, L):
        """
        Returns the steering angle from the local goal point
        """
        y = goal_point_car[1]
        steer_dir = np.sign(y)
        r = L ** 2 / (2 * np.abs(y))
        gamma = 1 / r
        steering_angle = (gamma * kp * steer_dir)
        return steering_angle
  
    def use_obs_avoid_callback(self, avoid_msg):
        self.use_obs_avoid = avoid_msg.data

    def global_2_local(self, current_quat, current_position, goal_point_global):
        # Construct transformation matrix from rotation matrix and position
        H_global2car = np.zeros([4, 4]) #rigid body transformation from  the global frame of referce to the car
        H_global2car[3, 3] = 1
        current_rotation_matrix = R.from_quat(np.array([current_quat.x,current_quat.y,current_quat.z,current_quat.w])).as_matrix()
        H_global2car[0:3, 0:3] = np.array(current_rotation_matrix)
        H_global2car[0:3, 3] = np.array([current_position.x, current_position.y, current_position.z])

        # Calculate point
        goal_point_global = np.append(goal_point_global, 1).reshape(4, 1)
        goal_point_car = np.linalg.inv(H_global2car) @ goal_point_global

        return goal_point_car
    
    def get_closest_point_to_car(self, current_position, all_points):
        try:
            current_position=np.array([current_position.x, current_position.y, current_position.z])
        except:
            current_position=np.array([current_position[0], current_position[1], current_position[2]])
        current_position=np.transpose(np.multiply(current_position,np.transpose(np.ones((all_points.shape)))))

        dist = np.linalg.norm(current_position - all_points, axis=0)

        point_index = np.argmin(dist)
        return point_index

    
    def find_goal_point(self, L):
        # Returns the global x,y,z position of the goal point
        points_in_front = np.roll(self.pp_spline_points, - self.spline_index_car, axis=1)
        points_dist = np.linalg.norm(np.roll(points_in_front, 1, axis=1) - points_in_front, axis=0)
        points_dist = np.cumsum(points_dist)
        idx = np.argmin(np.abs(points_dist - L))
        goal_point_car = points_in_front[:, idx]
        return goal_point_car


def load_from_csv(traj_csv, TUM=False, scaler=1):
    # Open csv and read the waypoint data
    points = None
    velocity = None

    with open(traj_csv, 'r') as f:
        lines = (line for line in f if not line.startswith('#'))
        data = np.loadtxt(lines, delimiter=',')
    
    # Handle different CSV formats
    if data.shape[1] >= 4:  # t3.csv format: x, y, velocity, state
        points = data[:, 0:2] 
        velocity = data[:, 2]
    else:  # levine.csv format: x, y, velocity
        points = data[:, 0:2] 
        velocity = data[:, 2]

    return points, velocity

def main(args=None):
    rclpy.init(args=args)
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)
    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()