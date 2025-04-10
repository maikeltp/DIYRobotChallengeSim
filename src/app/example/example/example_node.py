#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import struct

# Constants
TARGET_DISTANCE = 1.0  # Target distance from obstacle in meters
SPEED = 1.0  # Speed in m/s

class ExampleNode(Node):
    def __init__(self):
        super().__init__('Example_node')
        
        # Create subscribers
        self.lidar_subscription = self.create_subscription(
            PointCloud2, 
            '/lidar/points', 
            self.lidar_callback, 
            10
        )
        
        # Create publisher for Twist messages
        self.twist_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.get_logger().info('Example node initialized')
        
        
    def lidar_callback(self, msg):
        # Process LiDAR data to find distance to the obstacle in the front
        front_obstacle_distance = self.process_lidar_data(msg)
        
        if front_obstacle_distance <= TARGET_DISTANCE:
            self.get_logger().warn(f'Front obstacle at less than {TARGET_DISTANCE} m')
            self.stop_vehicle()
        else:
            self.move()
            
    def process_lidar_data(self, cloud_msg):
        """
        Process LiDAR PointCloud2 data to calculate distance to a front obstacle.
        Returns the distance to a front obstacle or None if not found.
        """
        # Get cloud data
        points = self.point_cloud2_to_array(cloud_msg)
        if len(points) == 0:
            return None
            
        # Find points on the front side (0 degrees from forward direction)
        # Assuming the car's forward direction is along the x-axis in the LiDAR frame
        # and right is along the positive y-axis
        front_side_points = []
        
        for point in points:
            # Access fields by name instead of indexing
            x = point['x']
            y = point['y']
            z = point['z']
            
            # Filter points on the right side (roughly within -10 to 10 degrees from forward)
            if y > -0.5 and y < 0.5 and x > 0:  # Only consider points on the front side
                distance = math.sqrt(x*x + y*y)
                front_side_points.append((x, y, distance))
                    
        if not front_side_points:
            return None
            
        # Find the closest point
        front_side_points.sort(key=lambda p: p[2])
        closest_points = front_side_points[:10]  # Use the 10 closest points
        
        # Average the distance of the closest points
        avg_distance = sum(p[2] for p in closest_points) / len(closest_points)
        
        return avg_distance
    
    def point_cloud2_to_array(self, cloud_msg):
        """Convert a PointCloud2 message to a numpy array."""
        # Get the number of points
        num_points = cloud_msg.width * cloud_msg.height
        
        # Define the dtype based on the point fields
        dtype_list = []
        for field in cloud_msg.fields:
            if field.name in ['x', 'y', 'z']:
                dtype_list.append((field.name, np.float32))
        
        # Create a structured array with the appropriate dtype
        cloud_array = np.zeros(num_points, dtype=dtype_list)
        
        # Extract the data from the point cloud
        points_data = np.frombuffer(cloud_msg.data, dtype=np.uint8).reshape(num_points, -1)
        
        # The step size (in bytes) between consecutive points in the data
        point_step = cloud_msg.point_step
        
        # Extract x, y, z coordinates
        for i in range(num_points):
            start_idx = i * point_step
            for field in cloud_msg.fields:
                if field.name in ['x', 'y', 'z']:
                    # Extract the bytes for this field
                    field_data = points_data[i, field.offset:field.offset + 4]
                    # Convert bytes to float
                    value = struct.unpack('f', field_data.tobytes())[0]
                    cloud_array[i][field.name] = value
        
        return cloud_array
    
    def create_twist_message(self, steering_angle, speed):
        """
        Convert steering angle and speed to a Twist message.
        
        In a car-like robot (with Ackermann steering):
        - Linear velocity corresponds to the forward speed
        - Angular velocity (yaw rate) can be approximated as: speed * tan(steering_angle) / wheelbase
        
        We'll use a default wheelbase of 0.2 meters.
        Adjust this value based on your actual robot dimensions.
        """
        wheelbase = 0.2  # in meters, adjust based on your robot's dimensions
        
        twist_msg = Twist()
        
        # Linear velocity components
        twist_msg.linear.x = speed  # Forward velocity
        twist_msg.linear.y = 0.0    # No lateral velocity for car-like robot
        twist_msg.linear.z = 0.0    # No vertical velocity
        
        # Angular velocity components (in rad/s)
        # For a car, the yaw rate depends on speed, steering angle, and wheelbase
        if abs(speed) > 0.001:  # Avoid division by zero or very small speeds
            twist_msg.angular.z = speed * math.tan(steering_angle) / wheelbase
        else:
            twist_msg.angular.z = 0.0
            
        twist_msg.angular.x = 0.0  # No roll rate
        twist_msg.angular.y = 0.0  # No pitch rate
        
        return twist_msg
    
    def move(self):
        """
        Move the car at a constant velocity and 0 deg steering angle
        """
        
        # Create and publish Twist message
        twist_msg = self.create_twist_message(0, SPEED)
        self.twist_publisher.publish(twist_msg)
            
    
    def stop_vehicle(self):
        """Stop the vehicle"""
        
        # Create and publish Twist stop command
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        
        self.twist_publisher.publish(twist_msg)
        

def main(args=None):
    rclpy.init(args=args)
    example_node = ExampleNode()
    
    try:
        rclpy.spin(example_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the vehicle before shutting down
        example_node.stop_vehicle()
        example_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 