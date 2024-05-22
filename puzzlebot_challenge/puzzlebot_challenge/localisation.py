#!/usr/bin/env python
import rclpy
import math
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion, Pose
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf_transformations import quaternion_from_euler
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data

class Localisation(Node):
    def __init__(self):
        super().__init__('Odometry')

        # Initialize wheel variables
        self.wr = 0.0               # Right Wheel
        self.wl = 0.0               # Left Wheel
        self.linear_speed = 0.0     # Linear Speed
        self.angular_speed = 0.0    # Angular Speed
        self.l = 0.19               # Wheelbase
        self.r = 0.05               # Radius of the Wheel
        
        # Constants for error model
        self.kr = 0.00001  # Error coefficient for the right wheel
        self.kl = 0.00001  # Error coefficient for the left wheel

        # Starting pose for the puzzlebot
        self.angle = 0.0
        self.positionx = 0.0
        self.positiony = 0.0

        
        # Subscribers
        self.sub_wl = self.create_subscription(Float32, '/VelocityEncL', self.cbWl, qos_profile_sensor_data)
        self.sub_wr = self.create_subscription(Float32, '/VelocityEncR', self.cbWr, qos_profile_sensor_data)

        # Publishers 
        self.odom_pub = self.create_publisher(Odometry, 'odom', 1)

        # Start the timer now
        self.start_time = self.get_clock().now()
        time_period = 0.1
        self.timer = self.create_timer(time_period, self.odom_reading)



    def cbWr(self, msg):
        self.wr = msg.data

    def cbWl(self, msg):
        self.wl = msg.data

    def odom_reading(self):

        #Get time difference 
        self.current_time = self.start_time.to_msg()
        self.duration = self.get_clock().now() - self.start_time

        # Convert the duration to a float value (in seconds)
        self.dt = self.duration.nanoseconds * 1e-9
        
        self.linear_speed = self.r * (self.wr + self.wl) / 2.
        self.angular_speed = self.r * (self.wr - self.wl) / self.l

        self.angle += self.angular_speed * self.dt
        self.positionx += self.linear_speed * np.cos(self.angle) * self.dt
        self.positiony += self.linear_speed * np.sin(self.angle) * self.dt

        # Define Jacobian matrix H_k
        H_k = np.array([
            [1, 0, -self.dt * self.linear_speed * np.sin(self.angle)],
            [0, 1, self.dt * self.linear_speed * np.cos(self.angle)],
            [0, 0, 1]
        ])

        # Define the error matrix Q_k
        Q_k = np.diag([self.kr * abs(self.wr) * self.dt, 
                       self.kl * abs(self.wl) * self.dt, 
                       (self.kr * abs(self.wr) + self.kl * abs(self.wl)) * self.dt])

        # Update covariance matrix using the previous covariance matrix
        if not hasattr(self, 'sigma'):
            self.sigma = np.eye(3)  # Initializes the covariance matrix if it hasn't been defined

        # Predict the new covariance matrix
        self.sigma = H_k.dot(self.sigma).dot(H_k.T) + Q_k

        # self.get_logger().info(f"Covariance Matrix:\n{self.sigma}")
        # self.get_logger().info("Matrix Q_k: {}".format(Q_k))

        # Extend 3x3 matrix to 6x6 for ROS compatibility
        sigma_full = np.zeros((6, 6))
        sigma_full[:3, :3] = self.sigma  # Fill in the 3x3 position covariance
        sigma_full[3, 3] = 0.001  # Small value for orientation around x (roll)
        sigma_full[4, 4] = 0.001  # Small value for orientation around y (pitch)
        sigma_full[5, 5] = 0.001  # Small value for orientation around z (yaw)

        # Publish odometry via odom topic
        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"  # Set the frame id to "odom"
        odom.child_frame_id = "base_link"  # Set the child frame id to "base_link"
        odom.pose.pose.position.x = self.positionx
        odom.pose.pose.position.y = self.positiony  
        q = quaternion_from_euler(0., 0., self.angle)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.pose.covariance = sigma_full.flatten().tolist()  # Set the pose covariance matrix as a list
        odom.twist.twist.linear.x = self.linear_speed
        odom.twist.twist.angular.z = self.angular_speed
        self.odom_pub.publish(odom)
        # self.get_logger().info("Position y msg: {}".format(odom.pose.pose.position.y))
        self.start_time = self.get_clock().now()




def main(args=None):
    rclpy.init(args=args)
    odometry = Localisation()
    rclpy.spin(odometry)
    odometry.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()