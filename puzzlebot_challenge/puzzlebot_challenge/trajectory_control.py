import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32, Bool
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
from puzzlebot_msgs.msg import Arucoinfo, ArucoArray, LandmarkList, Landmark
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
from tf2_geometry_msgs import do_transform_point
import enum

class StateMachine(enum.Enum):
    FIND_LANDMARK = 1
    WANDER = 2
    GO_TO_TARGET = 3
    HANDLE_OBJECT = 4
    STOP = 5


class TrajectoryControl(Node):
    def __init__(self):
        super().__init__('Trajectory_Control')

        self.odometry_sub = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/filtered_scan', self.lidar_callback, 10)
        self.aruco_sub = self.create_subscription(ArucoArray, '/aruco_info', self.aruco_callback, 10)

        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.goal_pub = self.create_publisher( PoseStamped, '/goal', 1)
        self.landmarks_pub = self.create_publisher( LandmarkList, '/landmarks', 1 )
        self.bug_pub = self.create_publisher( Bool, '/bug2_run', 1)
        
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = "world"
        self.current_pose.pose.position.x = 0.0
        self.current_pose.pose.position.y = 0.0
        self.current_pose.pose.orientation.x = 0.0
        self.current_pose.pose.orientation.y = 0.0
        self.current_pose.pose.orientation.z = 0.0
        self.current_pose.pose.orientation.w = 1.0
        self.current_angle = 0.0


        self.current_state = StateMachine.WANDER
        
        self.discharge_area = PoseStamped()
        self.discharge_area.header.frame_id = "world"
        self.discharge_area.pose.position.x = 0.0
        self.discharge_area.pose.position.y = 0.0

        self.goal = PoseStamped()
        self.goal.header.frame_id = "world"
        self.goal.pose.position.x = 2.0
        self.goal.pose.position.y = 2.0

        self.cmd_vel = None
        self.landmarks_ids = {'2': False, '6': False, '7': False, '8': False}
        self.landmarks = LandmarkList()
        self.cube_id = '2'


        self.aruco_info = ArucoArray()
        self.aruco_info.length = 0
        self.aruco_info.aruco_array = []

        self.target_area_id = None
        self.distances = []  # Lista de la distancia a los puntos devueltos por el lidar
        self.distance_covered = 0.0  # Distancia cubierta por el robot

        
        # Timer para actualizar la pose
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.run)


    def odometry_callback(self, msg: Odometry):
        self.current_pose.pose = msg.pose.pose
        orientation_euler = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        self.current_angle = orientation_euler[2]

    def aruco_callback(self, msg):
        self.aruco_info = msg
        self.landmarks.landmarks = []
        for aruco in self.aruco_info.aruco_array:
            if aruco.id in self.landmarks_ids.keys():
                print(f'aruco id: {aruco.id}, Dictionary keys: {self.landmarks_ids.keys()}')
                if self.landmarks_ids[aruco.id] == False:
                    landmark = Landmark()
                    landmark.id = aruco.id
                    landmark.x, landmark.y = self.transform_cube_position(aruco.point.point)
                    self.landmarks.landmarks.append(landmark)
                    self.landmarks_ids[aruco.id] = True
        self.landmarks_ids = {key: False for key in self.landmarks_ids}
        
        self.landmarks_pub.publish(self.landmarks)
        

    
    def lidar_callback(self, msg):
        # Update the distances to obstacles based on laser scan data
        self.distances = np.array(msg.ranges)
    

    def transform_cube_position(self, aruco_point):

        rotation_matrix = np.array([
                    [np.cos(self.current_angle), -np.sin(self.current_angle)],
                    [np.sin(self.current_angle), np.cos(self.current_angle)]
                ])

        # Camera coordinates to robot coordinates
        puzzlebot_coords = np.array([aruco_point.z, aruco_point.x])
        world_coords = rotation_matrix.dot(puzzlebot_coords)

        # Robot coordinates to odometry coordinates
        aruco_x = self.current_pose.pose.position.x + world_coords[0]
        aruco_y = self.current_pose.pose.position.y + world_coords[1]

        return aruco_x, aruco_y


    def run(self):
        
        # self.get_logger().info(f'Current State: {self.current_state}')
        
        if self.current_state is StateMachine.FIND_LANDMARK:
            if self.landmarks: 
               self.current_state = StateMachine.WANDER

        elif self.current_state is StateMachine.WANDER:
            if self.aruco_info.length > 0:
                if self.aruco_info.aruco_array[0].id == self.cube_id:
                    self.goal.pose.position.x, self.goal.pose.position.y = self.transform_cube_position(self.aruco_info.aruco_array[0].point.point)
                    self.goal_pub.publish(self.goal)
                    self.current_state = StateMachine.GO_TO_TARGET
            else:
                print("Wandering, TODO node")


        elif self.current_state is StateMachine.GO_TO_TARGET:
            if self.aruco_info.aruco_array[0].point.point.z < 0.15:
                self.current_state = StateMachine.HANDLE_OBJECT
            else:
                self.bug_pub.publish(Bool(data=True))
                # self.get_logger().info(f'VELOCITY: x={self.velocity_msg.linear}')
  

        # elif self.current_state is StateMachine.HANDLE_OBJECT:
        #     if self.handle.picked:
        #         self.current_state = StateMachine.GO_TO_TARGET
        #     elif self.handle.placed: 
        #         self.current_state = StateMachine.STOP
        #     else:
        #         self.handle.run()
        elif self.current_state is StateMachine.STOP:
            stop = Twist()
            stop.linear.x = 0.0
            stop.angular.z = 0.0
            self.velocity_pub.publish(stop)
        
        self.there_is_aruco = False
        
def main(args=None):
    rclpy.init(args=args)
    slam_node = TrajectoryControl()
    rclpy.spin(slam_node)
    slam_node.plot_map()
    slam_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
