
#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
import math
import tf

class OccupancyGridMap:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("occupancy_grid_map")

        # 1. Load parameters from params.yaml
        self.scan_topic = rospy.get_param('~scan_topic')      # LiDAR scan topic
        self.odom_topic = rospy.get_param('~odom_topic')      # Wheel odometry topic
        self.map_topic = rospy.get_param('~occ_map_topic')    # Occupancy grid map topic

        # Occupancy grid map parameters (resolution, dimensions)
        self.resolution = rospy.get_param('~map_res')         # Grid cell size [meters]
        self.width = rospy.get_param('~map_width')            # Map width [cells]
        self.height = rospy.get_param('~map_height')          # Map height [cells]

        # Occupancy probabilities
        self.p_occ = rospy.get_param('~p_occ')                # Probability cell occupied
        self.p_free = rospy.get_param('~p_free')              # Probability cell free

        # Reference Frame
        self.grid_frame = rospy.get_param('~odom_frame')

        # 2. Subscribe to odometry and LiDAR scan topics
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        rospy.Subscriber(self.scan_topic, LaserScan, self.lidar_callback)

        # Occupancy Grid Publisher
        self.map_pub = rospy.Publisher(self.map_topic, OccupancyGrid, queue_size=10)

        # Initialize OccupancyGrid ROS message
        self.grid = OccupancyGrid()
        self.grid.header.frame_id = self.grid_frame
        self.grid.info.resolution = self.resolution
        self.grid.info.width = self.width
        self.grid.info.height = self.height

        # Set grid origin (bottom-left corner)
        self.grid.info.origin.position.x = -(self.width * self.resolution) / 2
        self.grid.info.origin.position.y = -(self.height * self.resolution) / 2
        self.grid.info.origin.orientation.w = 1

        # 3. Set all cells to Unknown initially (probability = 0.5)
        self.grid.data = [-1 for _ in range(self.width * self.height)]

        # To store current vehicle pose
        self.current_pose = None

    # 4. Odometry callback (retrieve vehicle pose)
    def odom_callback(self, odom_msg):
        orientation_q = odom_msg.pose.pose.orientation
        (_, _, yaw) = tf.transformations.euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,    
            orientation_q.w
        ])
        # Vehicle pose: x, y, yaw angle
        self.current_pose = (
            odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y,
            yaw
        )

    # 5. LiDAR callback (update occupancy grid using LiDAR data)
    def lidar_callback(self, scan_msg):
        if self.current_pose is None:
            return  # Do nothing if vehicle pose isn't available yet

        robot_x, robot_y, robot_yaw = self.current_pose

        # Iterate over each LiDAR ray
        for idx, distance in enumerate(scan_msg.ranges):
            angle = scan_msg.angle_min + idx * scan_msg.angle_increment

            if distance < scan_msg.range_min or distance > scan_msg.range_max:
                continue  # Skip invalid measurements

            # Compute global coordinates of obstacle detected by LiDAR
            obstacle_x = robot_x + distance * math.cos(robot_yaw + angle)
            obstacle_y = robot_y + distance * math.sin(robot_yaw + angle)

            # Convert global coordinates to grid indices
            cell_x = int((obstacle_x - self.grid.info.origin.position.x) / self.resolution)
            cell_y = int((obstacle_y - self.grid.info.origin.position.y) / self.resolution)

            # Mark the obstacle cell as occupied
            if 0 <= cell_x < self.width and 0 <= cell_y < self.height:
                obstacle_idx = cell_y * self.width + cell_x
                self.grid.data[obstacle_idx] = 100  # Occupied cell

            # Mark all cells along the LiDAR ray as free until reaching the obstacle
            steps_along_ray = int(distance / self.resolution)
            for step in range(steps_along_ray):
                free_x = robot_x + step * self.resolution * math.cos(robot_yaw + angle)
                free_y = robot_y + step * self.resolution * math.sin(robot_yaw + angle)
                free_cell_x = int((free_x - self.grid.info.origin.position.x) / self.resolution)
                free_cell_y = int((free_y - self.grid.info.origin.position.y) / self.resolution)

                if 0 <= free_cell_x < self.width and 0 <= free_cell_y < self.height:
                    free_idx = free_cell_y * self.width + free_cell_x
                    if self.grid.data[free_idx] != 100:  # Avoid overwriting occupied cells
                        self.grid.data[free_idx] = 0  # Free cell

        # Publish updated occupancy grid
        self.grid.header.stamp = rospy.Time.now()
        self.map_pub.publish(self.grid)

if __name__ == '__main__':
    try:
        OccupancyGridMap()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
