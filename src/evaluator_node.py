#!/usr/bin/env python

import rospy
import carla
import time

# might not be necessary
from carla_environment_bridge import filter_facing_ego
from common_msgs.msg import TrafficSignList, TrafficSign
from common_msgs.msg import TrackedObstacleList, TrackedObstacle, TrackedObstacleState, Obstacle
from carla_common.transforms import carla_transform_to_ros_pose, carla_velocity_to_ros_twist

from std_msgs.msg import Bool
from std_msgs.msg import Header

from common_msgs.msg import TrafficSignList as TrafficSignListMsg
from common_msgs.msg import TrafficSign as TrafficSignMsg
from common_msgs.msg import Obstacle as ObstacleMsg
from common_msgs.msg import ObstacleList as ObstacleListMsg
from common_msgs.msg import TrackedObstacle as TrackedObstacleMsg
from common_msgs.msg import TrackedObstacleList as TrackedObstacleListMsg
from common_msgs.msg import TrackedObstacleState as TrackedObstacleStateMsg


class EvaluatorNode:
    def __init__(self):
		# Publishers (publish to the object tracker)
		obstacles_topic = '/obstacles_3d'
		self.obstacles_sub = rospy.Publisher(
				obstacles_topic, ObstacleListMsg, queue_size=self.queue_size)

		traffic_sign_topic = '/traffic_signs_3d'
		self.obstacles_sub = rospy.Publisher(
				traffic_sign_topic, TrafficSignListMsg, queue_size=self.queue_size)

		lidar_obstacles_topic = '/object_detection'  # non classified objects from euclidean clustering
		self.lidar_obstacles_sub = rospy.Publisher(
				lidar_obstacles_topic, ObstacleListMsg, queue_size=self.queue_size)

        # Subscribers (subscribe to the object tracker)
		tracked_obstacles_topic = '/tracked_obstacles'
		self.tracked_obstacles_publisher = rospy.Subscriber(
				tracked_obstacles_topic, TrackedObstacleListMsg, self.tracked_obstacles_callback, queue_size=self.queue_size)

		tracked_signs_topic = '/tracked_signs'
		self.tracked_signs_publisher = rospy.Subscriber(
				tracked_signs_topic, TrafficSignListMsg, self.traffic_signs_callback, queue_size=self.queue_size)
    
    def nuscene_to_kitti(self):

    def tracked_obstacles_callback(self, tracked_obstacles_msg):
        start_time = time.time()

        timestamp = tracked_obstacles_msg.header.stamp
		frame_id = tracked_obstacles_msg.header.frame_id

    
    def traffic_signs_callback(self, traffic_signs_msg):
        start_time = time.time()

        timestamp = traffic_signs_msg.header.stamp
		frame_id = traffic_signs_msg.header.frame_id

    def publish_data(self):

    def create_obstacle_list_msg(self):
        msg = ObstacleListMsg()
        msg.header.frame_id = self.reference_frame

    def create_traffic_sign_list_msg(self):
        msg = TrafficSignListMsg()
        msg.header.frame_id = self.reference_frame

    def create_lidar_obstacles_msg(self):
        msg = ObstacleListMsg()
        msg.header.frame_id = self.reference_frame


def main():

if __name__ == "__main__":
	main()
