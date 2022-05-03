#!/usr/bin/env python

import rospy
import carla

from carla_environment_bridge import filter_facing_ego
from common_msgs.msg import TrafficSignList, TrafficSign
from common_msgs.msg import TrackedObstacleList, TrackedObstacle, TrackedObstacleState, Obstacle
from carla_common.transforms import carla_transform_to_ros_pose, carla_velocity_to_ros_twist

def main():
    rospy.init_node('carla_object_tracking_evaluator_bridge', log_level=rospy.DEBUG)
    role_name = rospy.get_param("/carla/ego_vehicle/role_name", "ego_vehicle")
    carla_host = rospy.get_param("/carla/host", "localhost")
    carla_port = rospy.get_param("/carla/port", 2000)
    carla_timeout = rospy.get_param("/carla/timeout", 2) # s

    client = carla.Client(carla_host, carla_port)
    client.set_timeout(carla_timeout)

    world = client.get_world()
    world.wait_for_tick()
    actors = world.get_actors()

    ego_vehicle = None
    while not rospy.is_shutdown() and ego_vehicle is None:
        rospy.sleep(0.5)
        rospy.logdebug_once("Looking for actor with role_name {}".format(role_name))
        for actor in world.get_actors().filter("*"):
            print(actor.attributes.get('role_name'))
            if actor.attributes.get('role_name') == role_name:
                ego_vehicle = actor
                break

    if rospy.is_shutdown():
        return


    obs_pub = rospy.Publisher('/tracked_obstacles', TrackedObstacleList, queue_size=10)
    tl_pub = rospy.Publisher('/tracked_signs', TrafficSignList, queue_size=10)
    obs_pub_rate = rospy.Rate(10) # Hz

    while not rospy.is_shutdown():



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
