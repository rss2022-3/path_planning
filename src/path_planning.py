#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray, Point
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
from utils import LineTrajectory
import tf
from utilities.planning import PRM, BiRRT

class PathPlan(object):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)

        self.H_map_BL_start = None
        self.H_map_BL_goal = None
        self.H_map_occupancy_grid = None
        self.resolution = None

        self.planner = None

    def map_cb(self, msg):
        self.resolution = msg.info.resolution

        roll, pitch, yaw = tf.transformations.euler_from_quaternion([msg.info.origin.orientation.x,
                                                                     msg.info.origin.orientation.y,
                                                                     msg.info.origin.orientation.z,
                                                                     msg.info.origin.orientation.w])

        x, y = msg.info.origin.position.x, msg.info.origin.position.y

        self.H_map_occupancy_grid = np.array([[np.cos(yaw), -np.sin(yaw), x],
                                              [np.sin(yaw),  np.cos(yaw), y],
                                              [          0,            0, 1]]) 

        ### INITILIZE PRM ###
        #collision_step_size = 1
        #seed = 17
        #K = 5
        #self.planner = PRM(msg, collision_step_size, seed, K)
        #self.planner.GrowRoadMap(5000)
        #print("Roadmap is grown")

        collision_step_size = 1
        K = 5
        self.planner = BiRRT(msg, collision_step_size, 17, 4, 10)



    def odom_cb(self, msg):
        roll, pitch, yaw = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,
                                                                     msg.pose.pose.orientation.y,
                                                                     msg.pose.pose.orientation.z,
                                                                     msg.pose.pose.orientation.w])
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y

        self.H_map_BL_start = np.array([[np.cos(yaw), -np.sin(yaw), x],
                                        [np.sin(yaw),  np.cos(yaw), y],
                                        [          0,            0, 1]]) 


    


    def goal_cb(self, msg):
        roll, pitch, yaw = tf.transformations.euler_from_quaternion([msg.pose.orientation.x,
                                                                     msg.pose.orientation.y,
                                                                     msg.pose.orientation.z,
                                                                     msg.pose.orientation.w])
        x, y = msg.pose.position.x, msg.pose.position.y

        self.H_map_BL_goal  = np.array([[np.cos(yaw), -np.sin(yaw), x],
                                        [np.sin(yaw),  np.cos(yaw), y],
                                        [          0,            0, 1]]) 


        self.plan_path()
        
    def plan_path(self):
        self.trajectory.clear()
        self.H_occupancy_grid_BL_start = np.linalg.inv(self.H_map_occupancy_grid).dot(self.H_map_BL_start)
        self.H_occupancy_grid_BL_goal = np.linalg.inv(self.H_map_occupancy_grid).dot(self.H_map_BL_goal)

        x_start, y_start = self.H_occupancy_grid_BL_start[0][-1],  self.H_occupancy_grid_BL_start[1][-1]
        x_goal, y_goal = self.H_occupancy_grid_BL_goal[0][-1],  self.H_occupancy_grid_BL_goal[1][-1]

        u_start, v_start = x_start/self.resolution, y_start/self.resolution
        u_goal, v_goal = x_goal/self.resolution, y_goal/self.resolution

        sequence = [np.array([u_start, v_start]), np.array([u_goal, v_goal])]
        output = self.planner.getPath(sequence, path_processing=self.planner.shortcut)
        if output is None:
            print("NO PATH FOUND")
            return None
        path, runtime = output

        path = map(lambda knot: np.array([[1, 0, knot[0]*self.resolution],
                                          [0, 1, knot[1]*self.resolution], 
                                          [0, 1,      1]]), path)
        path = map(lambda H_occupancy_grid_pt: self.H_map_occupancy_grid.dot(H_occupancy_grid_pt), path)
        path = map(lambda H: (H[0][-1], H[1][-1]), path)
        traj = list(map(lambda x: Point(x[0], x[1], 0), path))
        for pt in traj:
            self.trajectory.addPoint(pt)

        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz(duration = 200.0)


if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
