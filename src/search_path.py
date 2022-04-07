#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray, Point, Pose
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
from utils import LineTrajectory
#import dubins
from Queue import Queue
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf

class PathPlan(object):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """
    def __init__(self):
      #self.odom_topic = rospy.get_param("~odom_topic")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.pose_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped,
                                          self.init_pose,
                                          queue_size=1)
        self.grid = None
        self.start = None
        self.end = None
        self.x_shift = None
        self.y_shift = None
       # self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)

    def map_cb(self, msg):
        self.orient = msg.info.origin.orientation
        self.pos = msg.info.origin.position
        self.x_shift = self.pos.x
        self.y_shift = self.pos.y
        self.res = msg.info.resolution
        self.grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        print("i the grid got a message",self.grid)
        rospy.loginfo((msg.info.height, msg.info.width))
        #TODO: Add dialation and maybe  shrink grid size  

    def init_pose(self, pose_msg):
      #Get initial particle
      self.prev_time = pose_msg.header.stamp

      position = pose_msg.pose.pose.position

      #print(position)
    
      if self.end is None and not self.start is None:
          self.end = self.coord_to_pix((position.x, position.y))
          self.plan_path(self.start, self.end, self.grid)

      if self.start is None:
          self.start = self.coord_to_pix((position.x, position.y))
      
     

    def odom_cb(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        #TODO:
    
    def goal_cb(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        #TODO:
    
    def plan_path(self, start_point, end_point, map):
        ## CODE FOR PATH PLANNING ##

        #refrenced https://www.redblobgames.com/pathfinding/a-star/implementation.html
        get_path, costs = self.a_star_alg(start_point, end_point)

        path = self.make_path(get_path, start_point, end_point)
        
        for i in path:
            point = Point()
            point.x = i[0]
            point.y = i[1]
            self.trajectory.addPoint(point)
        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz(10.0)
    
    def make_path(self, get_path, start, end):
        curr = end
        path = []
        while curr != start:
            path.append(curr)
            curr = get_path[curr]
        path.append(start)
        path.reverse()
        return path

    def a_star_alg(self, start, end):
        #TODO: start point may not be formatted correctly
        frontier = Queue()
        print('start', start)
        print('end', end)
        frontier.put(start, 0)
        came_from = {start:None}
        cost_so_far = {start:0}

        while not frontier.empty():
            #print("i do the A star")
            curr = frontier.get()
            
            if curr == end:
                break
           
            neighbors = self.get_neighbors(curr)
            #print('ethan neighbors', self.get_neighbors(curr))    
            for node in neighbors:#self.get_neighbors(curr):     
                new_cost = cost_so_far[curr] + 1
                if node not in cost_so_far or new_cost < cost_so_far[node]:
                    cost_so_far[node] = new_cost
                    priority = new_cost + self.heur(node, end)
                    frontier.put(node, priority)
                    came_from[node] = curr
                    
       # print("came from", came_from)
      #  print("distances", cost_so_far)
        return came_from, cost_so_far

    def pix_to_coord(self, pix):

        '''
        #x and y here may be flipped
        pix[0] = pix[0] * self.res
        pix[1] = pix[1] * self.res

        theta = tf.transformations.euler_from_quaternion((
            self.orient.x,
            self.orient.y,
            self.orient.z,
            self.orient.w))[2]

        x = np.matrix([np.cos(theta), -np.sin(theta), self.pos.x],[np.sin(theta), np.cos(theta), self.pos.y],[0,0,1])
        out = x.dot(np.matrix([pix[0], pix[1],0]).T)
        print("that matrix")
        return (out[0, 0], out[1,0])
        #how do we apply rotation
        '''
        x,y = pix
        res_x = self.pos.x - x*self.res
        res_y = self.pos.y - y*self.res
        return (res_x, res_y)

    def coord_to_pix(self, coord):
        '''
        theta = tf.transformations.euler_from_quaternion((
            self.orient.x,
            self.orient.y,
            self.orient.z,
            self.orient.w))[2]
        #x_inv= np.linalg.inv(np.matrix([[np.cos(theta), -np.sin(theta), self.pos.x],[np.sin(theta), np.cos(theta), self.pos.y],[0,0,1]]))
        out = x_inv.dot(np.matrix([coord.x, coord.y,0]).T)
        print("this matrix")
        return (-int((out[0]/self.res)[0,0]), -int((out[1]/self.res)[0,0]))
        '''
       # print('coord', coord)
       # print('curr_pos', self.pos.x, self.pos.y)
        x,y = coord
        res_x = int((x-self.pos.x)/self.res)
        res_y = int((y-self.pos.y)/self.res)
        return (res_x, res_y)



    def heur(self, node, end_point):
        #TODO change this to dubins
        dist = np.sqrt((node[0] - end_point[0])**2 + (node[1] - end_point[1])**2)
        return dist

    def get_neighbors(self, node):
        neighbors = []
        for i in range(3):
            for j in range(3):
                if not ((j == 1 and i == 1) or node[0] +1 -i < 0 or node[1] +1 -j < 0 or node[0] +1 -i >= self.grid.shape[0] or node[1] +1 -j >= self.grid.shape[1]):
                    if self.grid[node[0]+1-i,node[1]+1-j] == 0:
                        neighbors.append((node[0] + 1 - i, node[1] + 1 -j))
        return neighbors
        
if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
