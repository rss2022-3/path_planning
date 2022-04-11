#!/usr/bin/env python

import rospy
import numpy as np
import time
import utils
import tf
from utilities.controllers import PurePursuit as OurPurePursuit
from utilities.Trajectory import LinearTrajectory

from geometry_msgs.msg import PoseArray, PoseStamped
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        self.odom_topic       = rospy.get_param("~odom_topic")
       # self.lookahead        = 2
        self.speed            = 2
       # self.wrap             = # FILL IN #
        self.wheelbase_length = 0.325
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=1)
        self.pursuit = OurPurePursuit(self.wheelbase_length)

        
    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        print ("Receiving new trajectory:", len(msg.poses), "points")
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)

    def odom_cb(self, msg):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y 
        roll, pitch, yaw = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,
                                                                     msg.pose.pose.orientation.y,
                                                                     msg.pose.pose.orientation.z,
                                                                     msg.pose.pose.orientation.w])
        self.pursuit.updatePos(x, y, yaw)



        if self.trajectory.empty():
            self.drive(0,0, None, None)
            return

        #Converting PoseArray message into a numpy array so we can use our own pure pursuit class
        knots = [np.array([self.trajectory.points[0][0], self.trajectory.points[0][1]])]
        t_breaks = [0]
        for pt in self.trajectory.points[1:]:
            knots.append(np.array([pt[0], pt[1]]))
            t_breaks.append(np.linalg.norm(knots[-1] - knots[-2])/self.speed + t_breaks[-1] + 1e-3)
        knots = np.array(knots)
        t_breaks = np.stack(t_breaks)

        traj = LinearTrajectory(t_breaks, knots)
        steer, speed = self.pursuit.adaptiveControl(traj, self.v_function)


        # TODO the speed computed is in the relative reference frame, thus it has the wrong signs
        drive_cmd = self.drive(steer, abs(speed), None, None)

        self.drive_pub.publish(drive_cmd)
        self.trajectory.publish_viz(duration=5.0)


    def drive(self, theta, speed, theta_dot = None,  acceleration = None):
        """
            Takes in steering and speed commands for the car.
            :param theta: steering angle [rad]. right sided
            :type theta: float
            :param theta_dot: steering angular velocity [rad/s]
            :type theta_dot: float
            :param speed: speed in [m/s]
            :type speed: float

            :param speed: speed in [m/s]
            :type speed: float
        """
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = rospy.get_rostime()
        ack_msg.header.frame_id = 'base_link'
        ack_msg.drive.steering_angle = np.clip(theta, -0.34, 0.34)
        ack_msg.drive.speed = speed
        if not theta_dot is None:
            ack_msg.drive.steering_angle_velocity = theta_dot
        else:
            ack_msg.drive.steering_angle_velocity = 0.1
        if not acceleration is None:
            ack_msg.drive.acceleration = acceleration
        return ack_msg

    def v_function(self, v_desired, traj): #EDIT THIS ITS COPY PASTED FROM PARKING CONTROLLER
        #adaptive velocity function
        Lfw, lfw = 0, 0
        v_desired = abs(v_desired)
        if v_desired < 2:
            Lfw = v_desired
        elif v_desired >=2 and v_desired < 6:
            Lfw = 1.5*v_desired
        else:
            Lfw = 12
        return Lfw, lfw

if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
