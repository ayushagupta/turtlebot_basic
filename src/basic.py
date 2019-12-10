#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
import tf
from math import radians, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np

# Constants
THRESHOLD_DISTANCE = 0.03
THRESHOLD_ANGLE = 0.05
LINEAR_VELOCITY = 0.5
ANGULAR_VELOCITY = 0.5

class Robot():

    def __init__(self):
        rospy.init_node('basic', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 5) # Publish linear and angular velocity
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback) # Subscribe to the Lidar messages
        self.map_sub = rospy.Subscriber('map', OccupancyGrid, self.map_callback)
        self.rate = rospy.Rate(10) # Rate of loops will be 10Hz wherever this is used
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'
        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")
        
    # Move the robot at a particular linear velocity and angular velocity
    def move(self, linear_velocity, angular_velocity):
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        self.cmd_vel_pub.publish(twist)

    # Rotate the robot at a particular angular velocity
    # CCW is considered positive and CW as negative
    def rotate(self, angular_velocity):
        twist = Twist()
        twist.angular.z = angular_velocity
        self.cmd_vel_pub.publish(twist)

    # Rotate to make final angle=goal_z
    # Value should between -180 and 180 (inclusive)
    def rotateToAngle(self, goal_z):
        if goal_z > 180 or goal_z < -180:
            print("Wrong angle value.")
            self.shutdown()
        goal_z = np.deg2rad(goal_z)
        print(str(goal_z))
        (position, rotation) = self.get_odom()
        while abs(rotation - goal_z) > THRESHOLD_ANGLE:
            (position, rotation) = self.get_odom()
            if goal_z >= 0:
                if rotation <= goal_z and rotation >= goal_z-pi:
                    angular_velocity = ANGULAR_VELOCITY
                else:
                    angular_velocity = -ANGULAR_VELOCITY
            else:
                if rotation <= goal_z+pi and rotation > goal_z:
                    angular_velocity = -ANGULAR_VELOCITY
                else:
                    angular_velocity = ANGULAR_VELOCITY
            self.rotate(angular_velocity)
            self.rate.sleep()
        self.move(0,0)
    
    # Final position is (goal_x, goal_y) and final orientation is goal_z
    def goToPoint(self, goal_x, goal_y, goal_z):
        if goal_z > 180 or goal_z < -180:
            print("Wrong angle value.")
            self.shutdown()
        goal_z = np.deg2rad(goal_z)
        (position, rotation) = self.get_odom()
        distance = sqrt(pow(goal_x-position.x, 2) + pow(goal_y-position.y, 2))
        while distance > THRESHOLD_DISTANCE:
            (position, rotation) = self.get_odom()
            path_angle = atan2(goal_y-position.y, goal_x-position.x)
            # Continuously aligns the robot along the straight line path
            self.rotateToAngle(np.rad2deg(path_angle)) 
            distance = sqrt(pow(goal_x-position.x, 2) + pow(goal_y-position.y, 2))
            linear_velocity = min(LINEAR_VELOCITY, distance) #Proportional control
            self.move(linear_velocity,0)
            self.rate.sleep()
        # Rotate the robot to the final angle
        goal_z = np.rad2deg(goal_z)
        self.rotateToAngle(goal_z)
        self.move(0,0)

    # Gives the latest position and orientation of the robot
    def get_odom(self):
        try:
           (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
           rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
           rospy.loginfo("TF Exception")
           return
        position = Point(*trans)
        #print(position)
        #print("r = "+str(rotation[2]))
        return (position, rotation[2])

    # Gives the lidar distances for every angle from 1 to 360
    def get_scan(self):
        print(self.scan_sub.ranges)

    def scan_callback(self, scan_msg):
        print("Scan Received")

    def map_callback(self, map_msg):
        global map
        map = map_msg.data
        # print(map_msg.info.width)
        # print(map_msg.info.height)
        

    # Shutdown the robot
    def shutdown(self):
        self.move(0,0)
        rospy.sleep(1)


if __name__== "__main__":
    global map
    exit = False
    try:
        while not exit:
            robot = Robot()
            robot.goToPoint(-1,-1,0)
            exit = True
        print("Goal Reached")


    except:
        rospy.loginfo("Shutdown program.")