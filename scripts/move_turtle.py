#!/usr/bin/env python

import rospy
import numpy as np
from numpy.linalg import inv
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from time import sleep
import matplotlib.pyplot as plt
import actionlib
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseActionResult 
from move_base_msgs.msg import MoveBaseActionFeedback

class Robot:
    def __init__(self):
        rospy.init_node("move_turtle_node")
        vel_topic = rospy.get_param("~cmd_vel_topic","/turtle1/cmd_vel")
        position_topic = rospy.get_param("~pose_topic","/turtle1/pose")
        self._action_name = rospy.get_param("~action_name","/move_turtle")
        self.robot_speed = rospy.get_param("~robot_max_speed", 1.5) #m/s
        self.pull_point = rospy.get_param("~pull_distance", 0.3) # m
        self.kp = rospy.get_param("~controller_gain", 10.0)
        self.plot_enable = rospy.get_param("~plot_enable", False)
        self.no_points = rospy.get_param("~path_number_of_points", 100)
        hz = rospy.get_param("~rate", 200)      
        self.position = [0.0, 0.0]
        self.orientation = 0.0
        self.__sub = rospy.Subscriber(position_topic, Pose, self.__callback )
        self.pub = rospy.Publisher(vel_topic, Twist, queue_size= 10)
        self.speed_msg = Twist()
        self.rate = rospy.Rate(hz)
        self._feedback = MoveBaseActionFeedback()
        self._result = MoveBaseActionResult()
        self._as = actionlib.SimpleActionServer(self._action_name,
                                    MoveBaseAction, 
                                    execute_cb=self.goTo,
                                    auto_start = False)
        self._as.start()
            
    def __callback(self,msg):
        self.position = [msg.x, msg.y]
        self.orientation = msg.theta
        
    def goTo(self, goal):
        rospy.loginfo("Executing goal ...")
        point = [goal.target_pose.pose.position.x,
                 goal.target_pose.pose.position.y]
                  
        path = self.__calculatePath(point)
        total_time = self.__calculate_distance(path) / self.robot_speed
        #loop
        cond = True
        for i in range(self.no_points):
            
            pull_point = [self.position[0]+self.pull_point * np.cos(self.orientation),
                         self.position[1]+self.pull_point * np.sin(self.orientation)]
            
            next_point = path[:,i]
            next_point_x = next_point[1]
            next_point_y = next_point[0]
            pull_point_x = pull_point[0]
            pull_point_y = pull_point[1]
            
            err_distance_x = next_point_x - pull_point_x
            err_distance_y = next_point_y - pull_point_y
            
            
            v_x = self.kp * err_distance_x
            v_y = self.kp * err_distance_y
            
            v_tangent = v_x * np.cos(self.orientation) + v_y * np.sin(self.orientation)
            v_perpendicular = -v_x * np.sin(self.orientation) + v_y * np.cos(self.orientation)
            
            self.speed_msg.linear.x = v_tangent
            
            self.speed_msg.angular.z = v_perpendicular * self.pull_point
            
            self.pub.publish(self.speed_msg)
            try:
                self._feedback.feedback.base_position.pose.position.x = self.position[0]
                self._feedback.feedback.base_position.pose.position.y = self.position[1]  
                self._feedback.feedback.base_position.pose.orientation.z = np.sin(self.orientation/2.0)
                self._feedback.feedback.base_position.pose.orientation.w = np.cos(self.orientation/2.0) 
                self._as.publish_feedback(self._feedback)
            except:
                pass
            rospy.sleep(total_time / float(self.no_points))
            
        self._result.status = 3
        rospy.loginfo("Robot succeeded")
        self._as.set_succeeded(self._result)

    def __calculate_distance(self, path):
        distance = 0.0
        ind = 0
        for  y_target, x_target in zip(path[0][1:],path[1][1:]):
            distance += np.linalg.norm(np.array([path[:,ind][1],path[:,ind][0]])-np.array([x_target, y_target]))
            ind += 1
        return distance
            
    def __calculatePath(self, point):
        x0 = self.position[0] +  self.pull_point *  np.cos(self.orientation)
        y0 = self.position[1] +  self.pull_point *  np.sin(self.orientation)
        x1 = point[0]
        y1 = point[1]
        y1_prime = (y1 - y0)/(x1 - x0)
        y0_prime = self.orientation

        
        X = np.linspace(x0,x1,self.no_points)
        A = np.array([[x0**3, x0**2, x0, 1.0],
                      [3*x0**2, 2*x0, 1.0, 0.0],
                      [x1**3, x1**2, x1, 1],
                      [3*x1**2, 2*x1, 1.0, 0.0]])
        C = np.array([y0, y0_prime, y1, y1_prime]).reshape(4,1)
        
        a, b, c, d = np.dot(np.linalg.inv(A), C)
        Y = a*X**3 + b*X**2 + c*X + d
        path = np.array([ Y , X ])
        
        if self.plot_enable:
            self.__plot(X,Y)  
        return path
        
    def __plot(self,x,y):
        plt.plot(x,y, 'r-')
        plt.plot(x[0],y[0], 'bo', linewidth = 10)
        plt.show()
        plt.grid()
        
        
turtle = Robot()
while not rospy.is_shutdown():
    turtle.rate.sleep()

