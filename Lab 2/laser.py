#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import tf.transformations as tftr
import time
import threading
import matplotlib.pyplot as plt
import numpy as np

from numpy import inf
import math


class LaserData():


    # constructor 
    def __init__(self, robot_name="turtlebot"):
        self.lock = threading.Lock()
        self.flag = True
        rospy.init_node('robot_control_node', anonymous=True)

        if robot_name == "summit":
            rospy.loginfo("Robot Summit...")
            cmd_vel_topic = "/summit_xl_control/cmd_vel"
            # We check sensors working
            self._check_summit_laser_ready()
            
        else:      
            rospy.loginfo("Robot Turtlebot...")      
            cmd_vel_topic='/cmd_vel'
            self._check_laser_ready()

        # We start the publisher
        self.vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self.cmd = Twist()        

        self.laser_subscriber = rospy.Subscriber(
            '/scan', LaserScan, self.laser_callback)
        self.summit_laser_subscriber = rospy.Subscriber(
            '/scan', LaserScan, self.summit_laser_callback)
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.odometry_callback)
        
        self.ctrl_c = False
        self.rate = rospy.Rate(1)
        rospy.on_shutdown(self.shutdownhook)
   
    def odometry_callback(self, msg):
        self.lock.acquire()
        # read current robot state
        self.cur_position = msg.pose.pose.position
        cur_q = msg.pose.pose.orientation
        cur_rpy = tftr.euler_from_quaternion((cur_q.x, cur_q.y, cur_q.z, cur_q.w))  # roll pitch yaw
        self.cur_rot_z = cur_rpy[2]

        if self.flag:
            self.zero_pose = [self.cur_position.x, self.cur_position.y, self.cur_position.z]
            self.flag = False
        self.lock.release()

    
    # method to check if the laser is ready 
    def _check_laser_ready(self):
        self.laser_msg = None
        rospy.loginfo("Checking Laser...")
        while self.laser_msg is None and not rospy.is_shutdown():
            try:
                self.laser_msg = rospy.wait_for_message("/scan", LaserScan, timeout=1.0)
                rospy.logdebug("Current /scan READY=>" + str(self.laser_msg))

            except:
                rospy.logerr("Current /scan not ready yet, retrying for getting scan")
        rospy.loginfo("Checking Laser...DONE")
        return self.laser_msg


    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True

    def laser_callback(self, msg):
        self.laser_msg = msg

    def summit_laser_callback(self, msg):
        self.summit_laser_msg = msg
   
    # method to get the range of obstacle
    def get_laser(self, pos):
        time.sleep(1)
        return self.laser_msg.ranges[pos]

    def get_laser_summit(self, pos):
        time.sleep(1)
        return self.summit_laser_msg.ranges[pos]

    def get_front_laser(self):
        time.sleep(1)
        return self.laser_msg.ranges[360]

    def get_laser_full(self):
        time.sleep(1)
        return self.laser_msg.ranges

    # a function to take angle and radius as input
    # returns cordinates of x and y of obstacles.
    def circle_sections(self, angle, radius):
        return radius*math.cos(angle), radius*math.sin(angle)

    def map(self):
        # creating empty list for cordinates
        x = []
        y = []
        
        
        # instance of class
        ld = LaserData()

        
        plt.ion()
        fig, ax = plt.subplots()
        ax.set_xlim(-5, 5)
        ax.set_ylim(-5, 5)
        sc = ax.scatter(x, y)
        plt.draw()
        

        while True:
                X = []
                Y = []
                a = ld.get_laser_full()
                

                # converting tuple laser output to array
                a_array = np.asarray(a)
                
                
                
                for i in range(0, 360):
                    
                # removing inf (infinity) values from the arrays
                    if a_array[i] == inf:
                        
                        pass

                    else:

                        r = a_array[i]
                        
                    
                        d2r = (i*np.pi)/180 # degree to radians conversion
                        
                        a = d2r + self.cur_rot_z

                        x, y = r*math.cos(a), r*math.sin(a)# getting x and y cordinates of obstacle
                        c = x + self.cur_position.x
                        d = y + self.cur_position.y
                        # appending  cordinate list
                        X.append(c)
                        Y.append(d)

                sc.get_offsets()

                # add the points to the plot
                #plt.plot(self.cur_position.x ,self.cur_position.y, color='red', marker="p", markersize=6)
                x = np.append(x, X)
                y = np.append(y, Y)
                sc.set_offsets(np.c_[x,y])
                #sc.set_offsets(self.cur_position.x, self.cur_position.y, c=)
                ax.scatter(self.cur_position.x, self.cur_position.y, c = [0, 1, 0])
                
                fig.canvas.draw_idle()
                plt.pause(0.000001)
                


if __name__ == '__main__':
    task2 = LaserData()
    task2.map()
    
