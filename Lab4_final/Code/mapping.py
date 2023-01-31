import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import tf.transformations as tftr
import time
import threading
import matplotlib.pyplot as plt
from matplotlib.markers import MarkerStyle
from matplotlib.animation import FuncAnimation
import numpy as np
from numpy import matrix, cos, arctan2, sqrt, pi, sin, cos

from numpy import inf
import math
from collections import deque
EXTEND_AREA = 1.0


class LaserData():


    # constructor 
    def __init__(self, robot_name="turtlebot3_burger"):
        self.lock = threading.Lock()
        self.RATE = rospy.get_param('/rate', 50)
        self.flag = True
        self.dt = 0.0
        self.time_start = 0.0
        self.dist2goal = 0.0
        self.state_ = 0
        self.follow_dir = -1
        self.previous_list = []
        
        self.previous_inten = []
        
        self.prev_error_angle = 0.0
        self.dist2goal = 0.0
        self.flag = True
        self.count = 0
        self.flag1 = True
        self.zero_pose1 = [0, 0, 0]


        "Desired values setup"
        # rotation matrix [4x4] from `world` frame to `body`
        self.bTw = tftr.euler_matrix(-np.pi, 0.0, 0.0, 'rxyz')

        # in `world` frame
        self.A = rospy.get_param('/A', 90.0)    # [degrees]
        self.pose_des = rospy.get_param('/pose_des', [0.5, 0.0, 0.2])

        # in 'body' frame
        self.pose_des = self.transform_pose(self.pose_des)
        #print(self.pose_des.T)

        rospy.init_node('robot_trajectory_follow_node', anonymous=True)        
        self.laser_subscriber = rospy.Subscriber(
            '/scan', LaserScan, self.laser_callback)
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.odometry_callback)
        self.pub_cmd_vel = rospy.Subscriber("/cmd_vel", Twist, self.map1)
        self.ctrl_c = False
        self.rate = rospy.Rate(1)
        rospy.on_shutdown(self.shutdownhook)
   
    def map1(self, msg):
        self.Twist = msg
              
                      
    def change_state(self, state):
        if state is not self.state_:
            self.state_ = state

    def callback_laser(self, msg):
        r = np.array(msg.ranges)
        self.msg1 = msg
        r[r == 0] = 3.5
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        angles = np.append(angles, angles[0])
        laser_length = len(r)
        x1 = [0, 45, 90, 135, 225, 270, 315, 359]
        x11 = []
        x22 = []
        intensity = msg.intensities
        intensities = np.asarray(intensity)

        for i in range(len(intensities)):
            if intensities[i] < 1:
                x22.append(i)

        for i in x1:
            if i < laser_length:
                d2r = (i*np.pi)/180
                p = min(angles, key=lambda x:abs(x - d2r))
                q = np.where(angles == p)
                x11.append(q)
                   
        laser_length = len(r)
        self.section = {
                'front': min(min(r[x11[0][0][0]:int(laser_length/8)]), min(r[int((7*laser_length)/8):laser_length])),
                'front2': min(min(r[x11[0][0][0]:int(laser_length/9)]), min(r[int((7*laser_length)/8):laser_length])),
                'front1': r[x11[0][0][0]],
                'left' : min (r[int(laser_length/8):int((3*laser_length)/8)]),
                'left1': r[int(laser_length/4)],
                'left2': r[int(laser_length/8) + 1],
                'left3' : min (r[int(laser_length/8):int(laser_length/4)]),
                'rear' : min(r[int((3*laser_length)/8):int((5*laser_length)/8)]),
                'right' : min(r[int((5*laser_length)/8):int((7*laser_length)/8)]),
                'right1': r[int((3*laser_length)/4)]
                
        }
        self.bug_action()

    def bug_action(self):    
        b = 0.5  # maximum threshold distance
        a = 0.5
        c = 0.1 # minimum threshold distance
        velocity = Twist()  # Odometry call for velocity
        if self.state_ == 0 and self.follow_dir == -1:
		
            
            if self.section['front1'] < c and self.section['left1'] < c and self.section['right1'] > c:
                self.change_state(7)
                time.sleep(3)           
            elif self.section['front1'] < c and self.section['left1'] > c and self.section['right1'] > c:
                self.change_state(7)
                time.sleep(3)
            if self.section['front1'] > b and self.section['left1'] > b and self.section['right1'] > b:  # Loop 1
                self.change_state(0)
                rospy.loginfo("Reset Follow_dir")
            elif self.section['front1'] < b:
                self.change_state(1)
                time.sleep(2)
                self.x = rospy.get_time()
            elif self.section['left'] < b and self.section['front'] < b and self.section['right'] > b:
                self.change_state(7)
                print('.')
            elif self.section['front'] < b and self.section['left'] < b and self.section['right'] < b:
                self.change_state(7)
                print('.')
        elif self.state_ == 1 and self.follow_dir == -1:
            self.change_state(1)
            if self.section['left1'] < b: 
                self.change_state(2)
                y = rospy.get_time() - self.x
                #print(self.section['left1'])        
        elif self.state_ == 2 and self.follow_dir == -1:
            self.follow_dir = 0
            if self.flag1:
                self.zero_pose1 = [self.cur_position.x, self.cur_position.y, self.cur_position.z]
                self.flag1 = False            

        if self.follow_dir == 0:
            
            a = 0.3
            b = 0.3 # maximum threshold distance
            	
            if self.section['front'] < 0.3 and self.section['right'] > b and self.section['left'] > a:
                self.change_state(1)
                
            elif self.section['left'] < b and self.section['front'] > b:
                self.change_state(2)     	
            elif self.section['left'] > b and self.section['front'] < 0.35:
                self.change_state(1) 
            elif self.section['left3'] < b and self.section['front'] < 0.35:
                self.change_state(1)
            elif self.section['left2'] > 0.3:
                if self.section['front2'] > b and self.section['left2'] > 0.32:
                    self.change_state(6)
                elif self.section['front'] > b:
                    self.change_state(5)            
            elif self.section['left'] < b and self.section['front'] > b:
                self.change_state(3)
            else:
                rospy.loginfo("follow left wall is not running")

                if self.section['front'] < 0.1:
                    #self.change_state(6)
                    time.sleep(3)
                    

    def transform_pose(self, pose_w):

        # in 'body' frame

        pose_des = self.bTw * np.matrix([pose_w[0], pose_w[1], pose_w[2], 0.0]).T
        return pose_des[:3]


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
         

        '''CALCULATE ERRORS HERE AND DISTANCE TO GOAL'''
        e_x = self.pose_des[0] - self.cur_position.x
        e_y = self.pose_des[1] - self.cur_position.y
        e_z = self.pose_des[2] - self.cur_rot_z
        
        self.dist2goal = sqrt((e_x**2) + (e_y**2))
        error_angle = -arctan2(e_y, e_x) + self.cur_rot_z
        if error_angle > np.pi:
            error_angle = -arctan2(e_y, e_x) + self.cur_rot_z - 2*np.pi
        elif error_angle < -np.pi:
            error_angle = -arctan2(e_y, e_x) + self.cur_rot_z + 2*np.pi
        else:
        	error_angle = -arctan2(e_y, e_x) + self.cur_rot_z
        
        self.pose_des = self.transform_pose(self.zero_pose1)
        
        
        # set control
        "Controllers"
        velocity = Twist()
        self.Kp_Linear = 0.15*self.dist2goal*cos(error_angle)
        self.Kp_Angular = -0.8*error_angle
        self.Ki_Linear = 0.20*self.dist2goal*cos(error_angle)*(self.dt)
        self.Ki_Angular = 0.5*error_angle*(self.dt)
        velocity.linear.x = self.Kp_Linear + self.Ki_Linear
        velocity.angular.z = self.Kp_Angular + self.Ki_Angular
        self.dist2goal_prev = self.dist2goal
        self.prev_error_angle = error_angle

        self.lock.release()


    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True

    def laser_callback(self, msg):
        self.laser_msg = msg

    def get_laser_full(self):
        time.sleep(1)
        return self.laser_msg

    def get_cor(self):
    	return self.previous_list
    	
    def map(self):
        # creating empty list for cordinates
        x = []
        y = []
        
        
        # instance of class
        ld = LaserData()
        rospy.Subscriber('/scan', LaserScan, self.callback_laser)
        rospy.Subscriber("/cmd_vel", Twist, self.map1)

        
        plt.ion()
        fig, ax = plt.subplots()
        ax.set_xlim(-8, 8)
        ax.set_ylim(-8, 8)
        sc = ax.scatter(x, y)
        plt.draw()
        
        for i in range(100000):
            self.time = rospy.get_time()
            if self.time == 0.0:
                self.count += 1
            else:
                break        

        self.dist = self.dist2goal
        x111 = []
        while not rospy.is_shutdown():
                X = []
                Y = []
                x = []
                y = []
                msg = ld.get_laser_full()
                a = msg.ranges
                intensity = msg.intensities
                intensities = np.asarray(intensity)
                imax = intensities.max()
                
                angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
                angles = np.append(angles, angles[0])
                angles = np.round(angles, 2)
                #print(a)

                # converting tuple laser output to array
                a_array = np.asarray(a)                
                
                for i in range(0, len(a_array)):
                    
                # removing inf (infinity) values from the arrays for gazebo and zero value for real enviroment
                    if a_array[i] == 0:
                        
                        pass

                    else:

                        r = a_array[i]
                        
                        a = angles[i] + self.cur_rot_z

                        x1, y1 = r*math.cos(a), r*math.sin(a) # getting x and y cordinates of obstacle
                        c = x1 + self.cur_position.x
                        d = y1 + self.cur_position.y
                        # appending  cordinate list
                        X.append(c)
                        Y.append(d)
                        
                      

                def conve(values, decs=1):
                    return np.trunc(values*10**decs)/(10**decs)
                
                arrx1 = conve(np.array(X))
                arrx1 = list(arrx1)
                arry1 = conve(np.array(Y))
                arry1 = list(arry1)
                arry3 = conve(intensities)
                arry3 = list(arry3)

                new_co = []
                T = self.Twist
                #print(T)
                
                for i in range(len(arrx1)):
                    p = [arrx1[i], arry1[i]]
                    new_co.append(p)
                ruf_list = self.previous_list + new_co
                p = []
                
                coords1=[ x+1j*y for (x,y) in ruf_list] # using complex; a way for grouping
                uniques1,counts1=np.unique(coords1,return_counts=True)
                res1=[ [x.real,x.imag] for x in uniques1[counts1==1] ] # ungroup
                res2=[ [x.real,x.imag] for x in uniques1[counts1==2] ]

                #print(counts1)
                
                T = self.Twist
                S = T.angular.z
                if S == 0.0:
                    for i in range(len(res1)):
                        self.previous_list.append(res1[i])
                        p.append(i)
                        
                    for i in range(len(res2)):
                        self.previous_list.append(res2[i])
                  

                coords2=[ x+1j*y for (x,y) in ruf_list] # using complex; a way for grouping
                uniques2,counts2=np.unique(coords2,return_counts=True)
                res3=[ [x.real,x.imag] for x in uniques2[counts2==1] ] # ungroup		
                
                X = []
                Y = []
                for i in range(len(self.previous_list)):
                    p, q = self.previous_list[i][0], self.previous_list[i][1]
                    X.append(p)
                    Y.append(q)                
                x = np.append(x, X)
                y = np.append(y, Y)

                ax.scatter(x, y, s = 15)
                
                l = self.cur_rot_z*180/np.pi
                m = MarkerStyle(">")
                m._transform.rotate_deg(l)	
                ax.scatter(self.cur_position.x, self.cur_position.y, marker = m, s = 50, c = "crimson", zorder=0)
                
                
                fig.canvas.draw_idle()
                plt.pause(0.000001)
                self.t = rospy.get_time() - self.time_start
                self.diff = self.dist2goal - self.dist
                if self.dt > 35:
                    self.dist = self.dist2goal
                if self.dt > 70 and self.dist < 0.6:
                    break 
                
                rospy.sleep(0.1)
            
        rospy.loginfo('Task completed!')               


if __name__ == '__main__':
    task2 = LaserData()
    task2.map()
    items = task2.get_cor()
    file_to_delete = open("ranges.txt",'w')
    file_to_delete.close()
    print(items)
    file = open('ranges.txt','w')
    for i in range(len(items)):
        s1 = str(items[i])
        file.write(s1 +"\n")
    file.close()
    
