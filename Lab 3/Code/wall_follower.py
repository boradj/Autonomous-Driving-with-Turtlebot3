import rospy
import numpy as np
from numpy import inf
from geometry_msgs.msg import Point, Twist
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf.transformations as tftr
import math
from math import atan2
from numpy import matrix, cos, arctan2, sqrt, pi, sin, cos
import time
import threading
import matplotlib.pyplot as plt
from matplotlib.markers import MarkerStyle
from matplotlib.animation import FuncAnimation


class wallfollower():
    def __init__(self):
        self.lock = threading.Lock()
        self.RATE = rospy.get_param('/rate', 50)
        velocity = Twist()
        self.dt = 0.0
        self.time_start = 0.0
        self.dist2goal = 0.0
        self.previous_list = []
        self.previous_inten = []
        self.parking_spot = []
        self.end = False
        self.end1 = False
        self.follow_dir = -1
        self.x = 0
        section = {
            'front': 0,
            'left': 0,
            'right': 0,
        }

        self.state_ = 0
        self.flag = True
        self.flag1 = True
        self.zero_pose1 = []
        self.toggle = False
        self.p = 0
        self.count = -1
        self.p1 = []
        self.p2 = []

        self.angle = 0
        self.zero_pose1 = [0, 0, 0]
        self.x = 0
        self.y = 0

        "Desired values setup"
        # rotation matrix [4x4] from `world` frame to `body`
        self.bTw = tftr.euler_matrix(-np.pi, 0.0, 0.0, 'rxyz')

        # in `world` frame
        self.A = rospy.get_param('/A', 90.0)    # [degrees]
        self.pose_des = rospy.get_param('/pose_des', [0.5, 0.0, 0.2])
             
        # in 'body' frame
        self.pose_des = self.transform_pose(self.pose_des)
        print(self.pose_des.T)

        self.rot_z_des = 0.0

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10, latch=False)
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.odometry_callback)
        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan, self.laser_callback)


        self.state_dict_ = {
            0: 'Find wall',
            1: 'Turn right',
            2: 'Follow the wall',
            3: 'Turn left',
            4: 'Diagonally right',
            5: 'Diagonally left',
            6: 'Turn left-',
            7: 'move backward'
        }


    def scan_parking_spot(self, msg, angles):
        self.scan_done = False
        ranges = np.array(msg.ranges)
        length_laser = len(ranges)
        intensity_index = []
        index_count = []
        spot_angle_index = []
        minimun_scan_angle = int(angles[int(length_laser/8)]* (180/np.pi))
        maximun_scan_angle = int(angles[int((3*length_laser)/8)]* (180/np.pi))
        intensity_threshold = 100
        center_angle = 0
        start_angle = 0
        end_angle = 0
        for i in range(length_laser):
            if i >= minimun_scan_angle and i < maximun_scan_angle:
                spot_intensity = msg.intensities[i]
                if spot_intensity <= intensity_threshold: #and spot_intensity > 0.0
                    intensity_index.append(i)
                    index_count.append(i)
                else:
                    intensity_index.append(0)
            else:
                intensity_index.append(0)

        for i in index_count:
            if abs(i - index_count[int(len(index_count) / 2)]) < 20:
                spot_angle_index.append(i)
                if len(spot_angle_index) > 10:
                    self.scan_done = True
                    center_angle = spot_angle_index[int(len(spot_angle_index) / 2)]
                    start_angle = spot_angle_index[0]
                    end_angle = spot_angle_index[-3]

                else:
                    self.scan_done = False
        a = msg.ranges
        
        a_array = np.asarray(a)
        angles0 = angles[start_angle] + self.cur_rot_z
        angles1 = angles[center_angle] + self.cur_rot_z
        angles2 = angles[end_angle] + self.cur_rot_z
        x0 = a_array[start_angle]*math.cos(angles0)
        x0 = x0.item()
        y0 = a_array[start_angle]*math.sin(angles0)
        y0 = y0.item() 
        x1 = a_array[center_angle]*math.cos(angles1)
        x1 = x1.item()
        y1 = a_array[center_angle]*math.sin(angles1)
        y1 = y1.item()
        x2 = a_array[end_angle]*math.sin(angles2)
        x2 = x2.item()
        y2 = a_array[end_angle]*math.sin(angles2)
        y2 = y2.item()
        c = x1 + self.cur_position.x
        d = y1 + self.cur_position.y
        t = sqrt(((x2 - x0)**2) + ((y2-y0)**2))
        s = 0.2/t
        s = s.item()

        x3 = c + s*(y2 - y0)
        y3 = d + s*(x2 - x0)
        x3_x = x3 - self.cur_position.x
        y3_y = y3 - self.cur_position.y
        d3 = sqrt((x3_x**2) + (y3_y**2))
        x4 = c - s*(y2 - y0)
        y4 = d - s*(x2 - x0)
        x4_x = x4 - self.cur_position.x
        y4_y = y4 - self.cur_position.y
        d4 = sqrt((x4_x**2) + (y4_y**2))
        if d3 > d4:
        	p = x4
        	q = y4
        else:
        	p = x3
        	q = y3
        return self.scan_done, center_angle, start_angle, end_angle, p, q


    def transform_pose(self, pose_w):

        # in 'body' frame

        pose_des = self.bTw * np.matrix([pose_w[0], pose_w[1], pose_w[2], 0.0]).T
        return pose_des[:3]
            
    def change_state(self, state):
        if state is not self.state_:
            #print('State of Bot - [%s] - %s' % (state, self.state_dict_[state]))
            self.state_ = state
            
    def laser_callback(self, msg):
        self.laser_msg = msg

    def get_laser_full(self):
        time.sleep(1)
        return self.laser_msg
    

    def callback_laser(self, msg):
        def Average(lst):
            return sum(lst) / len(lst)
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
        self.scan_done, center_angle, start_angle, end_angle, self.x, self.y = self.scan_parking_spot(msg, angles)

        if self.scan_done == False and self.toggle == False:
            self.p1 = []
            self.p2 = []
        	
        if center_angle  < 95 and  center_angle  > 85:
            self.toggle = True
            self.p1.append(self.x)
            self.p2.append(self.y)
        	
        if self.toggle and self.scan_done == False:
            p3 = []
            average1 = Average(self.p1)
            average2 = Average(self.p2)
            p3.append(average1)
            p3.append(average2)
            self.parking_spot.append(p3)
            self.toggle = False
        	 
        if self.parking_spot:
            self.end1 = True
        else:
            self.end1 = False
        self.bug_action()

    def bug_action(self):    
        b = 0.5  # maximum threshold distance
        a = 0.5
        c = 0.1 # minimum threshold distance
        velocity = Twist()  # Odometry call for velocity
        if self.state_ == 0 and self.follow_dir == -1:
		
            
            if self.section['front1'] < c and self.section['left1'] < c and self.section['right1'] > c:
                self.change_state(7) # if in starting it is in corner than it will go back
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
            	
            if self.section['front'] < 0.38 and self.section['right'] > b and self.section['left'] > a:
                self.change_state(1)
                
            elif self.section['left'] < b and self.section['front'] > b:
                self.change_state(2)     	
            elif self.section['left'] > b and self.section['front'] < 0.38:
                self.change_state(1) 
            elif self.section['left3'] < b and self.section['front'] < 0.38:
                self.change_state(1)
            elif self.section['left2'] > 0.3:
                if self.section['front2'] > b and self.section['left2'] > 0.38:
                    self.change_state(6)
                elif self.section['front'] > b:
                    self.change_state(5)            
            elif self.section['left'] < b and self.section['front'] > b:
                self.change_state(3)
            else:
                rospy.loginfo("follow left wall is not running")                    
            
            	
        
        elif self.follow_dir == 1:  # Algorithm for right wall follower
            a = 0.3
            b = 0.3 # maximum threshold distance
            	
            if self.section['front'] < 0.38 and self.section['left'] > b and self.section['right'] > a:
                self.change_state(1)
                
            elif self.section['right'] < b and self.section['front'] > b:
                self.change_state(2)     	
            elif self.section['right'] > b and self.section['front'] < 0.38:
                self.change_state(3) 
            elif self.section['right'] < b and self.section['front'] < 0.38:
                self.change_state(3)
            elif self.section['right'] > 0.3:
                if self.section['front2'] > b and self.section['right'] > 0.38:
                    self.change_state(4)
                elif self.section['front'] > b:
                    self.change_state(4)            
            elif self.section['right'] < b and self.section['front'] > b:
                self.change_state(1)
            else:
                rospy.loginfo("follow right wall is not running")                    
            
        	

    def odometry_callback(self, msg):
        self.lock.acquire()
        # read current robot state
        self.cur_position = msg.pose.pose.position
        cur_q = msg.pose.pose.orientation
        cur_rpy = tftr.euler_from_quaternion((cur_q.x, cur_q.y, cur_q.z, cur_q.w))  # roll pitch yaw
        self.cur_rot_z = cur_rpy[2]

        if self.flag:
            self.zero_pose = [self.cur_position.x, self.cur_position.y, self.cur_position.z]
            self.zero_orientation = self.cur_rot_z
            self.flag = False

        '''CALCULATE ERRORS HERE AND DISTANCE TO GOAL'''
        e_x = self.pose_des[0] - self.cur_position.x
        e_y = self.pose_des[1] - self.cur_position.y
        e_z = self.pose_des[2] - self.cur_rot_z

        self.pose_des = self.transform_pose(self.zero_pose1)

        self.dist2goal = sqrt((e_x**2) + (e_y**2))
        error_angle = -arctan2(e_y, e_x) + self.cur_rot_z
        if error_angle > np.pi:
            error_angle = -arctan2(e_y, e_x) + self.cur_rot_z - 2*np.pi
        elif error_angle < -np.pi:
            error_angle = -arctan2(e_y, e_x) + self.cur_rot_z + 2*np.pi
        else:
            error_angle = -arctan2(e_y, e_x) + self.cur_rot_z
                    
        print(self.parking_spot)
        #print(self.zero_orientation)      
        self.lock.release()


    def get_cor(self):
        return self.zero_pose, self.zero_orientation,  self.zero_pose1

    def get_spot(self):
        return self.parking_spot
    	
    def get_obstacle(self):
        return self.previous_list


    '''
    Function: find-wall:  This function publishes linear and angular velocities for finding wall
    '''


    def find_wall(self):
        velocity = Twist()
        velocity.linear.x = 0.2
        velocity.angular.z = 0
        return velocity


    '''
    Function: turn_left:  This function publishes linear and angular velocities for turning left.
    '''


    def turn_left(self):
        #kp = 1
        velocity = Twist()
        
        velocity = Twist()
        velocity.linear.x = 0
        velocity.angular.z = 0.3
        return velocity


    '''
    Function: turn_right:  This function publishes linear and angular velocities for turning right.
    '''


    def turn_right(self):
        velocity = Twist()
        
        velocity = Twist()
        velocity.linear.x = 0
        velocity.angular.z = -0.25
        return velocity


    '''
    Function: move_ahead:  This function publishes linear and angular velocities for moving straight.
    '''


    def move_ahead(self):
        velocity = Twist()
        velocity.linear.x = 0.2
        velocity.angular.z = 0
        return velocity


    '''
    Function: move_diag_right:  This function publishes linear and angular velocities for moving diagonally right.
    '''


    def move_diag_right(self):
        velocity = Twist()    	
        velocity.linear.x = 0.08
        velocity.angular.z = -0.3
        return velocity


    '''
    Function: move_diag_left:  This function publishes linear and angular velocities for moving diagonally left.
    '''


    def move_back(self):
        velocity = Twist()
        velocity.linear.x = -0.2
        velocity.angular.z = 0.3
        return velocity
    
    '''
    
    Function: check: This function publishes velocity values if Turtlebot3 is going far from the left wall based on state and for moving diagonally left.
    
    '''


    def move_diag_left(self):
        velocity = Twist()
        velocity.linear.x = 0.05
        velocity.angular.z = 0.35
        return velocity

    '''
    
    Function: check: This function publishes velocity values if Turtlebot3 is going far from the left wall based on state and for moving diagonally left.
    
    '''


    def move_diag_left1(self):
        velocity = Twist()
        velocity.linear.x = 0.23
        velocity.angular.z = 0.3
        return velocity

    '''
    Function: check: This function publishes velocity values for the logic based on the states.
    '''


    def check(self):

        x = []
        y = []
        plt.ion()
        fig, ax = plt.subplots()
        ax.set_xlim(-8, 8)
        ax.set_ylim(-8, 8)
        sc = ax.scatter(x, y)
        plt.draw()        
        time_prev = 0.0
        ld = wallfollower()
        self.time_start = rospy.get_time()
        rospy.Subscriber('/scan', LaserScan, self.callback_laser)
        for i in range(100000):
            self.time = rospy.get_time()
            if self.time == 0.0:
                self.count += 1
            else:
                break        

        self.dist = self.dist2goal
        self.time_start = rospy.get_time()
        self.time_start1=self.time_start
        
        while not rospy.is_shutdown(): 

            t = rospy.get_time() - self.time_start1
            self.dt = t
            velocity = Twist()            
            if self.state_ == 0:
                velocity = self.find_wall()
            elif self.state_ == 1:
                velocity = self.turn_right()
            elif self.state_ == 2:
                velocity = self.move_ahead()
            elif self.state_ == 3:
                velocity = self.turn_left()
            elif self.state_ == 4:
                velocity = self.move_diag_right()
            elif self.state_ == 5:
                velocity = self.move_diag_left1()
            elif self.state_ == 6:
                velocity = self.move_diag_left()
            elif self.state_ == 7:
                velocity = self.move_back()
                time.sleep(2)
            else:
                rospy.logerr('Unknown state!')

            rospy.sleep(0.1)
            
            self.t = rospy.get_time() - self.time_start
            self.diff = self.dist2goal - self.dist
            if self.dt > 8:
                self.dist = self.dist2goal
            if self.dt > 60 and self.end1: #and self.dist < 1: 
                break 

            rospy.sleep(0.1)
            self.pub.publish(velocity)

        velocity = Twist()
        velocity.linear.x = 0.000
        velocity.angular.z = 0.000
        self.pub.publish(velocity)
                
        
if __name__ == "__main__":	
    rospy.init_node('followwall_node')
    task3 = wallfollower()
    task3.check()
