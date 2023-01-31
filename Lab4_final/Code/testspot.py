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


class Task1:

    def __init__(self):
        self.lock = threading.Lock()
        self.RATE = rospy.get_param('/rate', 50)

        self.dt = 0.0
        self.time_start = 0.0
        self.prev_error_angle = 0.0
        self.end = False
        self.dist2goal = 0.0
        self.pose_init = [0.0, 0.0, 0.0]
        self.flag = True
        self.count = 0
        self.do = True
        self.toggle = False

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

        "ROS stuff"
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=False)
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.odometry_callback)
    
    def transform_pose(self, pose_w):

        # in 'body' frame

        pose_des = self.bTw * np.matrix([pose_w[0], pose_w[1], pose_w[2], 0.0]).T
        return pose_des[:3]
            
    def odometry_callback(self, msg):
        self.lock.acquire()
        # read current robot state
        cur_position = msg.pose.pose.position
        cur_q = msg.pose.pose.orientation
        cur_rpy = tftr.euler_from_quaternion((cur_q.x, cur_q.y, cur_q.z, cur_q.w))  # roll pitch yaw
        self.cur_rot_z = cur_rpy[2]

        if self.flag:
            self.zero_pose = [cur_position.x, cur_position.y, cur_position.z]
            self.current_rot_z = self.cur_rot_z
            self.flag = False


        '''CALCULATE ERRORS HERE AND DISTANCE TO GOAL'''
        e_x = self.pose_des[0] - cur_position.x
        e_y = self.pose_des[1] - cur_position.y
        e_z = self.pose_des[2] - self.cur_rot_z
        
        self.dist2goal = sqrt((e_x**2) + (e_y**2))
        error_angle = -arctan2(e_y, e_x) + self.cur_rot_z
        if error_angle > np.pi:
        	error_angle = -arctan2(e_y, e_x) + cur_rot_z - 2*np.pi
        elif error_angle < -np.pi:
            error_angle = -arctan2(e_y, e_x) + self.cur_rot_z + 2*np.pi
        else:
        	error_angle = -arctan2(e_y, e_x) + self.cur_rot_z
        
        # set control
        "Controllers"
        velocity = Twist()
        self.Kp_Linear = 0.15*self.dist2goal*cos(error_angle)
        self.Kp_Angular = -0.8*error_angle
        self.Ki_Linear = 0.20*self.dist2goal*cos(error_angle)*(self.dt)
        self.Ki_Angular = 0.5*error_angle*(self.dt)
        velocity.linear.x = self.Kp_Linear + self.Ki_Linear
        velocity.angular.z = self.Kp_Angular + self.Ki_Angular
        self.pub_cmd_vel.publish(velocity)
        self.dist2goal_prev = self.dist2goal
        self.prev_error_angle = error_angle
        print('ERROR:' , e_x, e_y)
        print('ORIENTATION:' ,error_angle)     	
        self.lock.release()

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
        intensity_threshold = 1
        center_angle = 0
        start_angle = 0
        end_angle = 0
        for i in range(length_laser):
            if i >= minimun_scan_angle and i < maximun_scan_angle:
                spot_intensity = msg.intensities[i]
                if spot_intensity <= intensity_threshold and spot_intensity > 0.0:
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
                    start_angle = spot_angle_index[2]
                    end_angle = spot_angle_index[-3]

                else:
                    self.scan_done = False
        a = msg.ranges
        a_array = np.asarray(a)
        angles1 = angles[center_angle] + self.cur_rot_z
        x1 = a_array[center_angle]*math.cos(angles1)
        y1 = a_array[center_angle]*math.sin(angles1)
        c = x1 + self.cur_position.x
        d = y1 + self.cur_position.y
        return self.scan_done, center_angle, start_angle, end_angle, c, d

    '''
    def get_angle_distance(self, angle, msg):
        distance = msg.ranges[int(angle)]
        if msg.ranges[int(angle)] is not None and distance is not 0:
            angle = int(angle)
            distance = distance
        return angle, distance    

    '''    

    def get_point(self,center_angle, msg):
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        angles = np.append(angles, angles[0])
        a = msg.ranges
        a_array = np.asarray(a)
        angles1 = angles[center_angle] + self.cur_rot_z
        x1 = a_array[center_angle]*math.cos(angles1)
        y1 = a_array[center_angle]*math.sin(angles1)
        c = x1 + self.cur_position.x
        d = y1 + self.cur_position.y
        return c, d



    '''
    def get_point(self, start_angle_distance):
        angle = start_angle_distance[0]
        angle = np.deg2rad(angle - 180)
        distance = start_angle_distance[1]

        if angle >= 0 and angle < pi / 2:
            x = distance * cos(angle) * -1
            y = distance * sin(angle) * -1
        elif angle >= pi / 2 and angle < pi:
            x = distance * cos(angle) * -1
            y = distance * sin(angle) * -1
        elif angle >= -pi / 2 and angle < 0:
            x = distance * cos(angle) * -1
            y = distance * sin(angle) * -1
        else:
            x = distance * cos(angle) * -1
            y = distance * sin(angle) * -1

        return [x, y]
    '''

    '''
    def finding_spot_position(self, center_angle, start_angle, end_angle):
        print("scan parking spot done!")
        fining_spot = False
        start_angle_distance = self.get_angle_distance(start_angle)
        center_angle_distance = self.get_angle_distance(center_angle)
        end_angle_distance = self.get_angle_distance(end_angle)

        if start_angle_distance[1] != 0 and center_angle_distance[1] != 0 and end_angle_distance[1] != 0:
            print("calibration......")
            start_point = self.get_point(start_angle_distance)
            center_point = self.get_point(center_angle_distance)
            end_point = self.get_point(end_angle_distance)
            fining_spot = True
        else:
            fining_spot = False
            print("wrong scan!!")

        return fining_spot, start_point, center_point, end_point
    


    def rotate_origin_only(self, x, y, radians):
        xx = x * cos(radians) + y * sin(radians)
        yy = -x * sin(radians) + y * cos(radians)
        return xx, yy
    '''


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
        '''
        if self.scan_done and self.toggle == False:
            self.toggle = True
            self.x, self.y = self.get_point(center_angle, msg)
            print(self.x)
            print(self.y)
        '''
        if self.scan_done == False and self.toggle == False:
            self.p1 = []
            self.p2 = []
        	
        if center_angle == 90:
        	#self.p1 = []
        	#self.p2 = []
            self.toggle = True
            self.p1.append(self.x)
            self.p2.append(self.y)
        	#self.parking_spot.append(p1)
        	
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
        	
        print(self.x)
        print(self.scan_done)
        print(self.y)
        print(start_angle)  
        print(center_angle)
        print(end_angle) 
        self.bug_action()

    def bug_action(self):    
        b = 0.5  # maximum threshold distance
        a = 0.5
        c = 0.1 # minimum threshold distance
        velocity = Twist()  # Odometry call for velocity
        #linear_x = 0  # Odometry message for linear velocity will be called here.
        #angular_z = 0  # Odometry message for angular velocity will be called here.

        #rospy.loginfo("follow_direction {f}".format(f=self.follow_dir))  # This will indicate the direction of wall to follow.        

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
                print(self.section['left1'])        
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
                #rospy.loginfo("follow left wall is not running")

                if self.section['front'] < 0.1:
                    #self.change_state(6)
                    time.sleep(3)
                    
            
            	
        
        elif self.follow_dir == 1:  # Algorithm for right wall follower

            if self.state_ == 3:
                self.change_state(2)
            if self.section['right1'] < b:
                self.change_state(2)
            if self.section['right'] < a:
                self.change_state(4)
            if self.state_ == 2:
            	if self.section['left'] < a:
                    self.change_state(4)

            if self.section['front'] < a and self.section['left'] > b:
                self.change_state(1)
            elif self.section['right'] <= b and self.section['front1'] > b:
                self.change_state(2)
            elif self.section['right1'] <= b and self.section['front1'] <= b:
                self.change_state(1) 
            elif self.section['right'] > b and self.section['front'] > b:
                self.change_state(5)                       
            elif self.section['right1'] < b and self.section['front1'] > b:
                self.change_state(3)
            else:
                print('.')
                #rospy.loginfo("follow left wall is not running")

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

        self.pose_des = self.transform_pose(self.zero_pose1)

        self.dist2goal = sqrt((e_x**2) + (e_y**2))
        error_angle = -arctan2(e_y, e_x) + self.cur_rot_z
        if error_angle > np.pi:
            error_angle = -arctan2(e_y, e_x) + self.cur_rot_z - 2*np.pi
        elif error_angle < -np.pi:
            error_angle = -arctan2(e_y, e_x) + self.cur_rot_z + 2*np.pi
        else:
            error_angle = -arctan2(e_y, e_x) + self.cur_rot_z
                    
        self.lock.release()


    def get_cor(self):
        return self.zero_pose, self.zero_pose1

    def get_spot(self):
        return self.parking_spot
    	
    def get_obstacle(self):
        return self.previous_list


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
    
    def move_diag_left(self):
        velocity = Twist()
        velocity.linear.x = 0.05
        velocity.angular.z = 0.35
        return velocity

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
        while not rospy.is_shutdown(): 
            #a = self.section['front12']
            #print(a)
            #print(self.section['front'])
            t = rospy.get_time() - self.time_start
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
            s = self.get_cor()
            #print(s)       
            self.t = rospy.get_time() - self.time_start
            self.diff = self.dist2goal - self.dist
            if self.dt > 50:
                self.dist = self.dist2goal
            if self.dt > 50 and self.dist < 0.2 and self.end1:
                break 
            
            print(self.dist2goal)   
            rospy.sleep(0.1)
            self.pub.publish(velocity)

        velocity = Twist()
        velocity.linear.x = 0.000
        velocity.angular.z = 0.000
        self.pub.publish(velocity)
                
        #rospy.spin()
        
    	
    def map(self):
        # creating empty list for cordinates
        x = []
        y = []
        
        ld = wallfollower()
        # instance of class

        
        plt.ion()
        fig, ax = plt.subplots()
        ax.set_xlim(-8, 8)
        ax.set_ylim(-8, 8)
        sc = ax.scatter(x, y)
        plt.draw()
        

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
                
                #d2r = (90*np.pi)/180
                #p = min(angles, key=lambda x:abs(x - d2r))
                #i = np.where(angles == p)
                

                # converting tuple laser output to array
                a_array = np.asarray(a)
                #print(p)
                #print(i)
                #print(a_array[89])
                #print(a_array[90])
                #print(a_array[i])
                
                
                for i in range(0, len(a_array)):
                    
                # removing inf (infinity) values from the arrays
                    if a_array[i] == inf:
                        
                        pass
                        '''
                        r = 3.5
                        a = angles[i] + self.cur_rot_z
                        x1, y1 = r*math.cos(a), r*math.sin(a)
                        c = x1 + self.cur_position.x
                        d = y1 + self.cur_position.y
                        X.append(c)
                        Y.append(d)
                        '''

                    else:

                        r = a_array[i]
                        
                    
                        #d2r = (i*np.pi)/180 # degree to radians conversion
                        
                        a = angles[i] + self.cur_rot_z
                        #print(angles)
                        #print(len(angles))
                        

                        x1, y1 = r*math.cos(a), r*math.sin(a)# getting x and y cordinates of obstacle
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
                #print(arry1)
                arry1 = list(arry1)
                #print(arry1)
                arry3 = conve(intensities)
                arry3 = list(arry3)
                #new_list_ox = np.append(previous_ox, arrx1)
                #new_list_oy = np.append(previous_oy, arry1)
                '''
                for i in len(arrx1):
                	if arrx1[i] not in self.previous_ox:
                		if arry2[i] not in self.previous_oy: 
                			self.previous_ox = np.append(self.previous_ox, arrx1[i])
                			self.previous_oy = np.append(self.previous_oy, arry1[i])
                '''
                new_co = []
                
                for i in range(len(arrx1)):
                    p = [arrx1[i], arry1[i]]
                    new_co.append(p)
                print(new_co)
                ruf_list = self.previous_list + new_co
                p = []
                
                coords1=[ x+1j*y for (x,y) in ruf_list] # using complex; a way for grouping
                uniques1,counts1=np.unique(coords1,return_counts=True)
                res1=[ [x.real,x.imag] for x in uniques1[counts1==1] ] # ungroup
                res2=[ [x.real,x.imag] for x in uniques1[counts1==2] ]

                for i in range(len(new_co)):
                    if new_co[i] in res2:
                        pass
                    else:
                        self.previous_list.append(new_co[i])
                        p.append(i)
                        #self.previous_inten.append(arry3[i])

                for i in p:
                    self.previous_inten.append(arry3[i])
                
                coords2=[ x+1j*y for (x,y) in ruf_list] # using complex; a way for grouping
                uniques2,counts2=np.unique(coords2,return_counts=True)
                res3=[ [x.real,x.imag] for x in uniques2[counts2==1] ] # ungroup		


                        
                print(ruf_list)
                print(len(ruf_list))
                print(len(self.previous_inten))
                print(self.previous_inten)
                print(uniques2)
                print(len(uniques2))
                print(counts2)
                print(coords2)
                print(len(coords2))
                print(res3)
                print(len(res3))
                #x123 = np.array(x121)
                #new_list = np.append(x111, x123)
                #x321 = x123
                #print(X[0])
                #print(a_array[0])
                #print(intensities[0])
                #print(imax)
                #print(intensities)
                #sc.get_offsets()

                # add the points to the plot
                #plt.plot(self.cur_position.x ,self.cur_position.y, color='red', marker="p", markersize=6)
                X = []
                Y = []
                for i in range(len(self.previous_list)):
                    p, q = self.previous_list[i][0], self.previous_list[i][1]
                    #print(p)
                    X.append(p)
                    Y.append(q)                
                x = np.append(x, X)
                y = np.append(y, Y)
                #print(x)
                #print(x123)
                #print(len(x))
                #print(x)
                #print(len(y))
                #print(y)
                xl = x.max() + 2
                yl = y.max() + 2
                ax.cla()
                ax.set_xlim(-xl, xl)
                ax.set_ylim(-yl, yl)
                #sc.set_offsets(np.c_[x,y])
                ax.scatter(x, y, s = 15)
                #sc.scatter(x, y)
                #p = self.cur_position.x
                #q = self.cur_position.y
                #sc.set_offsets(p, q)
                l = self.cur_rot_z*180/np.pi
                m = MarkerStyle(">")
                m._transform.rotate_deg(l)	
                ax.scatter(self.cur_position.x, self.cur_position.y, marker = m, s = 50, c = "crimson", zorder=0)
                
                fig.canvas.draw_idle()
                plt.pause(0.000001)
        rospy.loginfo('Task completed!')               


if __name__ == "__main__":	
    rospy.init_node('followwall_node')
    task3 = wallfollower()
    task3.check()
    s, p = task3.get_cor()
    p1 = task3.get_spot()
    print(p1)
    #print(y)
    items = []
    items.append(s)
    items.append(p)
    print(s)
    print(p)
    file_to_delete = open("items.txt",'w')
    file_to_delete.close()
    file = open('items.txt','w')
    for item in items:
    	for i in range(3):
            s1 = str(item[i])
            file.write(s1 +"\n")
    file.close()
