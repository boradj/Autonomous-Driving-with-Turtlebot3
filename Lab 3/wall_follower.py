import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf.transformations as tftr
import math
import time

class wallfollower():
    def __init__(self):
        velocity = Twist()
        self.follow_dir = -1
        self.x = 0
        section = {
            'front': 0,
            'left': 0,
            'right': 0,
        }

        self.state_ = 0
        self.toggle = True

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.odometry_callback)


        self.state_dict_ = {
            0: 'Find wall',
            1: 'Turn right',
            2: 'Follow the wall',
            3: 'Turn left',
            4: 'Diagonally right',
            5: 'Diagonally left',
            6: 'move backward'
        }
        '''
        This block contains all the functions designed to execute the navigation process.
        Function: Change_state: This function gets information about the state of robot based on the distance from
                                from obstacle and changes the state of robot.
        Argument: state:{Type: Integer} This parameter is used to get the required action set for thr robot.
                        0 - 'Find wall',
                        1 - 'turn right',
                        2 - 'Follow the wall',
                        3 - 'turn left',
                        4 - 'diagonally right',
                        5 - 'diagonally left',
                        
                        
        '''


    def change_state(self, state):
        if state is not self.state_:
            print('State of Bot - [%s] - %s' % (state, self.state_dict_[state]))
            self.state_ = state
    
    def laserdata(self,msg):
        global rp
        rp = np.array(msg.ranges)        




    def callback_laser(self, msg):
        r = np.array(msg.ranges)
        self.section = {
                'front' : min(min(r[0:5]), min(r[355:360])),
                'front1': r[0],
                'left' : min (r[90:91]),
                'left1': r[90],
                'rear' : min(r[135:225]),
                'right' : min(r[265:275]),
                'right1': r[270]
        }

        self.bug_action()


    def bug_action(self):    
        b = 0.5  # maximum threshold distance
        a = 0.5  # minimum threshold distance
        velocity = Twist()  # Odometry call for velocity
        #linear_x = 0  # Odometry message for linear velocity will be called here.
        #angular_z = 0  # Odometry message for angular velocity will be called here.

        rospy.loginfo("follow_direction {f}".format(f=self.follow_dir))  # This will indicate the direction of wall to follow.
        
        if self.state_ == 0:
		
            if self.section['front1'] > b and self.section['left1'] > b and self.section['right1'] > b:  # Loop 1
                self.change_state(0)
                print(self.section['front'])
                rospy.loginfo("Reset Follow_dir")
            elif self.section['front1'] <= b:
                self.change_state(1)
                self.x = rospy.get_time()
            elif self.section['front'] < a and self.section['left'] < a and self.section['right'] < a:
                print('.')
                #self.change_state(6)
        elif self.state_ == 1:
            y = self.x + 3.0
            if self.section['left1'] >= 0.5:
                self.change_state(1)
                print(rospy.get_time())
                print(self.section['left1'])
		
            elif self.section['left1'] < 0.5 and velocity.angular.z == 0 and rospy.get_time() > y:
                self.change_state(2)
                y = rospy.get_time() - self.x
                print(self.section['left1'])
                
        #elif self.state_ == 2:
            #print(self.section['left1'])
       	
        '''
        elif self.section['left1'] <= 0.5:
        	self.change_state(1)
        elif self.section['left1'] <= 0.5
        elif self.section[
        else:
            rospy.loginfo("Running")
         
         '''

        '''
        elif self.follow_dir == -1:  # To set the direction of wall to follow
            if self.section['left1'] <= b:
                self.change_state(2)
                self.follow_dir = 0
                rospy.loginfo("following left wall")
            elif self.section['right'] <= b:
                self.change_state(2)
                self.follow_dir = 1
                rospy.loginfo("following right wall")
            else:
                if self.section['left1'] >= 0.5:
                    self.change_state(1)
                rospy.loginfo("follow direction not set")
        '''

        '''

        if self.section['front'] > b and self.section['left'] > b and self.section['right'] > b:  # Loop 1
            self.change_state(0)
            rospy.loginfo("Reset Follow_dir")
        elif self.follow_dir == -1:  # To set the direction of wall to follow
            if self.section['left'] <= b:
                self.change_state(1)
                self.follow_dir = 0
                rospy.loginfo("following left wall")
            elif self.section['right'] <= b:
                self.change_state(3)
                self.follow_dir = 1
                rospy.loginfo("following right wall")
            else:
                self.change_state(2)
                rospy.loginfo("follow direction not set")
        elif self.section['front'] < a and self.section['left'] < a and self.section['right'] < a:
            print('.')
            #self.change_state(6)
        else:
            rospy.loginfo("Running")
        
        '''
        if self.follow_dir == 0:
            if self.section['left'] <= b and self.section['front'] > a:
                self.change_state(2)
            elif self.section['left'] > b and self.section['front'] > a:
                self.change_state(3)
            elif self.section['left'] <= b and self.section['front'] <= a:
                self.change_state(1) 
            else:
                rospy.loginfo("follow left wall is not running")
        
        elif self.follow_dir == 1:  # Algorithm for right wall follower
            if self.section['right'] > b and self.section['front'] > a:
                self.change_state(5)
            elif self.section['right'] < b and self.section['front'] > a:
                self.change_state(2)
            elif self.section['right'] < b and self.section['front'] < a:
                self.change_state(1)
            else:
                rospy.loginfo("follow right wall is not running")
        '''
        if self.follow_dir == 0:  # Algorithm for left wall follower
            if self.section['left'] > b and self.section['front'] > a:
                self.change_state(4)
            elif self.section['left'] < b and self.section['front'] > a:
                self.change_state(2)
            elif self.section['left'] < b and self.section['front'] < a:
                self.change_state(3)
            else:
                rospy.loginfo("follow left wall is not running")
        '''

    def odometry_callback(self, msg):
        #self.lock.acquire()
        # read current robot state
        self.cur_position = msg.pose.pose.position
        cur_q = msg.pose.pose.orientation
        cur_rpy = tftr.euler_from_quaternion((cur_q.x, cur_q.y, cur_q.z, cur_q.w))  # roll pitch yaw
        self.cur_rot_z = cur_rpy[2]

        #self.lock.release()


    def find_wall(self):
        velocity = Twist()
        velocity.linear.x = 0.3
        velocity.angular.z = 0
        return velocity


    '''
    Function: turn_left:  This function publishes linear and angular velocities for turning left.
    '''


    def turn_left(self):
        kp = 0.5
        velocity = Twist()
        '''
        velocity = Twist()
        velocity.linear.x = 0
        velocity.angular.z = 0.3
        
        '''
        target_rad = (90)*math.pi/180
        velocity.angular.z = kp * (target_rad - self.cur_rot_z)
        return velocity


    '''
    Function: turn_right:  This function publishes linear and angular velocities for turning right.
    '''


    def turn_right(self):
        kp = 0.5
        velocity = Twist()
        '''
        velocity = Twist()
        velocity.linear.x = 0
        velocity.angular.z = 0.3
        
        '''
        target_rad = (-90)*math.pi/180
        velocity.angular.z = kp * (target_rad - self.cur_rot_z)
        return velocity


    '''
    Function: move_ahead:  This function publishes linear and angular velocities for moving straight.
    '''


    def move_ahead(self):
        velocity = Twist()
        velocity.linear.x = 0.3
        velocity.angular.z = 0
        return velocity


    '''
    Function: move_diag_right:  This function publishes linear and angular velocities for moving diagonally right.
    '''


    def move_diag_right(self):
        velocity = Twist()
        velocity.linear.x = 0.1
        velocity.angular.z = -0.3
        return velocity


    '''
    Function: move_diag_left:  This function publishes linear and angular velocities for moving diagonally left.
    '''


    def move_diag_left(self):
        velocity = Twist()
        velocity.linear.x = 0.1
        velocity.angular.z = 0.3
        return velocity


    '''
    Function: check: This function publishes velocity values for the logic based on the states.
    '''


    def check(self):

        rospy.init_node('follow_wall')
        rospy.Subscriber('/scan', LaserScan, self.callback_laser)
        rospy.Subscriber('/scan', LaserScan, self.laserdata)
        

        while not rospy.is_shutdown():

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
                velocity = self.move_diag_left()
            else:
                rospy.logerr('Unknown state!')

            self.pub.publish(velocity)

        rospy.spin()


if __name__ == "__main__":
    task3 = wallfollower()
    task3.check()
