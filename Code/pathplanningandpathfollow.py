import rospy
import threading

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
import tf.transformations as tftr
#from traj import oval, gener_traj

from numpy import matrix, cos, arctan2, sqrt, pi, sin, cos
import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib.markers import MarkerStyle
from Astar import AStarPlanner
show_animation = True
class pathplanning:

    def __init__(self):
        self.lock = threading.Lock()
        self.RATE = rospy.get_param('/rate', 50)

        self.dt = 0.0
        self.time_start = 0.0
        self.end = False
        self.t = 0.0
        self.t_ = 0.0
        self.dist2goal = 0.0
        self.count = 0

        #self.T = rospy.get_param('~T', 1)
        #self.oval = oval(self.T)
        
        self.pose_init = [0.0, 0.0, 0.0]
        self.flag = True
        self.toggle = True

        "Desired values setup"
        # rotation matrix [4x4] from `world` frame to `body`
        
        self.bTw = tftr.euler_matrix(-np.pi, 0.0, 0.0, 'rxyz')

        # in `world` frame
        self.A = rospy.get_param('/A', 90.0)    # [degrees]
        self.pose_des = rospy.get_param('/pose_des', [0.5, 0.0, 2.0])
             
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
        self.cur_position = msg.pose.pose.position
        cur_q = msg.pose.pose.orientation
        cur_rpy = tftr.euler_from_quaternion((cur_q.x, cur_q.y, cur_q.z, cur_q.w))  # roll pitch yaw
        cur_rot_z = cur_rpy[2]
        self.cur_rot = cur_rot_z

        if self.flag:
            self.zero_pose = [self.cur_position.x, self.cur_position.y, self.cur_position.z]
            self.flag = False


        '''CALCULATE ERRORS HERE AND DISTANCE TO GOAL'''
        e_x = self.pose_des[0] - self.cur_position.x
        e_y = self.pose_des[1] - self.cur_position.y
        e_z = self.pose_des[2] - cur_rot_z
        
        self.dist2goal = sqrt((e_x**2) + (e_y**2))
        error_angle = -arctan2(e_y, e_x) + cur_rot_z
        if error_angle > np.pi:
            error_angle = -arctan2(e_y, e_x) + cur_rot_z - 2*np.pi
        elif error_angle < -np.pi:
            error_angle = -arctan2(e_y, e_x) + cur_rot_z + 2*np.pi
        else:
        	error_angle = -arctan2(e_y, e_x) + cur_rot_z
        
        # set control
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

        self.lock.release()
        
    def map_plot(self, x1, y1, X, Y):
        self.ax[0].cla()
        self.ax[1].scatter(x1, y1, s = 5, c = [0, 1, 0], zorder=0)
        self.ax[0].scatter(X, Y, s = 5, c = [0, 1, 0], zorder=0)
        l = self.cur_rot*180/np.pi        
        m = MarkerStyle(">")
        m._transform.rotate_deg(l)
        self.ax[0].scatter(x1, y1, marker=m, s=100, color="crimson")
        self.fig.canvas.draw_idle()
        plt.pause(0.000001)

    def spin(self, sx, sy, gx, gy):
        rospy.loginfo('Task started!')
        X = []
        Y = []
        file = open('ranges.txt','r')
        lines = file.readlines()
        collision = []
        for i in range(len(lines)):
            s12 = []
            s1 = lines[i]
            #s1 = lines[1]
            p = s1.split("\n")
            p = p[0].replace("[", "")
            p = p.replace("]", "")
            p = p.split(",")
            p = np.array(p)
            res = p.astype(np.float)
            c1 = res.tolist()
            collision.append(c1)

        ox, oy = [], []
        for i in range(len(collision)):
            p, q = collision[i][0], collision[i][1]
            ox.append(p)
            oy.append(q)
            
        # start and goal position

        grid_size = 0.2  # [m]
        robot_radius = 0.105  # [m]

        a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
        self.rx, self.ry, D = a_star.planning(sx, sy, gx, gy)

        def Convert(lst):
            return [ -i for i in lst ]
	   
        self.ry = Convert(self.ry)
        rate = rospy.Rate(self.RATE)
        time_step = 5.0
        self.end = False

        time_prev = 0.0
        self.time_start = rospy.get_time()

        last_time = 0
        self.count = len(self.rx) - 1
        while not rospy.is_shutdown():
            t = rospy.get_time() - self.time_start
            self.dt = t - time_prev
            time_prev = t
            #print(self.count)
            
            if self.dist2goal < 0.2 and self.toggle:
                
                self.count -= 1
                self.toggle = False
                last_time = t
              

            self.x = round(self.rx[self.count], 2)
            self.y = float(round(self.ry[self.count], 2))
            	
            self.dt1 = t - last_time
            if self.dt1 > 3 and self.count != 0:
                self.toggle = True          	
            if self.count > -1:
                self.pose_des = self.transform_pose([self.x,self.y, 0.0])


            rate.sleep()
            time.sleep(0.8)
            print(self.count)
            if self.count == -1:
            	self.count = 0
            if self.count == 0 and self.dist2goal < 0.1:
            	break
            #self.map_plot(self.cur_position.x, self.cur_position.y, X, Y)

            
        rospy.loginfo('Task completed!')


if __name__ == "__main__":
    rospy.init_node('task1_node')
    task1 = Task1()
    # starting point and goal point
    task1.spin(-1.0, 0.2, -2, 0.6 )
