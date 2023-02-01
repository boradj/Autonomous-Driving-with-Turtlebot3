
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
import math
import numpy as np
from numpy import inf
from wall_following import wallfollower
from Astar import AStarPlanner
from Astar import main
from pathplanningandpathfollowing import pathplanning
from rotate import rotate
import time

# variable which need later on

roll = pitch = yaw = 0.0

kp=0.5


def get_rotation (msg):
    global roll, pitch, yaw, target_rad, cur_position
    cur_position = msg.pose.pose.position
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	

sub = rospy.Subscriber ('/odom', Odometry, get_rotation)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=False)


if __name__ == "__main__":
    task3 = wallfollower()
    task3.check()

    s, target_rad, p = task3.get_cor()
    p1 = task3.get_spot()

    items = []
    items.append(s)
    items.append(p)
    
    file_to_delete = open("items.txt",'w')
    file_to_delete.close()
    file = open('items.txt','w')
    for item in items:
    	for i in range(3):
            s1 = str(item[i])
            file.write(s1 +"\n")
    file.close()
    
    sx = cur_position.x
    
    sy = cur_position.y
    gx = s[0]
    gy = s[1]
    task1 = pathplanning()
    task1.spin(sx, sy, gx, gy)
    velocity = Twist()
    velocity.linear.x = 0.000
    velocity.angular.z = 0.000
    pub.publish(velocity)    

    a = target_rad + yaw
    time.sleep(3)

    if yaw >= 0 and a > 6.24:
        target_rad = a + 2*np.pi
        
    elif yaw >= 0 and a > 3.14:
        target_rad = (a-2*np.pi)
        
    elif yaw < 0 and a > 0:
        target_rad = a
        
    elif yaw < 0 and a < 0:
        target_rad = a
    else:
        print('conversion is not possible')

    Dis = []
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

    file.close()

    ox, oy, oi = [], [], []
    for i in range(len(collision)):
        if collision[i][0] == inf or collision[i][1] == inf:
            pass
        else:
            p, q = collision[i][0], collision[i][1] #, collision[i][2]
            ox.append(p)
            oy.append(q)

        
    # start and goal position

    grid_size = 0.12  # [m]
    robot_radius = 0.12  # [m]
    sx = s[0]
    sy = s[1]
    p23 = []

    
    for i in range(len(p1)):
        p12 = p1[i]
        if math.isnan(p12[0]):
            print('.')
        else:
            p23.append(p12)

    print(len(p1))
    for i in range(len(p23)):
        p13 = p23[i]
        gx1 = p13[0]
        gy1 = p13[1]

        a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
        rx, ry, D = a_star.planning(sx, sy, gx1, gy1)
        Dis.append(D)
        
    time.sleep(3)
    Dis1 = Dis
    if len(Dis1) == 1:
        i = 0
    else:
        i = Dis1.index(min(Dis1))


    rospy.sleep(5)

    f = p23[i]
    fgx = f[0]
    fgy = f[1]
    task1 = pathplanning()
    task1.spin(sx, sy, fgx, fgy) 
    velocity = Twist()
    velocity.linear.x = 0.000
    velocity.angular.z = 0.000
    pub.publish(velocity)      
    	
