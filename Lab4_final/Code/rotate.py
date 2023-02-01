import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
import math
import numpy as np
roll = pitch = yaw = 0.0
target = 180
kp=0.5

def get_rotation (msg):
    global roll, pitch, yaw, target_rad
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    print(yaw)
	

sub = rospy.Subscriber ('/odom', Odometry, get_rotation)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

rospy.init_node('parking_node')
target_rad = target*math.pi/180
s = target_rad + yaw

if yaw >= 0 and s > 6.24:
	target_rad = s + 2*np.pi
	
elif yaw >= 0 and s > 3.14:
	target_rad = (s-2*np.pi)
	
elif yaw < 0 and s > 0:
	target_rad = s
	
elif yaw < 0 and s < 0:
	target_rad = s
else:
	print('conversion is not possible')

print(target_rad)
r = rospy.Rate(10)
command =Twist()
def rotate():
	r = rospy.Rate(10)
	command =Twist()
	while not rospy.is_shutdown():
	    s = target_rad-yaw
	    s = float(round(s, 2))
	    print(s)
	    command.angular.z = kp * (target_rad-yaw)
	    pub.publish(command)
	    if s == 0.0:
	    	break
	    r.sleep()

	velocity = Twist()
	velocity.linear.x = 0.00000
	velocity.angular.z = 0.00000
	pub.publish(velocity)

