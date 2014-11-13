#RBE 3002 Lab 3
#Authors: Brian Eccles, Adria Fung
import rospy, math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid

#Callback for turtlebot odometry messages
#param msg: Incoming message of type nav_msgs/Odometry
#returns: nothing
def odometryCallback(msg):
	print msg

#Callback for turtlebot mapping messages
#param msg: Income message of type nav_msgs/OccupancyGrid
#returns: nothing
def mapCallback(msg):
	print msg


#Main Function
if __name__ == '__main__':
	#initialize ros nod
	rospy.init_node('lab3')
	
	#Publishers
	global teleop_pub
	teleop_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist)
	
	#Subscribers
	global odom_sub
	global map_sub
	odom_sub = rospy.Subscriber('/odom', Odometry, odometryCallback, queue_size=10)
	map_sub = rospy.Subscriber('/map', OccupancyGrid, mapCallback, queue_size=10)

	print 'Lab3 node setup complete'

	while not rospy.is_shutdown():
		pass

	print 'Lab 3 node exiting'
