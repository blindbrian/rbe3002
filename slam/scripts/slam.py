#!/usr/bin/python
#RBE 3002 Final Project
#Authors: Brian Eccles, Adria Fung, Prateek Sahay
import rospy, math
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import GridCells
from nav_msgs.msg import Odometry
from drive.srv import Goto
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from pathfinder.srv import GetPath
from kobuki_msgs import ButtonEvent
from kobuki_msgs import Sound

def frontierCallback(msg):
	global frontiers
	frontiers = msg.cells

def read_odometry(msg):
	global currPos
	currPos = msg.pose.pose

def read_button(msg):
	global state
	if (msg.button == 0 and msg.state == 1):
		if (state == 1):
			state = 0
		else:
			state = 1
			
def findClosestFrontier():
	global frontiers
	global currPos
	
	max_d = math.sqrt((currPos.position.x-frontiers[0].position.x)**2 +(currPos.position.y-frontiers[0].position.y))
	max_p = frontiers[0]
	
	for p in frontiers:
		d = math.sqrt((currPos.position.x-p.position.x)**2 +(currPos.position.y-p.position.y))
		if (d < max_d):
			max_d = d
			max_p = p
			
	return max_p

#Main Function
if __name__ == '__main__':
	#Initialize ROS Node
	rospy.init_node('slam')

	#Setup globals
	global frontiers
	
	#Publishers
	global sound_pub
	sound_pub = rospy.Publisher(rospy.get_param('kobuki_sound_output_topic', '/sound'), Sound)
	
	#Subscribers
	global frontier_sub
	global odom_sub
	global button_sub
	frontier_sub = rospy.Subscriber(rospy.get_param('frontier_input_topic', '/frontiers'), Path, frontierCallback, queue_size = 10)
	odom_sub = rospy.Subscriber(rospy.get_param('odometry_input_topic', '/odom'), Odometry, read_odometry, queue_size=1) 
	button_sub = rospy.Subscriber(rospy.get_param('kobuki_button_input_topic', '/button'), ButtonEvent, read_button, queue_size=1)

	#Service proxies
	global getpath_srv
	global goto_srv
	rospy.wait_for_service('/getpath')
	rospy.wait_for_service('/goto')
	getpath_srv = rospy.ServiceProxy('/getpath', GetPath)
	goto_srv = rospy.ServiceProxy('/goto', Goto)
	
	#Setup Complete
	print 'SLAM node setup complete'
	
	#Loop Until Shutdown
	while not rospy.is_shutdown():
		if (state == 1):
			if (len(frontiers) == 0):
				state == 0
				sound_pub.publish(6)
			goal = findClosestFrontier()
			path = getpath_srv(currPos, goal)
			for p in path:
				goto_srv(p)
			rospy.sleep(3)
	
	#Exit Node
	print 'SLAM node exiting'