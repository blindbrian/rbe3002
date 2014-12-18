#!/usr/bin/python
#RBE 3002 Final Project
#Authors: Brian Eccles, Adria Fung, Prateek Sahay
import rospy, math
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import GridCells
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from rbe_srvs.srv import GetPath
from rbe_srvs.srv import Goto
from kobuki_msgs.msg import ButtonEvent
from kobuki_msgs.msg import Sound


#Callback for receiving frontier messages
def frontierCallback(msg):
	global frontiers
	frontiers = msg.poses


#Callback for receiving odometry messages
def read_odometry(msg):
	global currPos
	currPos = msg.pose.pose


#Callback for receiving turtlebot button events
def read_button(msg):
	global state
	#Toggle running state when button 0 is pressed
	if (msg.button == 0 and msg.state == 1):
		if (state == 1):
			print "Exiing SLAM"
			state = 0
		else:
			print "Starting SLAM"
			state = 1
			
#Find the closest frontier location and return it
def findClosestFrontier():
	global frontiers
	global currPos	
	#default to the first frontier in the list
	max_d = math.sqrt((currPos.position.x-frontiers[0].pose.position.x)**2 +(currPos.position.y-frontiers[0].pose.position.y)**2)
	max_p = frontiers[0]	
	#find the closest frontier
	for p in frontiers:
		d = math.sqrt((currPos.position.x-p.pose.position.x)**2 +(currPos.position.y-p.pose.position.y)**2)
		if (d < max_d):
			max_d = d
			max_p = p			
	return max_p
	
#Return a pose 180 degrees oposite of the given pose
def spin180(pose):
	angle = 2*math.asin(pose.orientation.z)
	if (angle > 0):
		angle -= math.pi
	else:
		angle += math.pi+0.1 #add a little so it will spin in one direction
	ret = Pose()
	ret.position = pose.position
	ret.orientation.z = math.sin(angle/2)
	return ret
	

#Main Function
if __name__ == '__main__':
	#Initialize ROS Node
	rospy.init_node('slam')

	#Setup globals
	global frontiers
	global state
	global currPos
	state = 0
	
	#Publishers
	global sound_pub
	sound_pub = rospy.Publisher(rospy.get_param('kobuki_sound_output_topic', '/sound'), Sound)
	
	#Subscribers
	global frontier_sub
	global odom_sub
	global button_sub
	frontier_sub = rospy.Subscriber(rospy.get_param('frontier_input_topic', '/frontiers'), Path, frontierCallback, queue_size = 10)
	odom_sub = rospy.Subscriber(rospy.get_param('odometry_input_topic', '/odom'), Odometry, read_odometry, queue_size=1) 
	button_sub = rospy.Subscriber(rospy.get_param('kobuki_button_input_topic', '/mobile_base/events/button'), ButtonEvent, read_button, queue_size=1)
	
	print "Waiting for services"

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
			else:
				goal = findClosestFrontier()
				path = getpath_srv(currPos, goal.pose).path
				if (len(path) > 0):
					print 'waypoint'
					goto_srv(path[0])
				else:
					print 'spin'
					goto_srv(spin180(currPos))
				rospy.sleep(5)
	
	#Exit Node
	print 'SLAM node exiting'
