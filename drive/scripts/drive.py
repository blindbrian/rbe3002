#!/usr/bin/env python
import rospy, math 
from Queue import Queue
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rbe_srvs.srv import Goto
from rbe_srvs.srv import GotoResponse
#from drive.srv import Goto
# Add additional imports for each of the message types used


##This function sets the turtle bot wheels to drive at the given velocieits
#param u1: Right wheel velocity
#param u2: Left wheel velocity
#return: Nothing
#notes: This function must be called at least once per second to maintain turtlebot speed, otherwise the wheels will stop
def spinWheels(u1, u2):
	global teleop_pub
	u = (u1+u2)/2
	omega = (u1-u2)/0.2286
	twist_msg = Twist()
	twist_msg.linear.x = u
	twist_msg.linear.y = 0
	twist_msg.linear.z = 0
	twist_msg.angular.x = 0
	twist_msg.angular.y = 0
	twist_msg.angular.z = omega
	teleop_pub.publish(twist_msg)


##Enqueues a command to drive the robot straight at a given speed for a given distance
#param speed: Speed to drive straight at in meters per second(max ~0.5)
#param distance: Distance to drive in meters
#returns: Nothing 
def driveStraight(speed, distance):
	global currpose
	sx = currpose.position.x
	sy = currpose.position.y
	cx = sx
	cy = sy
	spinWheels(speed, speed)
	while (math.sqrt((cx-sx)**2+(cy-sy)**2) < distance):
		spinWheels(speed, speed)
		cx = currpose.position.x
		cy = currpose.position.y
	spinWheels(0,0)
	

    
##Enqueues a command to rotate the robot in place through a given angle
#param angle: Angle to rotate through in radians (positive angles rotate counter-clockwise, negative angles rotate clockwise)
#returns: nothing
#note: Will rotate at ~2 radians per second
def rotate(speed, angle):
	global currpose
	currTh = 2*math.asin(currpose.orientation.z)
	r = currTh-angle
	if (r > math.pi):
		r = r-(2*math.pi)
	if (r < -math.pi):
		r = r+(2*math.pi)
	while math.fabs(2*math.asin(currpose.orientation.z) - angle) > 0.1:
		if r > 0:
			spinWheels(-speed, speed)
		else:
			spinWheels(speed, -speed)
	spinWheels(0,0)
		


##Turtlebot odometry message callback; executes enqueued driving commands
#param msg: Message
def read_odometry(msg): 
	global currpose
	currpose = msg.pose.pose


def goto(msg):
	global currpose
	print "Drive to:", msg.pose.position
	print "Current:", currpose.position
	cx = currpose.position.x
	cy = currpose.position.y
	gx = msg.pose.position.x
	gy = msg.pose.position.y
	gth = 2*math.asin(msg.pose.orientation.z)
	r = math.atan2(gy-cy, gx-cx) 
	d = math.sqrt((gx-cx)**2+(gy-cy)**2)
	print d, r
	print 'r'
	rotate(0.05, r)
	print 'd'
	driveStraight(0.1, d)
	print 'r'
	rotate(0.05, gth)
	print 'done'
	return GotoResponse()



# This is the program's main function
if __name__ == '__main__':
    #Initialize ros node
    rospy.init_node('beccles_lab2')

    # These are global variables. Write "global <variable_name>" in any other function
    #  to gain access to these global variables    
    global teleop_pub
    global odom_sub
    global goal_srv
    global currpose

    teleop_pub = rospy.Publisher(rospy.get_param('drive_output_topic', '/cmd_vel_mux/input/teleop'), Twist) # Publisher for commanding robot motion
    odom_sub = rospy.Subscriber(rospy.get_param('odometry_input_topic', '/odom'), Odometry, read_odometry, queue_size=1) # Callback function to read in robot Odometry messages
    goal_srv = rospy.Service(rospy.get_param('goto_service', '/goto'), Goto, goto)

    print "Starting Drive Node" 
    #main loop, wait for ros shutdown to exit
    while not rospy.is_shutdown():
		pass

    print "Exiting Drive Node"
    
