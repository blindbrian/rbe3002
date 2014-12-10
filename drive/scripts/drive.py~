#!/usr/bin/env python
import rospy, tf, math 
from Queue import Queue
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
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
    	global actionqueue
	actionqueue.put((speed, speed, distance, 0))    

    
##Enqueues a command to rotate the robot in place through a given angle
#param angle: Angle to rotate through in radians (positive angles rotate counter-clockwise, negative angles rotate clockwise)
#returns: nothing
#note: Will rotate at ~2 radians per second
def rotate(speed, angle):
    	global actionqueue
	if angle > 0:
		actionqueue.put((-speed, speed, 0, angle))
	else:
		actionqueue.put((speed, -speed, 0, angle))


##Turtlebot odometry message callback; executes enqueued driving commands
#param msg: Message
def read_odometry(msg): 
	global standby
	global actionqueue
	global deltaAng
	global deltaDist
	global action
	global lastX
	global lastY
	global lastTh
	global stop
	#standby mode is entered whenever a command finishes
	if standby:
		lastX = msg.pose.pose.position.x
		lastY = msg.pose.pose.position.y
		lastTh = 2*math.asin(msg.pose.pose.orientation.z)
		#pull the next command off the queue if one is available
		if not actionqueue.empty():
			#setup for the new command
			standby = 0
			action = actionqueue.get()
			deltaDist = 0
			deltaAng = 0
			#pull current robot position and orientation from odometry message
			print 'Run command: ',  action
	else: #execute active command 
		#calculate delta distance traveld and angle change
		deltaX = msg.pose.pose.position.x - lastX
		deltaY = msg.pose.pose.position.y - lastY
		currTh = 2*math.asin(msg.pose.pose.orientation.z) 
		#orientation provided by odomentry returns values from -pi to pi, this makes sure we don't add 2pi to the delta if that boundary is crossed
		if (currTh == 0 or lastTh == 0 or currTh/math.fabs(currTh) == lastTh/math.fabs(lastTh)):
			deltaTh = currTh - lastTh
		else:
			deltaTh = 0
		deltaDist = deltaDist + math.sqrt(deltaX*deltaX + deltaY*deltaY)
		deltaAng = deltaAng + deltaTh
		lastX = msg.pose.pose.position.x
		lastY = msg.pose.pose.position.y
		lastTh = 2*math.asin(msg.pose.pose.orientation.z)
		#maintain the command's wheels speed
		spinWheels(action[0], action[1])
		#wait for desired delta distance or angle
		if (stop or action[3] == 0 and deltaDist >= action[2]) or (action[2] == 0 and math.fabs(deltaAng) >= math.fabs(action[3])):
			#Stop the robot and enter standby to wait for next command
			spinWheels(0,0)
			standby = 1
			stop = 0
			print 'Command done"

	

#Path received callback
#param msg: GridCells message indicating a path to travel	
def read_goal(msg):
	global startpos
	global actionqueue
	global stop
	global lastX
	global lastY
	global lastTh
	global start_actual
	global standby
	while not actionqueue.empty():
		actionqueue.get()
	if not standby:
		stop = 1
	path = msg.cells
	if (start_actual):
		x = lastX
		y = lastY
		th = lastTh
	else:
		x = startpos.position.x
		y = startpos.position.y
		th = 2*math.asin(startpos.orientation.z)
	print "Current pos ", x, y, th
	for p in path:
		d = math.sqrt((p.x-x)**2+(p.y-y)**2)
		a = math.atan2(p.y-y, p.x-x)
		r = th-a
		if (r > math.pi):
			r = r-(2*math.pi)
		if (r < -math.pi):
			r = r+(2*math.pi)
		print "Turn:", r, "Drive:", d
		rotate(0.1, r)
		driveStraight(0.05, d)
		x = p.x
		y = p.y
		th = a



# This is the program's main function
if __name__ == '__main__':
    #Initialize ros node
    rospy.init_node('beccles_lab2')

    # These are global variables. Write "global <variable_name>" in any other function
    #  to gain access to these global variables    
    global teleop_pub
    global pose
    global odom_tf
    global odom_list
    global standby
    global actionqueue
    global lastTh
    global lastX
    global lastY
    global startpos
    global start_actual
    global stop
    start_actual = True
    startpos = Pose()
    lastX = 0
    lastY = 0
    lastTh = 0
    standby = 1
    stop = 0
    #actionqueue is a queue of actions
    #an action is a 4-tuple that represents a command
    #Format: (right_wheel_speed, left_wheel_speed, distance_to_travel, angle_to_travel_through)
    actionqueue = Queue()

    teleop_pub = rospy.Publisher(rospy.get_param('drive_output_topic', '/cmd_vel_mux/input/teleop'), Twist) # Publisher for commanding robot motion
    odom_sub = rospy.Subscriber(rospy.get_param('odometry_input_topic', '/odom'), Odometry, read_odometry, queue_size=1) # Callback function to read in robot Odometry messages
    goal_sub = rospy.Subscriber(rospy.get_param('goal_input_topic', '/goal'), Pose, read_goal, queue_size=10)

    print "Starting Drive Node"
    
    timer = Rate(100)
    #main loop, wait for ros shutdown to exit
    while not rospy.is_shutdown():
	
	timer.sleep()

    print "Exiting Drive Node"
    
