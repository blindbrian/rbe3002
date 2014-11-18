#RBE 3002 Lab 3
#Authors: Brian Eccles, Adria Fung
import rospy, math
from Queue import PriorityQueue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import GridCells
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

#Callback for turtlebot odometry messages
#param msg: Incoming message of type nav_msgs/Odometry
#returns: nothing
def odometryCallback(msg):
	print msg

#Callback for turtlebot mapping messages
#param msg: Income message of type nav_msgs/OccupancyGrid
#returns: nothing
def mapCallback(msg):
	global map
	global regen_map
	print 'Got new map, regenrating path'
	map = msg
	regen_map = 1
	
#Callback for PoseWithCovarianceStamped
#param msg: Income message of type geometry_msgs/PoseWithCovarianceStamped
#returns: nothing
def posecovarianceCallback(msg):
	global startpos
	global regen_map
	print 'Got new starting position, regenerating path'
	print "x: ", msg.point.x
	print "y: ", msg.point.y
	startpos = msg.point
	regen_map = 1

#Callback for PoseStamped
#param msg: Income message of type geometry_msgs/PoseStamped
#returns: nothing
def posecovarianceCallback(msg):
	global endpos
	global regen_map
	print 'Got new gloal position, regenerating map'
	print "x: ", msg.point.x
	print "y: ", msg.point.y
	endpos = msg.point
	regen_map = 1

def heuristic(p1, p2)
	return math.sqrt(p1*p1+p2*p2)
	
def generatePath(goal, parents)
	global path_pub
	path = Path()
	path_pub.publish(path)
	
def neighbors(current, width, height)
	

#Main Function
if __name__ == '__main__':
	#initialize ros nod
	rospy.init_node('lab3')
	
	#Publishers
	global visited_pub
	global frontier_pub
	global path_pub
	visited_pub = rospy.Publisher('/lab3/astar/visited', GridCells)
	frontier_pub = rospy.Publisher('/lab3/astar/fringe', GridCells)
	path_pub = rospy.Publisher('/lab3/astar/path', Path)
	
	#Subscribers
	global odom_sub
	global map_sub
	global poseco_sub
	global pose_sub
	odom_sub = rospy.Subscriber('/odom', Odometry, odometryCallback, queue_size=10)
	map_sub = rospy.Subscriber('/map', OccupancyGrid, mapCallback, queue_size=10)
	poseco_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, posecovarianceCallback, queue_size=10)
	pose_sub = rospy.Subscriber('/move_base_simple/goal', OccupancyGrid, poseCallback, queue_size=10)

	print 'Lab3 node setup complete'

	global map
	global startpos
	global endpos
	global regen_map
	
	
	while not rospy.is_shutdown():
		if regen_map:
			start = startpos
			goal = endpos
			astarmap = map.data
			mapheight = map.info.height
			mapwidth = map.info.width
			res = map.info.res
			closedset = []
			openset = PriorityQueue()
			parents = dict()
			g_scores = dict()
			f_scores = dict()
			g_scores[start] = 0
			f_score[start] = heuristic(start, goal)
			parents[start] = None
			openset.put(([startPos], start))
			while !openset.empty():
				current = openset.get()
				if current = goal:
					generatePath(parents, goal)				
					break
				closedset.append(current)
				for n in neighbors(current, mapwidth, mapheight):
					c_g_score = g_score[current] + 1
					c_f_score = c_g_score + heuristic(n, goal)
					if n in closedset and c_f_score >= f_score[n]:
						continue
					if not n in openset or c_f_score < f_scores[n]:
						parents[n] = current
						g_score[n] = c_g_score
						f_score[n] = c_f_score
						if not n in openset:
							openset.put((c_f_score, n))
				 visited = GridCells()
				 visited.cell_width = res
				 visited.cell_height = res
				 visited.cells = closedset
				 fringe = GridCells()
				 fringe.cell_height = res
				 fringe.cell_width = res
				 fringe.cells = openset
			print 'A* Done'
		else:
			pass

	print 'Lab 3 node exiting'
