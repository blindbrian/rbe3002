#!/usr/bin/python
#RBE 3002 Lab 3
#Authors: Brian Eccles, Adria Fung, Prateek Sahay
import rospy, math
from Queue import PriorityQueue
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import GridCells
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

#Callback for turtlebot mapping messages
#param msg: Income message of type nav_msgs/OccupancyGrid
#returns: nothing
def mapCallback(msg):
	global map
	global regen_map
	global has_map
	print 'Got new map, regenrating path'
	map = msg
	regen_map = 1
	has_map = 1
	
#Callback for PoseWithCovarianceStamped
#param msg: Income message of type geometry_msgs/PoseWithCovarianceStamped
#returns: nothing
def newStartCallback(msg):
	global startpos
	global regen_map
	global start_pub
	global has_start	
	point = msg.pose.pose.position
	print 'Got new starting position, regenerating path'
	startpos = Point(round(point.x,0),round(point.y,0),0)
	print "x: ", startpos.x
	print "y: ", startpos.y
	start = GridCells()
	start.cell_width = 1
	start.cell_height = 1
	start.cells = [startpos]
	start.header.frame_id = 'map'
	start_pub.publish(start)
	regen_map = 1
	has_start = 1

#Callback for PoseStamped
#param msg: Income message of type geometry_msgs/PoseStamped
#returns: nothing
def newGoalCallback(msg):
	global endpos
	global regen_msg
	global goal_pub	
	global has_goal	
	point = msg.pose.position
	print 'Got new gloal position, regenerating map'
	endpos = Point(round(point.x,0),round(point.y,0),0)
	print "x: ", endpos.x
	print "y: ", endpos.y	
	end = GridCells()
	end.cell_width = 1
	end.cell_height = 1
	end.cells = [endpos]
	end.header.frame_id = 'map'
	goal_pub.publish(end)
	regen_map = 1
	has_goal = 1
    

def heuristic(p1, p2):
	return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)
	
def generatePath(goal, parents, start, width):
	global path_pub
	path = GridCells()
	path.cell_width = 1
	path.cell_height = 1
	path.header.frame_id = 'map'
	cells = [start]
	current = goal
	while current != start:
	    cells.append(current)
	    current = parents[normalize(current, width)]
	path.cells = cells
	rospy.sleep(0.25)
	path_pub.publish(path)
	
def neighbors(current, width, height, astarmap):
	n = list()
	if current.x > 0 and astarmap[int(round(current.y*width + current.x-1))] < 50:
		n.append(Point(current.x-1, current.y, 0))
	if current.x <= width and astarmap[int(round(current.y*width + current.x+1))] < 50:
		n.append(Point(current.x+1, current.y, 0))
	if current.y > 0 and astarmap[int(round((current.y-1)*width + current.x))] < 50:
		n.append(Point(current.x, current.y-1, 0))
	if current.y <= height and astarmap[int(round((current.y+1)*width + current.x))] < 50:
		n.append(Point(current.x, current.y+1, 0))
	return n

def getLowest(openset):
	tup = openset[0]
	lowest = tup[0] 
	for x in openset:
		if x[0] < lowest:
			lowest = x[0]
			tup = x
	return tup

def normalize(point, width):
	return int(round(point.y*width+point.x))

#Main Function
if __name__ == '__main__':
	#initialize ros nod
	rospy.init_node('lab3')


	global map
	global startpos
	global endpos
	global regen_map
	global has_map
	global has_start
	global has_goal
	has_map = 0
	has_start = 0
	has_goal = 0
	
	#Publishers
	global visited_pub
	global frontier_pub
	global path_pub
	global start_pub
	global goal_pub
	visited_pub = rospy.Publisher('/lab3/astar/visited', GridCells)
	frontier_pub = rospy.Publisher('/lab3/astar/fringe', GridCells)
	path_pub = rospy.Publisher('/lab3/astar/path', GridCells)
	start_pub = rospy.Publisher('/lab3/astar/start', GridCells)
	goal_pub = rospy.Publisher('/lab3/astar/goal', GridCells)
	
	#Subscribers
	global map_sub
	global poseco_sub
	global pose_sub
	map_sub = rospy.Subscriber('/map', OccupancyGrid, mapCallback, queue_size=10)
	poseco_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, newStartCallback, queue_size=10)
	pose_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, newGoalCallback, queue_size=10)

	print 'Lab3 node setup complete, waiting for map, start and goal'

	while not (has_map and has_start and has_goal):
		pass
	
	regen_map = 1
	print 'All data received, starting map generation'
	while not rospy.is_shutdown():
		if regen_map:
			print 'Running A*'
			path = GridCells()
			path.cell_width = 1
			path.cell_height = 1
			path.header.frame_id = 'map'
			path_pub.publish(path)
			start = startpos
			goal = endpos
			astarmap = map.data
			mapheight = map.info.height
			mapwidth = map.info.width
			res = map.info.resolution
			closedset = []
			openset = []
			parents = dict()
			g_scores = dict()
			f_scores = dict()
			g_scores[normalize(start, mapwidth)] = 0
			f_scores[normalize(start, mapwidth)] = heuristic(start, goal)
			parents[normalize(start, mapwidth)] = None
			openset.append((f_scores[normalize(start, mapwidth)], start))
			while len(openset) > 0:
				current_tup = getLowest(openset)
				openset.remove(current_tup)
				current = current_tup[1]
				if current == goal:
					generatePath(goal, parents, start, mapwidth)		
					break
				closedset.append(current)
				for n in neighbors(current, mapwidth, mapheight, astarmap):
					c_g_score = g_scores[normalize(current, mapwidth)] + 1
					c_f_score = c_g_score + heuristic(n, goal)
					if n in closedset and c_f_score >= f_scores[normalize(n, mapwidth)]:
						continue
					if not n in (x[1] for x in openset):
						parents[normalize(n, mapwidth)] = current
						g_scores[normalize(n, mapwidth)] = c_g_score
						f_scores[normalize(n, mapwidth)] = c_f_score
						openset.append((c_f_score, n))
					elif c_f_score < f_scores[normalize(n, mapwidth)]:
						parents[normalize(n, mapwidth)] = current
						g_scores[normalize(n, mapwidth)] = c_g_score
						f_scores[normalize(n, mapwidth)] = c_f_score
				visited = GridCells()
				visited.cell_width = res
				visited.cell_height = res
				visited.cells = closedset
				visited.header.frame_id = 'map'
				visited_pub.publish(visited)
				fringe = GridCells()
				fringe.cell_height = res
				fringe.cell_width = res
				fringe.cells = [x[1] for x in openset]
		                fringe.header.frame_id = 'map'
				frontier_pub.publish(fringe)
				startm = GridCells()
				startm.cell_width = 1
				startm.cell_height = 1
				startm.cells = [start]
				startm.header.frame_id = 'map'	
				start_pub.publish(startm)
			print 'A* Done'
			regen_map = 0
		else:
			pass

	print 'Lab 3 node exiting'
