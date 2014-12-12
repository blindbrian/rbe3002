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

#Callback for map messages
#param msg: Income message of type nav_msgs/OccupancyGrid
#returns: nothing
def mapCallback(msg):
	global map
	global regen_map
	global has_map
	print 'Got new map, regenrating path'
	#set global map
	map = msg
	#trigger A* path regeneration
	regen_map = 1
	#indicate that we have received a map
	has_map = 1
	
#Callback for start point messages
#param msg: Income message of type geometry_msgs/PoseWithCovarianceStamped
#returns: nothing
def newStartCallback(msg):
	global startpos
	global regen_map
	global start_pub
	global has_start
	#extract point from message
	point = msg.pose.pose.position
	print 'Got new starting position, regenerating path'
	#round point values to nearest integer
	startpos = Point(round(point.x,0),round(point.y,0),0)
	print "x: ", startpos.x
	print "y: ", startpos.y
	#send rounded start point as a GridCells message to /lab3/astar/start
	start = GridCells()
	start.cell_width = 1
	start.cell_height = 1
	start.cells = [startpos]
	start.header.frame_id = 'map'
	start_pub.publish(start)
	#trigger A* path regeneration
	regen_map = 1
	#indicate that we have receive a start position
	has_start = 1

#Callback for goal point messages
#param msg: Income message of type geometry_msgs/PoseStamped
#returns: nothing
def newGoalCallback(msg):
	global endpos
	global regen_msg
	global goal_pub	
	global has_goal	
	#extract point from message
	point = msg.pose.position
	print 'Got new gloal position, regenerating map'
	#round point values to nearest integer
	endpos = Point(round(point.x,0),round(point.y,0),0)
	print "x: ", endpos.x
	print "y: ", endpos.y	
	#send rounded goal point as a GridCells message to /lab3/astar/goal
	end = GridCells()
	end.cell_width = 1
	end.cell_height = 1
	end.cells = [endpos]
	end.header.frame_id = 'map'
	goal_pub.publish(end)
	#trigger A* path regeneration
	regen_map = 1
	#indicate that we have received a goal position
	has_goal = 1
    
#A* Heursitic function
#param p1: current position
#param p2: goal position
#returns: distance between p1 and p2
def heuristic(p1, p2):
	return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)
	
#Find the path from start to goal given the parents array from A*
#param goal: goal position
#param parents: Map of each point to its parent in the path A* found
#param start: start position
#param width: width of map (for normalizing points)
#returns: nothing
def generatePath(goal, parents, start, width):
	global path_pub
	global way_pub
	#Create GridCells() message to display path and waypoints in rviz
	path = GridCells()
	path.cell_width = 1
	path.cell_height = 1
	path.header.frame_id = 'map'
	way = GridCells()
	way.cell_width = 1
	way.cell_height = 1
	way.header.frame_id = 'map'
	waycells = [start]
	#Create a list of cells starting with the start position
	cells = [start]
	#trace path from goal back to start
	current = goal
	last_ang = 0
	ang = 0
	lastpt = goal
	while current != start:
	    cells.append(current)
	    #if we change travel direction, add a waypoint
	    if (heuristic(lastpt,current) != 0):
	    	ang = math.atan2((current.y-lastpt.y),(current.x-lastpt.x))
	    if (abs(ang-last_ang) > 0.1):
		waycells.append(lastpt)
	    last_ang = ang
	    lastpt = current
	    current = parents[normalize(current, width)]
	path.cells = cells
	way.cells = waycells
	path_pub.publish(path)
	way_pub.publish(way)

#Generates a list of neighbors for a given point
#param current: current point
#param width: width of the map
#param heigh: height of the map
#param astarmap: the actual map
#returns: a list of unoccupied neighboring points
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
	if current.y > 0 and current.x <= height and astarmap[int(round((current.y-1)*width + current.x+1))] < 50:
		n.append(Point(current.x+1, current.y-1, 0))
	if current.y <= height and current.x > 0 and astarmap[int(round((current.y+1)*width + current.x-1))] < 50:
		n.append(Point(current.x-1, current.y+1, 0))
	if current.y > 0 and current.x > 0 and astarmap[int(round((current.y-1)*width + current.x-1))] < 50:
		n.append(Point(current.x-1, current.y-1, 0))
	if current.y <= height and current.x <= height and astarmap[int(round((current.y+1)*width + current.x+1))] < 50:
		n.append(Point(current.x+1, current.y+1, 0))
	return n

#Get the lowest f_score point in the given list
#param openset: list of 2-tuples (f_score, point)
#returns: the tuple in openset with the lowest f_score
def getLowest(openset):
	tup = openset[0]
	lowest = tup[0] 
	for x in openset:
		if x[0] < lowest:
			lowest = x[0]
			tup = x
	return tup

#Converts a point into an single integer given the width of the map
#param point: point to convert
#param width: width of the map the point is on
#return: An integer representation of the point for use as a dictionary key
def normalize(point, width):
	return int(round(point.y*width+point.x))

#Main Function
if __name__ == '__main__':
	#initialize ros nod
	rospy.init_node('lab3')

	#setup globals
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
	global way_pub
	visited_pub = rospy.Publisher('/lab3/astar/visited', GridCells)
	frontier_pub = rospy.Publisher('/lab3/astar/fringe', GridCells)
	path_pub = rospy.Publisher('/lab3/astar/path', GridCells)
	start_pub = rospy.Publisher('/lab3/astar/start', GridCells)
	goal_pub = rospy.Publisher('/lab3/astar/goal', GridCells)
	way_pub = rospy.Publisher('/lab3/astar/way', GridCells)
	
	#Subscribers
	global map_sub
	global poseco_sub
	global pose_sub
	map_sub = rospy.Subscriber('/map', OccupancyGrid, mapCallback, queue_size=10)
	poseco_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, newStartCallback, queue_size=10)
	pose_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, newGoalCallback, queue_size=10)

	#setup complete
	print 'Lab3 node setup complete, waiting for map, start and goal'

	#wait to receive a map, start point and goal point
	while not (has_map and has_start and has_goal):
		pass
	
	#trigger first path generation
	regen_map = 1
	print 'All data received, starting map generation'
	#loop until shutdown
	while not rospy.is_shutdown():
		#if path needs regeneration
		if regen_map:
			#setup A* with the starting position on the fringe
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
			while len(openset) > 0: #continue until there is nothing left on the fringe
				#find the position on the fringe with the lowest f_score and pull it off
				current_tup = getLowest(openset)
				openset.remove(current_tup)
				current = current_tup[1]
				#check if we've reached the goal
				if current == goal:
					generatePath(goal, parents, start, mapwidth)		
					break
				#mark current as visited
				closedset.append(current)
				#iterate through all neighbors of currrent
				for n in neighbors(current, mapwidth, mapheight, astarmap):
					#calculate each neighbors g and f score
					c_g_score = g_scores[normalize(current, mapwidth)] + heuristic(current, n)
					c_f_score = c_g_score + heuristic(n, goal)
					#skip it if it has been visited and we havn't found a faster path
					if n in closedset and c_f_score >= f_scores[normalize(n, mapwidth)]:
						continue
					if not n in (x[1] for x in openset): #if it hasn't been visited
						#take down its g and f scores and parent, then add it to the fringe
						parents[normalize(n, mapwidth)] = current
						g_scores[normalize(n, mapwidth)] = c_g_score
						f_scores[normalize(n, mapwidth)] = c_f_score
						openset.append((c_f_score, n))
					elif c_f_score < f_scores[normalize(n, mapwidth)]: #its already on the fringe and this path is faster, update its scores and parent
						parents[normalize(n, mapwidth)] = current
						g_scores[normalize(n, mapwidth)] = c_g_score
						f_scores[normalize(n, mapwidth)] = c_f_score
				#send visulaization data to rviz
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
	
	#Exit Node
	print 'Lab 3 node exiting'
