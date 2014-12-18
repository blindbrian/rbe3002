#!/usr/bin/python
#RBE 3002 Lab 3
#Authors: Brian Eccles, Adria Fung, Prateek Sahay
import rospy, math
from Queue import PriorityQueue
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import GridCells
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from pathfinder.srv import GetPath

#Callback for map messages
#param msg: Income message of type nav_msgs/OccupancyGrid
#returns: nothing
def mapCallback(msg):
	global curmap
	#set global map
	curmap = msg

    
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
def generatePath(cmap, start, goal, parents)
	global path_pub
	global way_pub
	global curmap
	#Create GridCells() message to display path and waypoints in rviz
	path = GridCells()
	path.cell_width = cmap.info.resolution
	path.cell_height = cmap.info.resolution
	path.header.frame_id = 'map'
	way = GridCells()
	way.cell_width = cmap.info.resolution
	way.cell_height = cmap.info.resolution
	way.header.frame_id = 'map'
	waycells = [mapToWorldPos(cmap, goal)]
	#Create a list of cells starting with the start position
	cells = [mapToWorldPos(cmap, start)]
	#trace path from goal back to start
	current = parents[normalize(goal, cmap.info.width)]
	lastpt = goal
	last_ang = math.atan2((current.y-lastpt.y),(current.x-lastpt.x))
	ret = []
	p = Pose()
	while current != start:
	    cells.append(mapToWorldPos(cmap, current))
	    #if we change travel direction, add a waypoint
	    ang = math.atan2((current.y-lastpt.y),(current.x-lastpt.x))
	    if (abs(ang-last_ang) > 0.1):
			waycells.append(mapToWorldPos(cmap, lastpt))
			waycells.append(mapToWorldPos(cmap, lastpt))
			p.position = mapToWorldPos(cmap, lastpt))
			p.orientation.z = math.sin(ang/2)
			ret.append(p)
			p = Pose()
	    last_ang = ang
	    lastpt = current
	    current = parents[normalize(current, cmap.info.width)]
	path.cells = cells
	way.cells = list(reversed(waycells))
	path_pub.publish(path)
	way_pub.publish(way)
	return ret

#Generates a list of neighbors for a given point
#param current: current point
#param width: width of the map
#param heigh: height of the map
#param astarmap: the actual map
#returns: a list of unoccupied neighboring points
def neighbors(current, inmap):
	width = inmap.info.width
	height = inmap.info.height
	astarmap = inmap.data
	n = list()
	if current.x > 0 and astarmap[int(round(current.y*width + current.x-1))] < 50:
		n.append(Point(current.x-1, current.y, 0))
	if current.x < width and astarmap[int(round(current.y*width + current.x+1))] < 50:
		n.append(Point(current.x+1, current.y, 0))
	if current.y > 0 and astarmap[int(round((current.y-1)*width + current.x))] < 50:
		n.append(Point(current.x, current.y-1, 0))
	if current.y < height and astarmap[int(round((current.y+1)*width + current.x))] < 50:
		n.append(Point(current.x, current.y+1, 0))
	if current.y > 0 and current.x < height and astarmap[int(round((current.y-1)*width + current.x+1))] < 50:
		n.append(Point(current.x+1, current.y-1, 0))
	if current.y < height and current.x > 0 and astarmap[int(round((current.y+1)*width + current.x-1))] < 50:
		n.append(Point(current.x-1, current.y+1, 0))
	if current.y > 0 and current.x > 0 and astarmap[int(round((current.y-1)*width + current.x-1))] < 50:
		n.append(Point(current.x-1, current.y-1, 0))
	if current.y < height and current.x < height and astarmap[int(round((current.y+1)*width + current.x+1))] < 50:
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

#takes a map and a floating point position in the world and returns the map cell the position is in	
def worldToMapCell(curmap, point):
	res = curmap.info.resolution
	ret = Point()
	ret.x = int((point.x - curmap.info.origin.position.x)/res) 
	ret.y = int((point.y - curmap.info.origin.position.y)/res)
	return ret

#takes a map cell and returns the center of the cell in world coordinates
def mapToWorldPos(curmap, point):
	res = curmap.info.resolution
	ret = Point()
	ret.x = ((point.x*res) + curmap.info.origin.position.x) + (res/2);
	ret.y = ((point.y*res) + curmap.info.origin.position.y) + (res/2);
	return ret
	
def getPath(msg):
	global curmap
	StartPose = msg.start
	GoalPose = msg.goal
	InputMap = curmap
	StartCell = worldToMapCell(InputMap, StartPose.position)
	GoalCell = worldToMapCell(InputMap, GoalPose.position)	
	print 'Running A*'
	if (not StartCell == GoalCell):
		res = InputMap.info.resolution
		width = InputMap.info.width
		closedset = []
		openset = []
		parents = dict()
		g_scores = dict()
		f_scores = dict()
		g_scores[normalize(StartCell, width)] = 0
		f_scores[normalize(StartCell, width)] = heuristic(StartCell, GoalCell)
		parents[normalize(start, mapwidth)] = None
		openset.append((f_scores[normalize(StartCell, width)], StartCell))
		while len(openset) > 0: #continue until there is nothing left on the fringe
			#find the position on the fringe with the lowest f_score and pull it off
			current_tup = getLowest(openset)
			openset.remove(current_tup)
			current = current_tup[1]
			#check if we've reached the goal
			if current == GoalCell:
				return generatePath(InputMap, StartCell, GoalCell, parents)		
			#mark current as visited
			closedset.append(current)
			#iterate through all neighbors of currrent
			for n in neighbors(current, InputMap):
				#calculate each neighbors g and f score
				c_g_score = g_scores[normalize(current, width)] + heuristic(current, n)
				c_f_score = c_g_score + heuristic(n, GoalCell)
				#skip it if it has been visited and we havn't found a faster path
				if n in closedset and c_f_score >= f_scores[normalize(n, mapwidth)]:
					continue
				if not n in (x[1] for x in openset): #if it hasn't been visited
					#take down its g and f scores and parent, then add it to the fringe
					parents[normalize(n, width)] = current
					g_scores[normalize(n, width)] = c_g_score
					f_scores[normalize(n, width)] = c_f_score
					openset.append((c_f_score, n))
				elif c_f_score < f_scores[normalize(n, mapwidth)]: #its already on the fringe and this path is faster, update its scores and parent
					parents[normalize(n, width)] = current
					g_scores[normalize(n, width)] = c_g_score
					f_scores[normalize(n, width)] = c_f_score
			#send visulaization data to rviz
			visited = GridCells()
			visited.cell_width = res
			visited.cell_height = res
			visited.cells = map(lambda x: mapToWorldPos(astarmap, x), closedset)
			visited.header.frame_id = 'map'
			visited_pub.publish(visited)
			fringe = GridCells()
			fringe.cell_height = res
			fringe.cell_width = res
			fringe.cells = map(lambda x: mapToWorldPos(astarmap, x), [x[1] for x in openset])
			fringe.header.frame_id = 'map'
			frontier_pub.publish(fringe)
		return []

#Main Function
if __name__ == '__main__':
	#Initialize ROS Node
	rospy.init_node('pathfinder')

	#Setup globals
	global curmap #Current map of the environment
	
	#Publishers
	global visited_pub
	global frontier_pub
	global path_pub
	global start_pub
	global goal_pub
	global way_pub
	visited_pub = rospy.Publisher('visualize/visited', GridCells)
	frontier_pub = rospy.Publisher('visualize/fringe', GridCells)
	path_pub = rospy.Publisher('visualize/path', GridCells)
	start_pub = rospy.Publisher('visualize/start', GridCells)
	goal_pub = rospy.Publisher('visualize/goal', GridCells)
	way_pub = rospy.Publisher('visualize/way', GridCells)
	
	#Subscribers
	global map_sub
	global getpath_srv
	map_sub = rospy.Subscriber(rospy.get_param('map_input_topic', '/map'), OccupancyGrid, mapCallback, queue_size=10)
	getpath_srv = rospy.Service(rospy.get_param('getpath_service', '/getpath'), GetPath, getPath)

	#Setup Complete
	print 'Pathfinder node setup complete'
	
	#Loop Until Shutdown
	while not rospy.is_shutdown():
		pass
	
	#Exit Node
	print 'Pathfinder 3 node exiting'
