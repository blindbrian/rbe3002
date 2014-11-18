#RBE 3002 Lab 3
#Authors: Brian Eccles, Adria Fung, Prateek Sahay
import rospy, math
from Queue import PriorityQueue
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import GridCells
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped

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
	global start_pub
	print 'Got new starting position, regenerating path'
	print "x: ", msg.point.x
	print "y: ", msg.point.y
	startpos = msg.point
	regen_map = 1
	start = GridCells()
	start.cell_width = 0.2
	start.cell_height = 0.2
	start.cells = [endpos]
	goal_pub.publish(start)

#Callback for PoseStamped
#param msg: Income message of type geometry_msgs/PoseStamped
#returns: nothing
def posecovarianceCallback(msg):
	global endpos
	global regen_msg
	global goal_pub	
	print 'Got new gloal position, regenerating map'
	print "x: ", msg.point.x
	print "y: ", msg.point.y
	endpos = msg.point
	regen_map = 1
	start = GridCells()
	start.cell_width = 0.2
	start.cell_height = 0.2
	start.cells = [endpos]
	goal_pub.publish(start)
    

def heuristic(p1, p2):
	return math.sqrt(p1*p1+p2*p2)
	
def generatePath(goal, parents, start):
	global path_pub
	path = Path()
	current = goal
	while current != start
	    pose = PoseStamped()
	    pose.point = current
	    path.append(pose)
	    current = parents[current]
	pose = PoseStamped()
	pose.point = start
	path.append(pose)
	path_pub.publish(path)
	
def neighbors(current, width, height, astarmap):
    n = list()
    if (current.x > 0 and astarmap[current.y*width + current.x] < 50):
        n.append(Point(x-1, y))
    if (current.x <= width and astarmap[current.y*width + current.x] < 50):
        n.append(Point(x+1), y))
    if (current.y > 0 and astarmap[current.y*width + current.x] < 50):
        n.append(Point(x, y-1))
    if (current.y <= height and astarmap[current.y*width + current.x] < 50):
        n.append(Point(x, y+1))
    return n
	

#Main Function
if __name__ == '__main__':
    print 'HI WILCOX'
	#initialize ros nod
	rospy.init_node('lab3')
	
	#Publishers
	global visited_pub
	global frontier_pub
	global path_pub
	global start_pub
	global goal_pub
	visited_pub = rospy.Publisher('/lab3/astar/visited', GridCells)
	frontier_pub = rospy.Publisher('/lab3/astar/fringe', GridCells)
	path_pub = rospy.Publisher('/lab3/astar/path', Path)
	start_pub = rospy.Publisher('/lab3/astar/start', GridCells)
	goal_pub = rospy.Publisher('/lab3/astar/goal', GridCells)
	
	#Subscribers
	global map_sub
	global poseco_sub
	global pose_sub
	map_sub = rospy.Subscriber('/map', OccupancyGrid, mapCallback, queue_size=10)
	poseco_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, posecovarianceCallback, queue_size=10)
	pose_sub = rospy.Subscriber('/move_base_simple/goal', OccupancyGrid, poseCallback, queue_size=10)

	print 'Lab3 node setup complete'

	global map
	global startpos
	global endpos
	global regen_map
	regen_map = 0
	startpos = Point(0,0)
	endpos = Point(0,0)
	
	
	while not rospy.is_shutdown():
		if regen_map:
		    print 'Remake Map'
			"""start = startpos
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
					generatePath(parents, goal, start)		
					break
				closedset.append(current)
				for n in neighbors(current, mapwidth, mapheight, astarmap):
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
				 fringe.cells = openset"""
			print 'A* Done'
		else:
			pass

	print 'Lab 3 node exiting'
