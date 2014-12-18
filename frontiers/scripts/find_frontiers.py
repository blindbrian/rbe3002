#!/usr/bin/python
#RBE 3002 Final Project
#Authors: Brian Eccles, Adria Fung, Prateek Sahay
import rospy, math
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import GridCells
from nav_msgs.msg import Path


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

def denormalize(point, width):
	return Point(int(point%width),int(point/width),0)

def normalize(point, width):
	return int(round(point.y*width+point.x))
	
def getNeighbors(current, inmap):
	width = inmap.info.width
	height = inmap.info.height
	astarmap = inmap.data
	n = list()
	if current.x > 0:
		n.append(Point(current.x-1, current.y, 0))
	if current.x < width:
		n.append(Point(current.x+1, current.y, 0))
	if current.y > 0:
		n.append(Point(current.x, current.y-1, 0))
	if current.y < height:
		n.append(Point(current.x, current.y+1, 0))
	if current.y > 0 and current.x < height:
		n.append(Point(current.x+1, current.y-1, 0))
	if current.y < height and current.x > 0:
		n.append(Point(current.x-1, current.y+1, 0))
	if current.y > 0 and current.x > 0:
		n.append(Point(current.x-1, current.y-1, 0))
	if current.y < height and current.x < height:
		n.append(Point(current.x+1, current.y+1, 0))
	return n
	
def isEdge(current, inmap):
	width = inmap.info.width
	height = inmap.info.height
	astarmap = inmap.data
	n = list()
	val = astarmap[int(round(current.y*width + current.x-1))]
	if (val < 50 and val >= 0):
		if current.x > 0 and astarmap[int(round(current.y*width + current.x-1))] == -1:
			return 1
		if current.x < width and astarmap[int(round(current.y*width + current.x+1))] == -1:
			return 2
		if current.y > 0 and astarmap[int(round((current.y-1)*width + current.x))] == -1:
			return 3
		if current.y < height and astarmap[int(round((current.y+1)*width + current.x))] == -1:
			return 4
	return 0
	
def centroid(frontier)
	xc = 0
	yc = 0
	c = 0
	co = [0,0,0,0]
	for p in frontier:
		xc = xc + p.x
		yc = yc + p.y
		co[p.z]++
		c++
	ch = co[2]-co[1]
	cv = co[4]-co[3]
	th = atan2(ch, cv)
	p = Pose()
	p.position = Point(xc/c, yc/c, 0)
	p.orientation.z = sin(th/2)
	return p


def mapCallback(msg):
	global frontier_pub
	global viz_pub
	res = msg.info.resolution
	width = msg.info.width
	height = msg.info.height
	data = msg.data
	frontiers = []
	seen = []
	for cell in range(width*height):
		p = denormalize(cell, width)
		if p not in seen:
			seen.append(p)
			val = isEdge(p, msg)
			if (val):
				stack = []
				front = [Point(p.x,p.y,val)]
				for n in getNeighbors(p, msg):
					if n not in seen:				
						val = isEdge(n, msg)
						if (val):
							stack.append(n)
							front.append(Point(n.x, n.y, val))
						seen.append[p]
				while (len(stack) > 0):
					np = stack.pop()
					for n in getNeighbors(p, msg):
						if n not in seen:				
							val = isEdge(n, msg)
							if (val):
								stack.append(n)
								front.append(Point(n.x, n.y, val))
							seen.append(p)
				frontiers.append(front)
	centroids = map(centroid, frontiers)
	resp = GridCells()
	resp.cell_width = res
	resp.cell_height = res
	resp.header.frame_id = 'map'
	resp.cells = map(lambda x: x.position, centroids)
	viz_pub.publish(resp
	frontier_pub.publish(centroids)
	
	

#Main Function
if __name__ == '__main__':
	#Initialize ROS Node
	rospy.init_node('find_frontiers')
	
	#Publishers
	global frontier_pub
	global viz_pub
	frontier_pub = rospy.Publisher(rospy.get_param('frontier_output_topic', '/frontiers'), Path)
	viz_pub = rospy.Publisher(rospy.get_param('frontier_viz_output_topic', 'visualization/frontiers'), GridCells)
	
	
	#Subscribers
	global map_sub
	map_sub = rospy.Subscriber(rospy.get_param('map_input_topic', '/map'), OccupancyGrid, mapCallback, queue_size=10)

	#Setup Complete
	print 'Lab3 node setup complete'
	
	#Loop Until Shutdown
	while not rospy.is_shutdown():
		pass
	
	#Exit Node
	print 'Lab 3 node exiting'