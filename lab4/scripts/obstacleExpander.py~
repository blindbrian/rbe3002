#!/usr/bin/python
#RBE 3002 Lab 4
#Authors: Brian Eccles, Adria Fung, Prateek Sahay
#This is a simple node that performs obstacle expansion on map
import rospy, math
from nav_msgs.msg import OccupancyGrid

def getNeighbors(msg, cell, factor):
	width = msg.info.width
	height = msg.info.height
	neighbors = []
	for x in range(-factor, factor+1):
		for y in range(-factor, factor+1):
			test = cell + x + width*y
			if test > 0 and test < width*height:
				neighbors.append(msg.data[test])
	return neighbors


#Incoming map message callback
def mapCallback(msg):
	print "Got map"
	global expanded_map_pub
	expansion_factor = int(math.ceil(rospy.get_param('expand_by', 1)/msg.info.resolution))
	print expansion_factor
	height = msg.info.height
	width = msg.info.width
	new_map = OccupancyGrid()
	new_map.info = msg.info
	new_map.data = [0 for x in range(width*height)]
	for cell in range(0, width*height):
		for n in getNeighbors(msg, cell, expansion_factor):
			if n > 50:
				new_map.data[cell] = 100
				break
	print 'Publishing new map'
	expanded_map_pub.publish(new_map)


if __name__ == '__main__':
	#initialize ros node
	print 'node started'
	rospy.init_node('ObstacleExpander')
	
	#Publishers
	global expanded_map_pub
	expanded_map_pub = rospy.Publisher(rospy.get_param('output_map_topic', '/expanded_map'), OccupancyGrid, latch=True)
	
	#Subscribers
	global map_sub
	map_sub = rospy.Subscriber(rospy.get_param('input_map_topic', '/map'), OccupancyGrid, mapCallback, queue_size=10)
	
	#Run until node shuts down
	rospy.spin()



