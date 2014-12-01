#!/usr/bin/python
#RBE 3002 Lab 4
#Authors: Brian Eccles, Adria Fung, Prateek Sahay
#This is a simple node that performs obstacle expansion on map

def getNeighbors(map, cell, factor):
	width = map.info.width
	height = map.info.height
	neighhbors = []
	for (x in range(0, factor):
		neighbors.append(map.data[cell
	

#Incoming map message callback
def mapCallback(msg):
	global expanded_map_pub
	expansion_factor = math.ceil(rospy.get_param('expand_by', 1)/msg.info.resolution)
	height = msg.info.height
	width = msg.info width
	new_map = OccupancyGrid()
	new_map.info = map.info
	for cell in range(0, width*height):
		if (msg.data[cell] > 50)
			new_map.data[cell] = 100
		else
			for n in getNeighbors(msg, cell):
				if (n > 
		
	expanded_map_pub.publish(new_map)


if __name__ == '__main__':
	#initialize ros node
	rospy.init_node('ObstacleExpander')
	
	#Publishers
	global expanded_map_pub
	expanded_map_pub = rospy.Publisher(rospy.get_param('output_map_topic', '/expanded_map'), OccupancyGrid)
	
	#Subscribers
	global map_sub
	map_sub = rospy.Subscriber(rospy.get_param('input_map_topic', '/map'), OccupancyGrid, mapCallback, queue_size=10)
	
	#Run until node shuts down
	rospy.spin()