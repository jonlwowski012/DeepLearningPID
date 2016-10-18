#!/usr/bin/env python

# Title: Meta-Cluster Location Publisher
# Description: Clusters the clusters into Metaclusters
# Engineer: Jonathan Lwowski 
# Email: jonathan.lwowski@gmail.com
# Lab: Autonomous Controls Lab, The University of Texas at San Antonio


#########          Libraries         ###################
import sys
from std_msgs.msg import String
from std_msgs.msg import Header
import rospy
import math
import numpy as np
import time
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sklearn import cluster
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from cooperative_mav_asv.msg import *
from cooperative_mav_asv.srv import *


### Functions to Find Smalled Bounding Circle  ###

def make_circle(points):
    # Convert to float and randomize order
    shuffled = [(float(p[0]), float(p[1])) for p in points]
 
    
    # Progressively add points to circle or recompute circle
    c = None
    for (i, p) in enumerate(shuffled):
        if c is None or not _is_in_circle(c, p):
            c = _make_circle_one_point(shuffled[0 : i + 1], p)
    return c


# One boundary point known
def _make_circle_one_point(points, p):
    c = (p[0], p[1], 0.0)
    for (i, q) in enumerate(points):
        if not _is_in_circle(c, q):
            if c[2] == 0.0:
                c = _make_diameter(p, q)
            else:
                c = _make_circle_two_points(points[0 : i + 1], p, q)
    return c


# Two boundary points known
def _make_circle_two_points(points, p, q):
    diameter = _make_diameter(p, q)
    if all(_is_in_circle(diameter, r) for r in points):
        return diameter
    
    left = None
    right = None
    for r in points:
        cross = _cross_product(p[0], p[1], q[0], q[1], r[0], r[1])
        c = _make_circumcircle(p, q, r)
        if c is None:
            continue
        elif cross > 0.0 and (left is None or _cross_product(p[0], p[1], q[0], q[1], c[0], c[1]) > _cross_product(p[0], p[1], q[0], q[1], left[0], left[1])):
            left = c
        elif cross < 0.0 and (right is None or _cross_product(p[0], p[1], q[0], q[1], c[0], c[1]) < _cross_product(p[0], p[1], q[0], q[1], right[0], right[1])):
            right = c
    return left if (right is None or (left is not None and left[2] <= right[2])) else right


def _make_circumcircle(p0, p1, p2):
    # Mathematical algorithm from Wikipedia: Circumscribed circle
    ax = p0[0]; ay = p0[1]
    bx = p1[0]; by = p1[1]
    cx = p2[0]; cy = p2[1]
    d = (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by)) * 2.0
    if d == 0.0:
        return None
    x = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d
    y = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d
    return (x, y, math.hypot(x - ax, y - ay))


def _make_diameter(p0, p1):
    return ((p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0, math.hypot(p0[0] - p1[0], p0[1] - p1[1]) / 2.0)


_EPSILON = 1e-12

def _is_in_circle(c, p):
    return c is not None and math.hypot(p[0] - c[0], p[1] - c[1]) < c[2] + _EPSILON


# Returns twice the signed area of the triangle defined by (x0, y0), (x1, y1), (x2, y2)
def _cross_product(x0, y0, x1, y1, x2, y2):
    return (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0)


def kmeans_clustering(data):
	clusters = data.clusters
	location_array = []
	if(data != []):
				### Store clusters x&y poses into array
				#location_array = [(data.clusters[0].pose.position.x,data.clusters[0].pose.position.y)]
				for i in range(len(data.clusters)):
					location_array.append((data.clusters[i].pose.position.x,data.clusters[i].pose.position.y))
				
				### K-means clustering on Clusters				
				k = 2
				kmeans = cluster.KMeans(n_clusters=k)
				kmeans.fit(location_array)
				centroids = kmeans.cluster_centers_	
				inertia = kmeans.inertia_
				labels = kmeans.labels_
	return centroids,labels,location_array



def publish_locations(centroids,labels,location_array,clusters):

	### Setup publishers
	pub1 = rospy.Publisher('/metaclusters_locations', MetaClusters, queue_size=10)
	pub2 = rospy.Publisher('/metacluster_locations_rviz', PoseArray, queue_size=10)
	r = rospy.Rate(30) # 1hz

	### Init Variables
	posearr2 = []
	command2 = PoseArray()
	command2.header.stamp = rospy.get_rostime()
	command2.header.frame_id = 'world'
	posearr = []
	command = MetaClusters()
	#command.header.stamp = rospy.get_rostime()
	#command.header.frame_id = 'world'

	### For all the metacluster centers
	if len(centroids) > 0 :
		for i in range(len(centroids)):
			### Store Metacluster info into Msgs
			meta_cluster = MetaCluster()
			meta_cluster.label = str(i)
			pose = Pose()
			quat = Quaternion()
			point = Point()
			point.x = centroids[i][0]
			point.y = centroids[i][1]
			point.z = 0
			quat.x = 0
			quat.y = 0
			quat.z = 0
			quat.w = 0
			pose.position=point
			pose.orientation=quat
			meta_cluster.pose=pose
			posearr2.append(pose)
			meta_cluster.clusters=ClusterCenters()
			points = []

			### Link Clusters into Metaclusters
			#print "clusters " , len(clusters)
			#print "labels ", len(labels)
			#print "location_array ", len(location_array)
			for j in range(len(clusters)):
				try:
					if(labels[j] == i):
						#print ("lables[j]: ", labels[j])
						points.append((clusters[j].pose.position.x,clusters[j].pose.position.y))
						meta_cluster.clusters.clusters.append(clusters[j])
						meta_cluster.people = meta_cluster.people + clusters[j].number_people
				except:	
					print "Index Error"

			### Find smallest bounding circle
			circle = make_circle(points)
			#print "circle: ", len(circle)
			if circle is not None:
				meta_cluster.radius = circle[2]
			else:
				meta_cluster.radius = 0
			#print "radius for label ", i, ": ", meta_cluster.radius
			command.metaclusters.append(meta_cluster)
		command2.poses=posearr2
		#print command.metaclusters[0].clusters

		### Publish Metaclusters
		#pub1.publish(command)
		#pub2.publish(command2)
		return command.metaclusters,posearr2

def handle_metaclustering(req):
	time.sleep(2)
	### Init Variables
	command = MetaClusters()
	command2 = PoseArray()
	command2.header.stamp = rospy.get_rostime()
	command2.header.frame_id = 'world'

	### Get Clusters using Service
	clusters = call_clustering_service_client()
	#print "clusters: ", clusters.clusters

	### Perform K-means Clustering
	centroids,labels,location_array = kmeans_clustering(clusters.clusters)

	### Setup Service Response Messages
	metaclusters,metaclusters_rviz = publish_locations(centroids,labels,location_array,clusters.clusters.clusters)

	### Respond using Service Response
	command.metaclusters = metaclusters
	command2.poses = metaclusters_rviz
	return MetaClusteringResponse(command,command2)


def meta_cluster_service():
	rospy.init_node('metacluster_location_pub', anonymous=True)
	print "Starting the meta-clustering algorithm"
	rospy.Service('/metaclustering', MetaClustering, handle_metaclustering)
	print "Successfully finished the meta-clustering algorithm"
	rospy.spin()
	
def call_clustering_service_client():
	rospy.wait_for_service('/clustering')
	try:
		clusters_service = rospy.ServiceProxy('/clustering',Clustering)
		clusters_service_response  = clusters_service()
		return clusters_service_response
	except rospy.ServiceException, e:
        	print "Service call failed: %s"%e

    
if __name__ == '__main__':
	meta_cluster_service()
		

