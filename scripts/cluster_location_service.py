#!/usr/bin/env python

# Title: Cluster Location Publisher
# Description: Clusters all the peoples locations into desired radius and publishes the clusters
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


def clustering(data):
	location_array = []
	if(data.people_locations.poses != []):
				#print (len(data.poses))

				### Store peoples x&y locations in array
				#location_array = [(data.poses[0].position.x,data.poses[0].position.y)]
				for i in range(len(data.people_locations.poses)):
					location_array.append((data.people_locations.poses[i].position.x,data.people_locations.poses[i].position.y))

				### Perform K-means Clustering
				k = 1
				#print "location_array: ", location_array
				if(location_array != []):
					kmeans = cluster.KMeans(n_clusters=k)
					kmeans.fit(location_array)
					centroids = kmeans.cluster_centers_	
					inertia = kmeans.inertia_

					### while the average cluster radius is less than 10m
					while((inertia/len(data.people_locations.poses))>= 10):
						k = k+1
						#print (k)
						kmeans = cluster.KMeans(n_clusters=k)
						#print "location_array: ", location_array
						kmeans.fit(location_array)
						centroids_temp = kmeans.cluster_centers_	
						inertia = kmeans.inertia_

					### Calculate Radius for Cluster Msg
					radius = inertia/len(data.people_locations.poses)

					### Calculate Centroids of Clusters
					centroids = centroids_temp
					#print ("centroids len", len(centroids))

					### Get Labels for all people's locations
					labels = kmeans.labels_ 

	return centroids,labels,radius
	


def publish_locations(centroids,labels,radius):

	while(len(centroids) <= 1):
		a=1


	### Init some variables
	people_count = [0]*len(centroids)
	posearr = []
	posearr2 = []
	command2 = PoseArray()
	command2.header.stamp = rospy.get_rostime()
	command2.header.frame_id = 'world'
	command = ClusterCenters()

	if len(centroids) > 1 :
		centroids_temp = centroids

		### for all the cluster centers

		for i in range(len(centroids_temp)):
			try:
				### Setup Cluster Msg
                                cluster_center = ClusterCenter()

				### Store Cluster Label into Msg
				cluster_center.cluster_label = str(i)

				### Count number of people in Cluster
				for j in range(len(labels)):
					if(labels[j] == i):
						cluster_center.number_people = cluster_center.number_people + 1

				### Setup Pose Msg
				pose = Pose()
				quat = Quaternion()
				point = Point()
				point.x = centroids_temp[i][0]
				point.y = centroids_temp[i][1]
				point.z = 0
				quat.x = 0
				quat.y = 0
				quat.z = 0
				quat.w = 0
				pose.position=point
				pose.orientation=quat
				cluster_center.pose=pose
				
				### Append Clusters into both publishers
				posearr.append(cluster_center)
				posearr2.append(pose)

			except IndexError:
				print ("error", len(centroids))#'invalid index' 

		return posearr,posearr2

def handle_clustering(req):
	time.sleep(2)
	### Call people's locations service
	people_locations = call_peoplelocations_service_client()

	### Perform Clustering
	centroids,labels,radius = clustering(people_locations)

	
	### Publish clusters
	command = ClusterCenters()
	command2 = PoseArray()
	command2.header.stamp = rospy.get_rostime()
	command2.header.frame_id = 'world'
	clusters, clusters_rviz = publish_locations(centroids,labels,radius)
	command.clusters = clusters
	command2.poses = clusters_rviz

	return ClusteringResponse(command,command2)

def clustering_service():
 	rospy.init_node('cluster_location_pub', anonymous=True)
	print "Starting the clustering algorithm"
	rospy.Service('/clustering', Clustering, handle_clustering)
	print "Successfully finished the clustering algorithm"
	rospy.spin()

def call_peoplelocations_service_client():
	rospy.wait_for_service('/people_location_combiner')
	try:
		people_locations_service = rospy.ServiceProxy('/people_location_combiner',PeopleLocations)
		people_locations_service_response  = people_locations_service()
		return people_locations_service_response
	except rospy.ServiceException, e:
        	print "Service call failed: %s"%e

if __name__ == '__main__':
	clustering_service()
	
