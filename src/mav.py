#!/usr/bin/env python

# Quadrotor Simulator
# Jonathan Lwowski 
# Email: jonathan.lwowski@gmail.com 
# Unmanned Systems Lab
# The University of Texas at San Antonio



############  Import Libraries   ##########################
import cv2
import cv
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy
from geometry_msgs.msg import Twist 
import math
import numpy as np
from gazebo_msgs.msg import ModelStates

################  Global Variables    ############################
uav_location = 0
ugv_center = [-999,-999]

###########      Camera Functions   ###############################
class uav_image_red:

  def __init__(self):
    self.image_pub = rospy.Publisher("image",Image, queue_size=10)

    cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("downward_cam/camera/image",Image,self.callback)

  def callback(self,data):
	try:
		cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError, e:
		print e
	### Start of Image Processing ######
	(rows,cols,channels) = cv_image.shape
	hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
	lower_red = np.array([0,100,100])
	upper_red = np.array([10,255,255])
	mask = cv2.inRange(hsv, lower_red, upper_red)
	### Start of Image Processing ######
		
	### Find the contours of all red ###
	contours, _ = cv2.findContours(mask.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)

	### Filter noise out so you only select actual pink circles
	centre = (-999,-999)
	global ugv_center
	for i in range(len(contours)):
		moments = cv2.moments(contours[i])
		#print cv2.contourArea(contours[i])
		max_area = 0
		
		if(cv2.contourArea(contours[i]) > max_area):
			centre = ((int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])))
			max_area = cv2.contourArea(contours[i])
	ugv_center[0] = centre[0] - cols/2
	ugv_center[1] = centre[1] - rows/2
	cv2.circle(cv_image,(int(centre[0]),int(centre[1])),3,(0,0,255),-1)


	#    #cv2.imshow('Thresholded', mask)

	#    ### End of Image Processing ######
	cv2.imshow('UAV Image', cv_image)
	cv2.waitKey(1)
	try:
		cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError, e:
		print e


def uavCallback(data):
	global uav_location
	for i in range(len(data.name)):
		if "uav" in data.name[i]:
			uav_location = data.pose[i]

def uav_location_sub():
	rospy.Subscriber("/gazebo/model_states", ModelStates, uavCallback) 

  
def takeoff_height(height):
	global uav_location
	while(uav_location == 0):
		a=1 
	while(uav_location.position.z < height):
		set_velocity_uav(0,0,1.0,0,0,0)
	set_velocity_uav(0.0,0,0,0,0,0)	

def set_velocity_uav(lx1, ly1, lz1, ax1, ay1, az1):   
    pub1 = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    
    command1 = Twist()
    command1.linear.x = lx1
    command1.linear.y = ly1
    command1.linear.z = lz1
    command1.angular.x = ax1
    command1.angular.y = ay1
    command1.angular.z = az1
    hover = Twist()
    hover.linear.x = 0.0
    hover.linear.y = 0.0
    hover.linear.z = 0.0
    hover.angular.x = 0.0
    hover.angular.y = 0.0
    hover.angular.z = 0.0
    pub1.publish(command1)

def track_ugv_2(time): 
    global ugv_center
    
    D_pitch = 1   
    Derivator_pitch = 0
    error_pitch = 0;
    Kd_pitch = 1
    Derivator_roll = 0
    error_roll = 0;
    Kd_roll = 1
    I_roll = 0
    I_pitch = 0
    for q in range(0, time):
		print ugv_center
		constant = uav_location.position.z
		############## Pitch PID   #####################
		P_pitch=ugv_center[1]
		I_pitch += ugv_center[1]
		error_pitch = ugv_center[1]
		D_pitch = error_pitch - Derivator_pitch
		PD_pitch = 0.006*P_pitch + 0.4*D_pitch + 0.0000001*I_pitch
		Derivator_pitch = error_pitch
		Kd_pitch = D_pitch
		############# Roll PID      ##################### 
		P_roll= ugv_center[0]
		I_roll += ugv_center[0]
		error_roll = ugv_center[0]
		D_roll = error_roll - Derivator_roll
		PD_roll = 0.006*P_roll + 0.4*D_roll + 0.0000001*I_roll
		Derivator_roll = error_roll
		Kd_roll = D_roll
		set_velocity_uav(-PD_pitch, -PD_roll, 0, 0, 0, 0)

if __name__ == '__main__':
	rospy.init_node('mav', anonymous=True)
	r = rospy.Rate(50) # 10hz
	uav_location_sub()
	uav_image_red()
	try:
		takeoff_height(6)
		while(1):
			track_ugv_2(1000000)



	except rospy.ROSInterruptException: pass

