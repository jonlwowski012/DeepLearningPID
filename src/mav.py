#!/usr/bin/env python

# Jonathan Lwowski 
# Email: jonathan.lwowski@gmail.com 
# Autonomous Controls Lab 
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

### Callback function for UAV location Subscriber
def uavCallback(data):
	global uav_location
	for i in range(len(data.name)):
		if "uav" in data.name[i]:
			uav_location = data.pose[i]

### UAV Location Subscriber Function
def uav_location_sub():
	### Subscribe to /gazebo/model_states with a ModelStates Message and a callback of uavCallback
	rospy.Subscriber("/gazebo/model_states", ModelStates, uavCallback) 

### Function to takeoff UAV to a desired height  
def takeoff_height(height):
	global uav_location

	### Check if UAV Location has been recieved 
	while(uav_location == 0):
		a=1

	### While the UAV is lowered than the desired height 
	while(uav_location.position.z < height):
		### Call Publisher to move UAV
		set_velocity_uav(0,0,1.0,0,0,0)

	### Stop UAV at desired height
	set_velocity_uav(0.0,0,0,0,0,0)	

### UAV Publisher to publish a Twist Message to cmd_vel topic
def set_velocity_uav(lx1, ly1, lz1, ax1, ay1, az1):   
    pub1 = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    ### Setup twist message
    command1 = Twist()
    command1.linear.x = lx1
    command1.linear.y = ly1
    command1.linear.z = lz1
    command1.angular.x = ax1
    command1.angular.y = ay1
    command1.angular.z = az1

    ### Publish the twist message
    pub1.publish(command1)

### Function to track UGV using PID Controller
def track_ugv_2(time): 
    ### Get global variable containing location of UGV in UAV's camera
    global ugv_center
     
    ### Setup PID initial values 
    D_pitch = 1   
    Derivator_pitch = 0
    error_pitch = 0;
    Kd_pitch = 1
    Derivator_roll = 0
    error_roll = 0;
    Kd_roll = 1
    I_roll = 0
    I_pitch = 0
 
    ### Run PID tracking for desired time
    for q in range(0, time):
		print ugv_center
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

		### Call UAV Cmd_vel publisher
		set_velocity_uav(-PD_pitch, -PD_roll, 0, 0, 0, 0)

### Main function to start PID tracking
if __name__ == '__main__':
	### Start ROS Node
	rospy.init_node('mav', anonymous=True)

	### Publish at 50hz
	r = rospy.Rate(50) # 10hz

	### Subscribe to UAV location
	uav_location_sub()

	### Subscribe to UAV camera and detect UGV in camera
	uav_image_red()
	try:
		### Take off to desired height
		takeoff_height(6)

		while(1):
			### Track UGV using PID Controller
			track_ugv_2(1000000)



	except rospy.ROSInterruptException: pass

