#!/usr/bin/env python

# Jonathan Lwowski 
# Email: jonathan.lwowski@gmail.com
# Autonomous Controls Engineering Lab
# The University of Texas at San Antonio


#########          Libraries         ###################
import sys
import rospy
from geometry_msgs.msg import Twist 


### UGV Publisher Function ##### 

def set_velocity_ugv(lx1, ly1, lz1, ax1, ay1, az1):
	### Setup Publisher to publish to /p3dx/cmd_vel with a Twist Message   
	pub1 = rospy.Publisher('/p3dx/cmd_vel', Twist, queue_size=10)

	### Publish messages at 50hz
	r = rospy.Rate(50)

	### Setup message to be published
	command1 = Twist()
	command1.linear.x = lx1
	command1.linear.y = ly1
	command1.linear.z = lz1
	command1.angular.x = ax1
	command1.angular.y = ay1
	command1.angular.z = az1
    
   	### Publish Message
	pub1.publish(command1)
     

### Main Function to make UGV drive in a circle.
if __name__ == '__main__':
	rospy.init_node('ugv', anonymous=True)
	rospy.sleep(5.)
	try:
		while(1):
			lx1 = 1.0
			az1 = 0.5
			set_velocity_ugv(lx1, 0, 0, 0, 0, az1)
			
	except rospy.ROSInterruptException: pass
