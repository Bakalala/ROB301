#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String



class waffle:

	def callback(self, color_mono):
		self.position = int(color_mono.data)
		pass

	def __init__(self):
		# good for intense course
		#self.kp = .008
		#self.ki = .00035
		#self.kd = 0.003

		self.kp = .006
		self.ki = .00035
		self.kd = 0.003
		self.linearNormal = 0.25
		self.linear = 0.25
		self.position = 0
		self.desired = 320
		self.twist = Twist()
		self.cmd_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
		self.cam_subscriber=rospy.Subscriber('color_mono',String,self.callback,queue_size=1)
		self.freq = 14
		self.rate = rospy.Rate(self.freq) #10Hz
		rospy.sleep(1)
		self.time = 0
			

	def twistandturn(self,correction):
		self.twist.linear.x = self.linear
		self.twist.angular.z = correction
		#print(self.twist.angular.z)
		self.cmd_pub.publish(self.twist)

	def stop(self):
		self.twist.linear.x = 0
		self.twist.angular.z = 0
		self.cmd_pub.publish(self.twist)

	def findspeed(self, correction):
		if abs(correction)>2:
			return 0.1
		elif abs(correction)>1.75:
			return .125
		elif abs(correction)>1.5:
			return .15
		elif abs(correction)>1.25:
			return 0.175
		else
			return 0.2
		
		#else:
		#	return abs(correction)*0.15/2 + 0.1

		#We can use the linear function but it was nice when it was discrete




	def publisher_node(self):
	    	#print('TODO: initialize the publisher node here, \
		#    and publish wheel command to the cmd_vel topic')

		#Seteup of robot
		self.twist.linear.x = self.linear
		self.cmd_pub.publish(self.twist)

	



		#PI

		integral = 0
		integral_cap = 500
		lasterror = 0
		derivative = 0
		while self.time < 230:

			self.rate.sleep()


			error = self.desired - self.position
			integral = integral + error
			derivative = error - lasterror
			if integral > integral_cap:
				integral = integral_cap
			elif integral <-integral_cap:
				integral = -integral_cap
			correction = error * self.kp + integral * self.ki + derivative * self.kd

			#print("this is error")
			#print(error)

			lasterror = error

			self.linear = self.findspeed(correction)

			#correction = correction*(self.linear/0.2)		
			
			print'linear:',self.linear
			
	
			print'correction:',correction

			self.twistandturn(correction)
			self.time = self.time + 1


		self.stop()

	    	pass

	


def main():
    	try:
		rospy.init_node('pid')
		Frank = waffle()
		Frank.publisher_node()
   	except rospy.ROSInterruptException:
        	pass
    

if __name__ == '__main__':
    	main()
