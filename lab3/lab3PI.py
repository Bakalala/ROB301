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
		#self.angular = .2
		self.kp = .01
		self.ki = .00015
		self.linear = 0.1
		self.position = 0
		self.desired = 320
		self.twist = Twist()
		self.cmd_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
		self.cam_subscriber=rospy.Subscriber('color_mono',String,self.callback,queue_size=1)
		self.freq = 10
		self.rate = rospy.Rate(self.freq) #10Hz
		rospy.sleep(1)
		self.time = 0
			

	def twistandturn(self,correction):
		self.twist.linear.x = self.linear
		self.twist.angular.z = correction
		print(self.twist.angular.z)
		self.cmd_pub.publish(self.twist)

	def stop(self):
		self.twist.linear.x = 0
		self.twist.angular.z = 0
		self.cmd_pub.publish(self.twist)





	def publisher_node(self):
	    	#print('TODO: initialize the publisher node here, \
		#    and publish wheel command to the cmd_vel topic')

		#Seteup of robot
		self.twist.linear.x = self.linear
		self.cmd_pub.publish(self.twist)

	



		#PI

		integral = 0
		integral_cap = 1500
		while self.time < 350:

			self.rate.sleep()


			error = self.desired - self.position
			integral = integral + error
			if integral > integral_cap:
				integral = integral_cap
			elif integral <-integral_cap:
				integral = -integral_cap
			correction = error * self.kp + integral * self.ki

			#print("this is error")
			#print(error)


			self.twistandturn(correction)
			self.time = self.time + 1
			print(self.time)
			print(integral)


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
