#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String



class waffle:

    	def line_callback(self, data):
        	index = int(data.data)
        	self.position = index

	def move_callback(self,data):
		self.shouldmove = data.data
		print self.shouldmove


	def __init__(self):


		self.kp = .003
		self.ki = 0.00001
		self.kd = 0.00001

		self.linear = 0.05
		self.position = 320
		self.desired = 320
		self.twist = Twist()

		self.integral = 0
		self.integral_cap = 1000
		self.lasterror = 0
		self.derivative = 0

		self.shouldmove


		self.movement_sub = rospy.Subscriber('movement', String, self.move_callback)
		self.cmd_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
        	self.line_idx_sub = rospy.Subscriber('line_idx', String, self.line_callback)
		self.freq = 10
		self.rate = rospy.Rate(self.freq) #10Hz
		rospy.sleep(1)
		self.time = 0


	def twistandturn(self,correction):
		self.twist.linear.x = self.linear
		self.twist.angular.z = correction
		self.cmd_pub.publish(self.twist)

	def stop(self):
		self.twist.linear.x = 0
		self.twist.angular.z = 0
		self.cmd_pub.publish(self.twist)


	def go(self):


        	while self.position > 10:

			self.rate.sleep()


			error = self.desired - self.position
			#self.integral = self.integral + error
			#self.derivative = error - self.lasterror
			#if self.integral > self.integral_cap:
			#	self.integral = self.integral_cap
			#elif self.integral < -self.integral_cap:
			#	self.integral = -self.integral_cap
			self.correction = error * self.kp #+ self.integral * self.ki + self.derivative * self.kd

			#self.lasterror = error
			self.twistandturn(self.correction)

			print(self.position)

        	self.twistandturn(0)
        	rospy.sleep(1)
        	self.stop()
        	print(self.position)



	def publisher_node(self):
	    	#print('TODO: initialize the publisher node here, \
		#    and publish wheel command to the cmd_vel topic')

		#Seteup of robot

		while()
			self.twist.linear.x = self.linear
			self.cmd_pub.publish(self.twist)


			self.go()



			self.stop()

	    	pass




def main():
    	try:
            rospy.init_node('pid')
            Frank = waffle()
            while (1):
                Frank.publisher_node()


   	except rospy.ROSInterruptException:
        	pass


if __name__ == '__main__':
    	main()
