#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String



class waffle:

	def callbackCam(self, color_mono):
		self.position = int(color_mono.data)
		pass

	def callbackState(self, state):
		self.state = float(state.data)
		pass

	def __init__(self):
		#self.angular = .2
		#self.kp = .008
		#self.ki = .00012
		#self.kd = 0.0005

		self.kp = .002
		self.ki = .0003
		self.kd = 0.008


		#self.ki=0
		#self.kd=0

		#self.kp = .002
		#self.ki = .0002
		#self.kd = 0.006

		self.linear = 0.1
		self.position = 0
		self.desired = 320
		self.twist = Twist()
		self.state = 0

		self.integral = 0
		self.integral_cap = 1000
		self.lasterror = 0
		self.derivative = 0


		self.cmd_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
		self.state_sub = rospy.Subscriber('state',String, self.callbackState, queue_size = 1)
		self.cam_subscriber=rospy.Subscriber('color_mono',String,self.callbackCam,queue_size=1)
		
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


	def stopat(self,stop):

		while self.state < stop:

			self.rate.sleep()


			error = self.desired - self.position
			self.integral = self.integral + error
			self.derivative = error - self.lasterror
			if self.integral > self.integral_cap:
				self.integral = self.integral_cap
			elif self.integral < -self.integral_cap:
				self.integral = -self.integral_cap
			self.correction = error * self.kp + self.integral * self.ki + self.derivative * self.kd

			self.lasterror = error
			self.twistandturn(self.correction)
			self.time = self.time + 1
			print'state:', self.state
		self.stop()
		rospy.sleep(3)



	def publisher_node(self):
	    	#print('TODO: initialize the publisher node here, \
		#    and publish wheel command to the cmd_vel topic')

		#Seteup of robot
		self.twist.linear.x = self.linear
		self.cmd_pub.publish(self.twist)


		self.stopat(0.60)
		self.stopat(1.22)
		self.stopat(2.44)
		self.stopat(3.05)



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
