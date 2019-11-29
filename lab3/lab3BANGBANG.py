#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String



class waffle:

	def callback(self, color_mono):
		print('callback')
		self.position = int(color_mono.data)
		pass

	def __init__(self):
		self.position = 0
		self.desired = 320
		self.twist = Twist()
		self.cmd_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
		self.cam_subscriber=rospy.Subscriber('color_mono',String,self.callback,queue_size=1)
		self.freq = 10
		self.rate = rospy.Rate(self.freq) #10Hz
		rospy.sleep(1)
			

	def twistandturn(self,angular):
		self.twist.linear.x = 0.1
		self.twist.angular.z = angular
		self.cmd_pub.publish(self.twist)




	def publisher_node(self):
	    	#print('TODO: initialize the publisher node here, \
		#    and publish wheel command to the cmd_vel topic')

		#Seteup of robot
		linear = 0.1
		self.twist.linear.x = linear
		self.cmd_pub.publish(self.twist)

	
		angular = .2


		#BANG BANG
		while 1:

			self.rate.sleep()


			error = self.desired - self.position

			print("this is error")
			print(error)

			print("position is")
			print(self.position)



			if error<0:
				self.twistandturn(-angular)
			elif error>0:
				self.twistandturn(angular)
			else:
				self.twistandturn(0)


	    	pass

	


def main():
    	try:
		rospy.init_node('camera')
		Frank = waffle()
		Frank.publisher_node()
   	except rospy.ROSInterruptException:
        	pass
    

if __name__ == '__main__':
    	main()
