#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String

twist = Twist()
global position
position = 0
desired = 320



def stop(cmd_pub):
	twist.linear.x = 0
       	twist.angular.z = 0
       	cmd_pub.publish(twist)


def move(dist, linear,cmd_pub):
	twist.linear.x = linear
	twist.angular.z = 0
	cmd_pub.publish(twist)
	rospy.sleep(dist/linear)
	stop(cmd_pub)



def turn(angle,angular,cmd_pub):
        twist.linear.x = 0
        twist.angular.z = angular
        cmd_pub.publish(twist)
	rospy.sleep(angle/angular)
	stop(cmd_pub)

def twistandturn(linear,angular,cmd_pub):
	twist.linear.x = linear
        twist.angular.z = angular
        cmd_pub.publish(twist)




def publisher_node(cmd_pub):
    	#print('TODO: initialize the publisher node here, \
        #    and publish wheel command to the cmd_vel topic')

	#Seteup of robot

	freq = 100
	rate = rospy.Rate(freq) #10Hz
	linear = 0
	angular = .2



	#BANG BANG
	while 1:
		error = desired - position

		print("this is error")
		print(error)

		print("position is")
		print(position)



		if error<0:
			twistandturn(linear,(-angular),cmd_pub)
		elif error>0:
			twistandturn(linear,angular,cmd_pub)
		else:
			twistandturn(linear,0,cmd_pub)


    	pass

def callback(color_mono):
	print('callback')
	position = int(color_mono.data)
	pass


def main():
    	try:
		rospy.init_node('camera')
		cmd_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
		cam_subscriber=rospy.Subscriber('color_mono',String,callback,queue_size=1)
		publisher_node(cmd_pub)
   	except rospy.ROSInterruptException:
        	pass
    

if __name__ == '__main__':
    	main()
