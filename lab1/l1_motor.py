#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String

def publisher_node():
    	print('TODO: initialize the publisher node here, \
            and publish wheel command to the cmd_vel topic')
	cmd_pub=rospy.Publisher('cmd_vel',Twist,queue_size=1)
	rate = rospy.Rate(10) #10Hz
	time = 0
	twist = Twist()
	linear = .3
	angular = .3
	twist.linear.x = linear
	twist.angular.z = 0
	rospy.sleep(1)
	cmd_pub.publish(twist)
	while time < (10 / linear): 
		rospy.loginfo(twist)
		rate.sleep()
		time = time + 1
        twist.linear.x = 0
        twist.angular.z = angular
        cmd_pub.publish(twist)
	time = 0
	while time < (10*2*3.14/angular):
                rospy.loginfo(twist)
                rate.sleep()
                time = time + 1

       	twist.linear.x = 0
       	twist.angular.z = 0
       	cmd_pub.publish(twist)

    	pass


def main():
    	try:
        	rospy.init_node('motor')
        	publisher_node()
   	except rospy.ROSInterruptException:
        	pass
    

if __name__ == '__main__':
    	main()
