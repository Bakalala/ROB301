#!/usr/bin/env python

import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import re
import sys
import select
import os
import BayClass as Bay

if os.name == 'nt':
    import msvcrt
else:
    import tty
    import termios




def getKey():
    if os.name == 'nt':
        return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class BayesLoc:

    def __init__(self, ):

        self.limit = 20

        # updated with the measurement_callback
        self.measured_rgb = np.array([0, 0, 0])
        self.colour = 4  # 'None'

        self.Bay = Bay.BayClass()
        self.loc = -1

        self.kp = .0025
        self.ki = 0
        self.kd = 0

        self.linear = 0.05
        self.position = 320
        self.desired = 320
        self.twist = Twist()

        self.integral = 0
        self.integral_cap = 1000
        self.lasterror = 0
        self.derivative = 0

        self.freq = 10
        self.rate = rospy.Rate(self.freq)  # 10Hz
        rospy.sleep(1)
        self.time = 0

        self.colour_sub = rospy.Subscriber(
            'mean_img_rgb', String, self.measurement_callback)
        self.line_idx_sub = rospy.Subscriber(
            'line_idx', String, self.line_callback)
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def measurement_callback(self, msg):
        rgb = msg.data.replace('r:', '').replace(
            'b:', '').replace('g:', '').replace(' ', '')
        r, g, b = rgb.split(',')
        r, g, b = (float(r), float(g), float(b))
        self.measured_rgb = np.array([r, g, b])
        self.whatcolour(
            self.measured_rgb[0], self.measured_rgb[1], self.measured_rgb[2])
        #print (self.colour)

    def line_callback(self, data):
        index = int(data.data)
        self.position = index

    def whatcolour(self, r, g, b):

        # blue [150 140 140]
        if abs(r - 35) < self.limit and abs(g - 92) < self.limit and abs(b - 170) < self.limit:
            self.colour = 1  # 'Blue'

        # yellow [177 178 60]
        elif abs(r - 220) < self.limit and abs(g - 185) < self.limit and abs(b - 10) < self.limit:
            self.colour = 2  # 'Yellow'

        # orange [215 118 94]
        elif abs(r - 235) < self.limit and abs(g - 70) < self.limit and abs(b - 10) < self.limit:
            self.colour = 3  # 'Orange'

        # green [150 170 75]
        elif abs(r - 185) < self.limit and abs(g - 215) < self.limit and abs(b - 10) < self.limit:
            self.colour = 1  # 'Green'

        else:
            self.colour = 4  # "None"

    def twistandturn(self, correction):
        self.twist.linear.x = self.linear
        self.twist.angular.z = correction
        self.cmd_pub.publish(self.twist)

    def stop(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_pub.publish(self.twist)

    def turn(self):
        w = .2
        time = math.pi / (2*w)
        self.twist.linear.x = 0
        self.twist.angular.z = w
        self.cmd_pub.publish(self.twist)
        rospy.sleep(time)
        self.stop()
        rospy.sleep(1)
        self.twist.angular.z = -w
        self.cmd_pub.publish(self.twist)
        rospy.sleep(time)
        self.stop()


    def go(self):
        lasterror = 0
        self.integral = 0

        # 'None':  # or self.position < 30:
        while not rospy.is_shutdown() and self.colour == 4:
            self.rate.sleep()
            error = self.desired - self.position
            self.integral = self.integral + error
            self.derivative = error - lasterror
            if self.integral > self.integral_cap:
                self.integral = self.integral_cap
            elif self.integral < -self.integral_cap:
                self.integral = -self.integral_cap
            self.correction = error * self.kp + self.integral * \
                self.ki + self.derivative * self.kd

            lasterror = error

            self.twistandturn(self.correction)


        #rospy.loginfo("Measurement: {}".format(BL.measured_rgb))
        self.loc = self.Bay.update(1, self.colour)
        rospy.loginfo("Color : {}".format(BL.colour))
        self.twistandturn(0)
        # while self.position < 40:
        #	print(self.position)
        #	pass

        #rospy.loginfo("Measurement: {}".format(BL.measured_rgb))
        #rospy.loginfo("Color : {}".format(BL.colour))

        rospy.sleep(3)
        self.colour = 4
        self.stop()


if __name__ == "__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('bayes_loc')
    BL = BayesLoc()
    rospy.sleep(0.5)

    # Initialize your PID controller here ( to merge with the bayes_loc node )
    #PID = PIDcontrol()

    try:


        # while not rospy.is_shutdown():
        #     key = getKey()
        #     if (key == '\x03'):
        #         rospy.loginfo('Finished!')
        #         break
        #
        #     rospy.loginfo("Measurement: {}".format(BL.measured_rgb))
        #     rospy.loginfo("Color : {}".format(BL.colour))


        target = 5
        while not rospy.is_shutdown() and BL.loc != target:
            key = getKey()
            if (key == '\x03'):  # 1.22:bayesian.curPos >= 1.6 or
                rospy.loginfo('Finished!')
                break

            BL.go()
            if target == BL.loc:
                BL.turn()
            BL.twistandturn(0)
            rospy.sleep(3)
            BL.stop()

            #rospy.loginfo("Measurement: {}".format(BL.measured_rgb))
            #rospy.loginfo("Line index: {}".format(BL.position))
            #rospy.loginfo("Color : {}".format(BL.colour))
            rospy.loginfo("Location estimate: {}".format(BL.loc))

        rospy.loginfo("Reached target: {}".format(target))


        target = 7
        while not rospy.is_shutdown() and BL.loc != target:
            key = getKey()
            if (key == '\x03'):
                rospy.loginfo('Finished!')
                break

            BL.go()
            if target == BL.loc:
                BL.turn()
            BL.twistandturn(0)
            rospy.sleep(3)
            BL.stop()

            rospy.loginfo("Location estimate: {}".format(BL.loc))
        rospy.loginfo("Reached target: {}".format(target))

        target = 9
        while not rospy.is_shutdown() and BL.loc != target:
            key = getKey()
            if (key == '\x03'):
                rospy.loginfo('Finished!')
                break

            BL.go()
            if target == BL.loc:
                BL.turn()
            BL.twistandturn(0)
            rospy.sleep(3)
            BL.stop()

            rospy.loginfo("Location estimate: {}".format(BL.loc))
        rospy.loginfo("Reached target: {}".format(target))

#    except Exception as e:
#        print("comm failed:{}".format(e))

    finally:

            # Stop the robot when code ends
        cmd_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        cmd_publisher.publish(twist)
