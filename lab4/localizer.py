#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import matplotlib.pyplot as plt
import math
import numpy as np
import sys, select, os
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

e = """
Communications Failed
"""

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

class KalmanFilter(object):
    def __init__(self, h, d, x_0, Q, R, P_0):
        self.h = h
        self.d = d

        self.Q = Q
        self.R = R
        self.P = P_0
        self.x = x_0

	self.D = 0
	self.P = 10
	self.S = 0
	self.W = 0
	self.Z = 0

	
	self.time = 0
	self.plotx = []
	self.ploty = []
	self.plotang = []
	self.Pvar = []

	 



        self.u = 0 # initialize the cmd_vel input
        self.phi = np.nan #initialize the measurement input
        
        self.state_pub = rospy.Publisher('state', String, queue_size = 1)

    def cmd_callback(self, cmd_msg):
        self.u = cmd_msg.linear.x

    ## scall_callback updates self.phi with the most recent measurement of the tower.
    def scan_callback(self, data):
        self.phi = float(data.data)*math.pi/180

    ## call within run_kf to update the state with the measurement 
    def predict(self, u):

	self.x = self.x + u/30.0
	self.D = self.h / ((self.d - self.x)**2 + self.h**2)
	self.P = self.P + self.Q
	self.S = self.D**2 * self.P + self.R
	self.W = self.P * self.D / self.S
	self.P = self.P - self.W**2 * self.S


	
        return

    ## call within run_kf to update the state with the measurement 
    def measurement_update(self,current_measurement):

	self.Z = math.atan2(self.h , (self.d - self.x))   # convert to degree
	self.x = self.x + self.W * (current_measurement - self.Z) 
		
	
        return

    def run_kf(self):
        current_input = self.u
        current_measurement = self.phi
        print'current speed:',current_input   
	

	if not np.isnan(current_measurement) :

		self.predict(current_input)
		self.measurement_update(current_measurement)
	else :	
		self.predict(current_input)

        print'current position:',self.x    
	print 'angle', current_measurement   

	self.plotx.append(self.time)
	self.time = self.time + 1/30.0
	self.ploty.append(self.x)
	self.plotang.append(current_measurement)
	self.Pvar.append(self.P)
	
	
	plt.plot(self.plotx, self.ploty, '-r')
	plt.plot(self.plotx,self.plotang, 'b')
	plt.plot(self.plotx,self.Pvar, 'g')

	self.state_pub.publish(str(float(self.x)))


	


if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
        
    rospy.init_node('Lab4')
    try:
        h = 0.6 #y distance to tower
        d = 1.5 #x distance to tower (from origin)  
        
        x_0 = 0.0 #initial state position
        
        Q = 1 #process noise covariance
        R = 1 #measurement noise covariance
        P_0 = 1 #initial state covariance 
        kf = KalmanFilter(h, d, x_0, Q, R, P_0)
        kf.scan_sub = rospy.Subscriber('scan_angle', String, kf.scan_callback, queue_size=1)
        kf.cmd_sub = rospy.Subscriber('cmd_vel_noisy', Twist, kf.cmd_callback)
        rospy.sleep(1)
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            kf.run_kf()  
            rate.sleep()
	plt.show()
            
    #except:
        #print(e)

    finally:
        rospy.loginfo("goodbye")

