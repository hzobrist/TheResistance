#!/usr/bin/env python

# Revision $Id$

import rospy
from std_msgs.msg import String
from subscriber.msg import FieldPositions
from subscriber.srv import *
from Point import Point
from velchange import *
import math
from numpy import matrix
import signal
import sys
# from roboclaw import *

start_time = 1

centerField = Point()
goal = Point()
#hardcoded if vision is untrustworthy
centerField.x = 423
centerField.y = 234
goal.x = 0
goal.y = 234
defensex = 50
globalCounter = 0
dbehind = 20
yNorthThreshold = -30
ySouthThreshold = 15
yNorthOffset = -16
ySouthOffset = 18
goalYOffset = -12
fastMaxSpeed = .9
slowMaxSpeed = .3
maxSpeed = fastMaxSpeed

FIELD_CENTERX = 304
FIELD_CENTERY = 227
FIELD_HEIGHT = 448
FIELD_WIDTH = 664
FIELD_START = 154
FIELD_END = 818
FIELD_TOP = 7
FIELD_BOTTOM = 455
RESET_ROBOT = True
myBall = Point()
OFFSET_CONST = .06

away = 0

def callback(data):
	global start_time
	global RESET_ROBOT

	data.robot.x = data.robot.x - FIELD_START
	data.robot.y = data.robot.y - FIELD_TOP
	data.ball.x = data.ball.x - FIELD_START
	data.ball.y = data.ball.y - FIELD_TOP
	data.opponent1.x = data.opponent1.x - FIELD_START
	data.opponent1.y = data.opponent1.y - FIELD_TOP

	# If robot needs to be moved to the starting position
	if(False):
		if(away == 1):
			if(abs(data.robot.x - FIELD_WIDTH*2/3) < 5 and
				abs(data.robot.y - FIELD_CENTERY) < 5):
				RESET_ROBOT = False
				stopAllMotors()
				inpt = ""
				while(inpt != "play"):
					print(str(inpt))
					inpt = str(raw_input("Type play to start playing. "))
				
			print("going to home: ")
			print(FIELD_WIDTH*2/3,FIELD_CENTERY)
			goToPoint(FIELD_WIDTH*2/3, FIELD_CENTERY, 180, data)
		else:
			if(abs(data.robot.x - FIELD_WIDTH/3) < 5 and
				abs(data.robot.y - FIELD_CENTERY) < 5):
				RESET_ROBOT = False
				stopAllMotors()
				inpt = ""
				while(inpt != "play"):
					print(str(inpt))
					inpt = str(raw_input("Type play to start playing. "))
				
			print("going to home: ")
			print(FIELD_WIDTH/3,FIELD_CENTERY)
			goToPoint(FIELD_WIDTH/3, FIELD_CENTERY, 0, data)
	else:		
		if(data.ball.x >0):	#if vision can see the ball, update position
			myBall.x = data.ball.x
			myBall.y = data.ball.y
		if(away == 1):
			ballXOffset = 23
			goal.x = 0
		else:
			ballXOffset = -23
			goal.x = 661
		# Offensive Code
		rospy.loginfo("Robot x: %f y: %f theta: %f", data.robot.x, data.robot.y, data.robot.theta)
		rospy.loginfo("Ally1 x: %f y: %f theta: %f", data.ally1.x, data.ally1.y, data.ally1.theta)
		rospy.loginfo("opponent 1 x: %f y: %f theta: %f", data.opponent1.x, data.opponent1.y, data.opponent1.theta)
		rospy.loginfo("ball x: %f y: %d", myBall.x, myBall.y)

		# Collision Avoidance - slows robot down if it comes close to enemy
		# careful = 0
		# distanceFromOpponent = math.sqrt(math.pow(data.opponent1.x - data.robot.x, 2) + math.pow(data.opponent1.y - data.robot.y, 2))
		# if(distanceFromOpponent < 70):
		# 	maxSpeed = slowMaxSpeed
		# else:
		# 	maxSpeed = fastMaxSpeed
		maxSpeed = fastMaxSpeed

		#trig used to go behind ball
		ballTheta = math.atan2(goal.y-myBall.y, myBall.x)
		ybehind = dbehind*math.sin(ballTheta)
		xbehind = dbehind*math.cos(ballTheta)
		pointBehindY = myBall.y - ybehind
		if(away == 1):
			pointBehindX = myBall.x + xbehind
		else:
			pointBehindX = myBall.x - xbehind

		# Offset calculation. As distance from center line increases, offset increases
		pointBehindY = (pointBehindY - FIELD_CENTERY)*OFFSET_CONST + pointBehindY

		if(math.fabs(pointBehindX - data.robot.x) < 10 and math.fabs(pointBehindY - data.robot.y) < 10):
			
			goToPoint(goal.x, goal.y + goalYOffset, 0, data)
		else:
			goToPoint(pointBehindX, pointBehindY, 0, data)
			

		#Defensive Code
		# rospy.loginfo("Robot x: %f y: %f theta: %f", data.robot.x, data.robot.y, data.robot.theta)
		# rospy.loginfo("Ball x: %f y: %f", data.ball.x, data.ball.y)
		# ## TYGAN:: DO WHAT YOU NEED TO WITH THE POSITIONS
	   
		#Defense based on goToPoint
		# dx = (centerField.x-data.robot.x)/80
		# dy = (centerField.y-data.robot.y)/50
		# diffx = 90-data.robot.x
		# diffy = centerField.y-data.robot.y
		# if(data.ball.y > 175 and data.ball.y < 325):
		#     goToPoint(100, data.ball.y, 0, data)
	   
		# goToPoint(round(FIELD_CENTERX),round(FIELD_CENTERY), 0, data)

		#diffDef = defensex = data.robot.x
		# print("diffx:")
		# print(diffx)
		# #print("diffy:")
		# #print(diffy)

		# print("theta:")
		# print(data.robot.theta)

		# if(data.robot.theta > 20 and data.robot.theta < 180):
		#     moveAllMotors(15,15,15)
		# elif(data.robot.theta > 180 and data.robot.theta < 340):
		#     moveAllMotors(-15,-15,-15)
		# else:
		#     if(diffx < -20):
		#         goXYOmega(-.3,0,0)
		#     elif(diffx > 10):
		#         goXYOmega(.3,0,0)
		#     else:
		#         diffBally = data.ball.y - data.robot.y
		#         if(data.ball.y > 325):
		#             goXYOmega(0,0,0);
		#         elif(data.ball.y < 175):
		#             goXYOmega(0,0,0);
		#         else:
		#             if(diffBally > 10):
		#                 goXYOmega(0,.3,0);
		#             elif(diffBally < -10):
		#                 goXYOmega(0,-.3,0);
		#             else:
		#                 goXYOmega(0,0,0);

	# stopAllMotors()
	start_time = time.time()

def goToPoint(x,y,theta,data):
	global globalCounter
	SCALE_VEL = 0.007
	SCALE_OMEGA = 1.0
	if(globalCounter >= 0):
		desired_x = x
		desired_y = y
	
		dx = (desired_x-data.robot.x) * SCALE_VEL
		dy = (desired_y-data.robot.y) * SCALE_VEL
	
		if(dx > maxSpeed):
			dx = maxSpeed
		elif(dx < -1*maxSpeed):
			dx = -1*maxSpeed
		if(dy > maxSpeed):
			dy = maxSpeed
		elif(dy < -1*maxSpeed):
			dy = -1*maxSpeed
		mag = math.sqrt(dx**2+dy**2) 
		angle = math.atan2(desired_y-data.robot.y, desired_x-data.robot.x)
	
		delta_angle = angle-(data.robot.theta*math.pi/180)
	
		bestDelta = math.atan2(math.sin(delta_angle), math.cos(delta_angle)) * SCALE_OMEGA

		goXYOmegaTheta(dx,dy,0,data.robot.theta*math.pi/180)
		globalCounter = 0
	else:
		print "Skipped"
		globalCounter = globalCounter + 1

def goToTheta(theta, data):
	###STILL WORKING ON THIS###
	#Create a theta on the opposite side of where we want to go
	print("Going to Theta")
	thetaRef = 0
	threshhold = 20
	thetaUpperBound = theta + threshhold
	thetaLowerBound = theta - threshhold
	x = 0
	if(theta >=180):
		thetaRef = theta-180
	else:
		thetaRef = theta + 180
	if(thetaUpperBound >= 360):
		thetaUpperBound = thetaUpperBound - 360
	if(thetaLowerBound < 0):
		thetaLowerBound = 360 + thetaLowerBound
	#print("upper, lower, ref: ")
	#print(thetaUpperBound, thetaLowerBound, thetaRef)
	#Now check to see if it is in the right range
	if(data.robot.theta >= thetaLowerBound and data.robot.theta <= thetaUpperBound):
		moveAllMotors(0,0,0)
	else:
		if(data.robot.theta > thetaRef):
			#Turn clockwise
			moveAllMotors(-16,-16,-16)
		else:
			#turn counterclockwise
			moveAllMotors(16,16,16)

def get_field_info(refresh):
	
	global FIELD_CENTERX
	global FIELD_CENTERY
	global FIELD_HEIGHT
	global FIELD_WIDTH
	global FIELD_START
	global FIELD_END
	global FIELD_TOP
	global FIELD_BOTTOM
	rospy.wait_for_service('getFieldInfo')

	try:
		field_info = rospy.ServiceProxy('getFieldInfo', fieldInfo)
		resp1 = field_info(refresh)
	   
		FIELD_CENTERX = resp1.centerX
		FIELD_CENTERY = resp1.centerY
		FIELD_HEIGHT = resp1.height
		FIELD_WIDTH = resp1.width
		FIELD_START = resp1.start
		FIELD_END = resp1.end
		FIELD_TOP = resp1.top
		FIELD_BOTTOM = resp1.bottom

		print("field info: ("+str(FIELD_WIDTH)+"x"+str(FIELD_HEIGHT)+")")
		print("start: "+str(FIELD_START))
		print("end: "+str(FIELD_END))
		print("center: ("+str(FIELD_CENTERX)+", "+str(FIELD_CENTERY)+")")
		print("top: ("+str(FIELD_TOP)+" bottom: "+str(FIELD_BOTTOM)+")")
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def signal_handler(signal, frame):
	print('You pressed Ctrl+C!')
	
	stopAllMotors()
	RESET_ROBOT = True
	sys.exit(0)

def listener():
	global start_time
	
	calibrateRoboclaws()
	signal.signal(signal.SIGINT, signal_handler)

	print("batteryLevel: "+str(readmainbattery()/10.0)+"V")

	inpt = ""
	while(inpt != "start"):
		print(str(inpt))
		inpt = str(raw_input("Type start to move to starting position. "))

	print("3....")
	print("2....")
	print("1....")
	print("GOOOO!!!!")

	moveAllMotors(20,20,20)
	# uncomment this line to get field info from camera 
	# for now these values are hardcoded
	# get_field_info(False)
	
	# In ROS, nodes are uniquely named. If two nodes with the same
	# name are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.

	rospy.init_node('listener', anonymous=True)

	rospy.Subscriber('chatter', FieldPositions, callback)

	start_time = time.time()
	# spin() simply keeps python from exiting until this node is stopped
	
	rospy.spin()

if __name__ == '__main__':
	listener()
