#!/usr/bin/python
from numpy import matrix
from numpy import linalg
import math

#define s1,s2,s3

realWorldOffset = 1 #1.698
s = .0282977488817 #radius of wheel
r = .08 #radius from center to center of wheel

r1theta = -math.pi/3.0
r1x = .04 #math.cos(r1theta)*r
r1y = -.069282 #math.sin(r1theta)*r
r2theta = math.pi/3.0
r2x = r1x #math.cos(r2theta)*r
r2y = -r1y #math.sin(r2theta)*r
r3theta = math.pi
r3x = -r
r3y = 0

#print "R:"
#print r1x
#print r1y
#print r2x
#print r2y
#print r3x
#print r3y

s1theta = (math.pi)/6.0
s1x = .8660254
s1y = 0.5

s2theta = math.pi*(150/180)
s2x = -0.8660254
s2y = 0.5

s3theta = -math.pi/2.0
s3x = 0
s3y = -1
#print "S:"
#print s1x
#print s1y
#print s2x
#print s2y
#print s3x
#print s3y
mSub = matrix( [[0.8660254,0.5,(0.08)],
                [-0.8660254,0.5,(0.08)],
                [0,-1,(0.08)]] )

#print "mSub:"                
#print mSub

M = realWorldOffset*(1.0/s)*mSub
#print s
#print "M:"
#print M 

R = lambda theta: matrix( [[math.cos(theta),math.sin(theta),0.0],
             [math.sin(-theta),math.cos(theta),0.0],
             [0.0,0.0,1.0]] )

#print M

def getWheelVel(x,y,omega):
  desired = matrix( [[x],
                     [y],
                     [omega]] )
                   
  result = M*desired

  return result.getA()[0][0], result.getA()[1][0], result.getA()[2][0]
  
def getXYOmega(v1,v2,v3):
  velocity = matrix( [[v1],
                      [v2],
                      [v3]] )
  Minv = linalg.inv(M)
  
  result = Minv*velocity
  
  return result.getA()[0][0], result.getA()[1][0], result.getA()[2][0]

def getRobotXYOmega(x,y,omega,theta):
  desired = matrix( [[x],
                     [y],
                     [omega]] )
  desired = R(theta)*desired
  return desired
  
def getRobotXYOmegaAsTuple(x, y, omega, theta):
  desired = getRobotXYOmega(x, y, omega, theta)
  asArray = desired.getA()
  return asArray[0][0], asArray[1][0], asArray[2][0]

def getWheelVelTheta(x,y,omega,theta):
  desired = getRobotXYOmega(x, y, omega, theta)
                   
  result = M*desired

  return result.getA()[0][0], result.getA()[1][0], result.getA()[2][0]

def radianToQpps(radian):
  result = int(radian * 19820.0 / (2*math.pi))
  #print result 
  if result > 308420:
    return 308420
  elif result < -308420:
    return -308420
  else:
    return result