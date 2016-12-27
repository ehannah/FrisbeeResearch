'''
Calculate each torque.

1. Calculate angular velocities in frisbee frame.
(Hummel Pg. 34: w=phidot*cos(theta)*xbhat+thetadot*ybhat+(phidot*cos(theta)+gammadot)*zbhat
2. Calculate angular velocities in the lab 

import numpy as np
import math

#Frisbee frame angular velocity

def ang_velocitybod(theta, phidot, thetadot, gammadot):
#Equation for inertial angular velocity can be found on pg. 34 of Hummel 2003:
#w=phidot*cos(theta)*xhat+thetadot*yhat+(phidot*sin(theta)*gammadot)*zhat
#Velocities are calculated here in frisbee frame

  return np.array([phidot*math.cos(theta), thetadot, phidot*math.sin(theta)+gammadot)])

'''
#Lab frame angular velocity

def ang_velocitylab
'''
