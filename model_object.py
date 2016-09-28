#This file creates a 'model object' which is initalized with 
#a Frisbee's parameter values. The file calculates the forces
#and torques acting on the Frisbee.

#Code adapted from Hummel 2003

import numpy as np
import math


#---------------------------------------------------------------------------------------------------#

#Create 'model class'. Initialized with Frisbee parameters. 

class Model(object):
	def __init__(self, PL0, PLa, PD0, PDa, PTya, PTywy, PTy0, PTxwx, PTxwz, PTzwz):
		self.PL0=PL0 #lift paramter for alpha=0, constant value (y-intercept in linear approximation)
		self.PLa=PLa #lift parameter corresponding angle of attack (slope in linear approximation)
		self.PD0=PD0 #"form drag,"" i.e. drag parameter at alpha=alpha_0, where alpha_0 is angle of attack
		#that produces zero lift and minimum drag
		self.PDa=PDa #"induced drag," i.e. drag parameter corresponding to angle of attack not equal to alpha_0 
		self.PTy0=PTy0 #y-body torque parameter (pitch) at alpha=0
		self.PTya=PTya #y-body torque parameter corresponding to alpha
		self.PTywy=PTywy #y-body torque parameter corresponding to y-direction angular velocity
		self.PTxwx=PTxwx #torque parameter for x direction, corresponds to x angular velocity
		self.PTxwz=PTxwz #torque parameter for x direciton, corresponds to z angular velocity
		self.PTzwz=PTzwz #torque parameter for z direction, corresponds to z angular velocity
		self.alpha_0=4.*math.pi/180. #radians (4 degrees), constant value reported in Hummel 2003

	#Print frisbee parameters 
	def __str__(self):
		return 'Frisbee Parameters: PL0, PLa, PD0, PDa, PTya, PTywy, PTy0, PTxwx, PTxwz, PTzwz'+'\n'+'(%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,)'%(self.PL0, self.PLa, self.PD0, self.PDa, self.PTya, self.PTywy, self.PTy0, self.PTxwx, self.PTxwz, self.PTzwz)

#---------------------------------------------------------------------------------------------------#
	
	#Calculate total lift coefficient 
	#Lift coefficient is a linear function of alpha
	def coef_lift(self, alpha):
		#print "\nIn coef_lift"
		#print self.PL0,self.PLa,alpha
		return self.PL0+self.PLa*alpha

#---------------------------------------------------------------------------------------------------#

	#Calculate total drag coefficient
	#Drag coefficient is a quadratic function of alpha
	#Note that drag force is never equal to zero
	def coef_drag(self,alpha):
		#print "\nIn coef_drag"
		#print self.PD0,self.PDa,alpha,self.alpha_0
		return self.PD0+self.PDa*((alpha-self.alpha_0)**2)

	'''
	For torque coefficient equations, see pg. 12 equn. 2.8b, Hummel 2003
	'''

#---------------------------------------------------------------------------------------------------#

	#Calculate total pitch moment (y-body torque coefficient)
	#wy stands for angular velocity in y-direction

	def coef_pitch(self, alpha, wy):
		#print "\nIn coef_pitch"
		#print "alpha,wy:",alpha,wy
		#print "PTy0,PTywy,PTya:",self.PTy0,self.PTywy,self.PTya
		return -self.PTy0+(self.PTywy*wy)+(self.PTya*alpha)

	#Calculate total roll moment (x-body torque coefficient)
	#wz is z-direction angular velocity, wx is x-direction angular velocity

	def coef_roll(self, wx, wz):
		#print "\nIn coef_roll"
		#print "wx,wz:",wx,wz
		#print "PTxwx,PTxwz:",self.PTxwx,self.PTxwz
		return -(self.PTxwz*wz)  - (self.PTxwx*wx)

	#Calculate total spin down moment (z-body torque coefficient)
	def coef_spin(self, wz):
		return -self.PTzwz*wz

#---------------------------------------------------------------------------------------------------#

