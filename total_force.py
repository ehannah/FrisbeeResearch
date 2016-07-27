'''
Calculate total forces acting on disc:

1. Multiply lift and drag forces by appopriate unit vectors to give them
correct directionality
2. Subtract force of gravity
3. Sum forces
'''

import unitvectors as unitv
import forces_and_torques as ft
import numpy as np

def F_gravity(m):
	return -m*g*np.array([0,0,1])

#Drag vector points opposite velocity of disc
#Lift vector points orthogonal to drag vector (cross product of vhat,ybhat)

def total_force():
	return ft.F_drag(vx,vy,vz,alpha)*-unitv.vhat() 
	+ft.F_lift(vx,vy,vz,alpha)*(np.cross(unitv.vhat(),unitv.ybhat))
	+F_gravity
