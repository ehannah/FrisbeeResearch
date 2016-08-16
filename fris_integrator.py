import numpy as np
from scipy.integrate import odeint #odeint(func, y0, t[, args, Dfun, col_deriv, ...] Integrate a system of ordinary differential equations.
import math
import frisbee_object
import model_object

def main():
	#Define initial conditions 
	x=1
	y=1
	z=1
	vx=1
	vy=1
	vz=1
	phi=1
	theta=1
	gamma=1
	phidot=1
	thetadot=1
	gammadot=1

	initial_conditions=np.array([x,y,z,vx,vy,vz,phidot,thetadot,gammadot,phi,theta,gamma])

	#Define initial and final times
	ti=0.0 #s, initial time
	tf=5.0 #s, final time

	#Define number of steps and calculate step size
	n=1000 #number of steps
	dt=(tf-ti)/1000

	#Create time array to feed to integrator
	time=np.array([ti,tf,dt])
