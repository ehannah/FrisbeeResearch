import numpy as np
from scipy.integrate import odeint #odeint(func, y0, t[, args, Dfun, col_deriv, ...] Integrate a system of ordinary differential equations.
import math
import frisbee_object
import model_object

test_fris=frisbee_object.Frisbee(1,2,3,4,5,6,7,8,9,10,11,12)
test_fris.initialize_model(1,2,3,4,5,6,7,8,9,10)

def equations_of_motion(positions, t):

	x,y,z,vx,vy,vz,phi,theta,gamma,phidot,thetadot,gammadot=positions[0:12]

	positionsdot=test_fris.derivatives_array()
	return positionsdot

def main():

	#Define initial conditions 
	x=0 #m, lab displacement in x direction
	y=0 #m, lab displacement in y direction
	z=1 #m, lab displacement in z direction
	vx=10 #initial x component velocity
	vy=0 #initial y component velocity
	vz=5 #initial z component velocity 
	phi=16 #degrees, roll angle - corresponds to angle about x-axis
	theta=0 #degrees, pitch angle - corresponds to angle about y-axis
	gamma=0 #degrees, spin angle - corresponds to angle about z-xis
	phidot=0 #radians/s, roll angular momentum
	thetadot=0 #radians/s, pitch angular momentum
	gammadot=50 #radians/s, spin angular momentum

	positions=np.array([x,y,z,vx,vz,vz,phi,theta,gamma,phidot,thetadot,gammadot])

	#Define initial and final times
	ti=1.0
	tf=5.0

	#Define number of steps and calculate step size
	n=100.0 #number of steps
	dt=(tf-ti)/n

	#Create time array
	time=np.linspace(ti,tf, n)

	solution=odeint(equations_of_motion, positions, time)
	
	print(solution)

if __name__ == '__main__':
    main()