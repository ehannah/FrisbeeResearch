#Import relevant modules.

import numpy as np
from scipy.integrate import odeint #Integrates a system of ordinary differential equations given initial conditions.
import math
import frisbee_object
import model_object
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#---------------------------------------------------------------------------------------------------#
#Initialize frisbee object with appropriate coefficient values and initial conditions.
#Current parameter input values obtained from Hummel 2003 (pg. 82)
test_fris=frisbee_object.Frisbee(1,2,3,4,5,6,7,8,9,10,11,12)
test_fris.initialize_model(0.331,1.9124,0.1769,0.685,-0.0821,0.4338,-.005,-.0055,0.00171,0.0000071)

#---------------------------------------------------------------------------------------------------#
#Define function to feed into ODE integrator. The function will compute all relevant derivatives
#at time t, defined in main. All derivatives are explained and calculated in frisbee_object.py
#module, and an array of derivatives is called below.

def equations_of_motion(positions, t):

	x,y,z,vx,vy,vz,phi,theta,gamma,phidot,thetadot,gammadot=positions[0:12]

	positionsdot=test_fris.derivatives_array()
	return positionsdot

#---------------------------------------------------------------------------------------------------#
def main():

	#Integration of ODEs that reflect equations of motion.

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

#---------------------------------------------------------------------------------------------------#

	#Graph solutions to analyze relevance and assess code.
	for i in range(12):
		derivativenames=(['x-Position (m)','y-Position (m)','z-Position (m)','vx (x-velocity (m/s))','vy (y-velocity (m/s))','vz (z-velocity (m/s))',
			'Phi','Theta','Gamma','phidot (phi angular velocity (radians/s))','thetadot (theta angular velocity (radians/s))',
			'gammadot (gamma angular velocity (radians/s))'])
		fig=plt.figure()
		plt.plot(time, solution[:,i])
		plt.ylabel(derivativenames[i])
		plt.xlabel('Time (Seconds)')
		plt.draw()
		plt.pause(1)
		raw_input('Press enter to close.')
		plt.close(fig)

if __name__ == '__main__':
    main()