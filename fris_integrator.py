#Import relevant modules.

import numpy as np
from scipy.integrate import odeint #Integrates a system of ordinary differential equations given initial conditions.
import math
import frisbee_object as frisbee_object
import model_object
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys

#---------------------------------------------------------------------------------------------------#
#Initialize frisbee object with appropriate coefficient values and initial conditions.
#Current parameter input values obtained from Hummel 2003 (pg. 82)
#Change to debug=False to supress printing
test_fris=frisbee_object.Frisbee(0.,0.,1.,20.,0.,0.,0.,-.087,0.,0.,0.,50.,debug=False)
test_fris.initialize_model(0.331,1.9124,0.1769,0.685,  0.4338,0.0144,0.0821,  0.0125,0.00171,0.0000341)
#print(test_fris.get_force())
#print(test_fris.get_torque())
print("Initial derivatives: ",test_fris.derivatives_array())
#sys.exit()
#---------------------------------------------------------------------------------------------------#
#Define function to feed into ODE integrator. The function will compute all relevant derivatives
#at time t, defined in main. All derivatives are explained and calculated in frisbee_object.py
#module, and an array of derivatives is called below.

def equations_of_motion(positions, t):

    #Current positions
    (test_fris.x,test_fris.y,test_fris.z,
        #Current velocities
        test_fris.vx,test_fris.vy,test_fris.vz,
        #Current angular positions
        test_fris.phi,test_fris.theta,test_fris.gamma,
        #Current angular velocities
        test_fris.phidot,test_fris.thetadot,test_fris.gammadot)=positions[0:12]

    #Calculate all derivatives based on current positions. Return array of derivatives.
    positionsdot=test_fris.derivatives_array()
    #print "EOM:",t,positionsdot[9:12]
    #if t > 0.001:
    #    sys.exit()
    return positionsdot

#---------------------------------------------------------------------------------------------------#
def main():

    #Integration of ODEs that reflect equations of motion.
    #Common release conditions obtained from Hummel 2003 (pg. 83)

    #List of initial conditions to feed to equations_of_motion function.
    #Ordered in same manner as 'positions' array above
    positions=np.array([test_fris.x,test_fris.y,test_fris.z,
        test_fris.vx,test_fris.vy,test_fris.vz,
        test_fris.phi,test_fris.theta,test_fris.gamma,
        test_fris.phidot,test_fris.thetadot,test_fris.gammadot])

    #Define initial and final times
    ti=0.0
    tf=3.5

    #Define number of steps and calculate step size
    n=50 #number of steps
    dt=(tf-ti)/(n-1)

    #Create time array
    time=np.linspace(ti,tf, n)

    print equations_of_motion(positions,time[0])
    solution=odeint(equations_of_motion, positions, time)

    print time[:10]
    print(solution.shape)
    np.savetxt("solution.txt",solution)
    solution = np.loadtxt("solution.txt")
#---------------------------------------------------------------------------------------------------#

    #Graph solutions to analyze relevance and assess code.
    
    
    for i in xrange(0,1):
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
    
    #sys.exit()
    fig=plt.figure()
    ax=fig.add_subplot(111, projection='3d')
    plt.plot(solution[:,0], solution[:,1], solution[:,2])
    plt.show()
    raw_input('Press enter to close.')

if __name__ == '__main__':
    main()
