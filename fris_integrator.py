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
init_positions = [0.,0.,1.,20.,0.,0.,0.,-.087,0.,0.,0.,-50.]
test_fris=frisbee_object.Frisbee(0.,0.,1.,20.,0.,0.,0.,-.087,0.,0.,0.,-50.,debug=False)
test_fris.initialize_model(0.331,1.9124,0.1769,0.685,  0.4338,0.0144,0.0821,  0.0125,0.00171,0.0000341)

#print("Initial derivatives: ",test_fris.derivatives_array())
#sys.exit()
#---------------------------------------------------------------------------------------------------#
#Define function to feed into ODE integrator. The function will compute all relevant derivatives
#at time t, defined in main. All derivatives are explained and calculated in frisbee_object.py
#module, and an array of derivatives is called below.

def equations_of_motion(positions, t):

    #Update the positions of the frisbee
    (test_fris.x,test_fris.y,test_fris.z,
     test_fris.vx,test_fris.vy,test_fris.vz,
     test_fris.phi,test_fris.theta,test_fris.gamma,
     test_fris.phidot,test_fris.thetadot,test_fris.gammadot)=positions.copy()
    
    if test_fris.z <= 0.0:
        return np.zeros_like(positions)

    #Calculate all derivatives based on current positions. Return array of derivatives.
    positionsdot=test_fris.derivatives_array()
    #print "\nEOM @ t = %f:"%t
    #for i in range(len(positionsdot)):
    #    print "\t%f"%positionsdot[i]
    #if t > 0.001:
    #    sys.exit()
    return positionsdot

#---------------------------------------------------------------------------------------------------#
def main():

    #Integration of ODEs that reflect equations of motion.
    #Common release conditions obtained from Hummel 2003 (pg. 83)

    #The starting positions are a copy of the test frisbee's
    #positions at the start of this main() function.
    initial_positions=np.array(test_fris.get_positions()).copy()

    #Define initial and final times
    ti=0.0
    tf=.03

    #Define number of steps and calculate step size
    n=4 #number of steps
    dt=(tf-ti)/(n-1)

    #Create time array
    time=np.linspace(ti,tf, n)

    """
    THIS IS A SIMPLE RK4 ODE SOLVER WRITTEN BY TOME FOR DEBUGGING
    """
    solution = np.zeros((n,12))
    positions = np.array(initial_positions).copy()
    for i in range(n):
        print test_fris
        solution[i] = positions.copy()
        k1 = np.array(equations_of_motion(positions,time[i]))
        temp = positions + k1*dt/2.
        k2 = np.array(equations_of_motion(temp,time[i]+dt/2.))
        temp = positions + k2*dt/2.
        k3 = np.array(equations_of_motion(temp,time[i]+dt/2.))
        temp = positions + k3*dt
        k4 = np.array(equations_of_motion(temp,time[i]+dt))
        positions = positions + dt/6.*(k1+2*k2+2*k3+k4)

        #print [positions[:]]
        #print "positions:",test_fris.get_positions()
    sys.exit()

    #print equations_of_motion(initial_positions,time[0])

    #solution=odeint(equations_of_motion, initial_positions, time)
    print solution.shape

    np.savetxt("solution.txt",solution)
    solution = np.loadtxt("solution.txt")
#---------------------------------------------------------------------------------------------------#

    #Graph solutions to analyze relevance and assess code.
    
    
    for i in xrange(0,12):
        derivativenames=(['x-Position (m)','y-Position (m)','z-Position (m)','vx (x-velocity (m/s))','vy (y-velocity (m/s))','vz (z-velocity (m/s))',
            'Phi','Theta','Gamma','phidot (phi angular velocity (radians/s))','thetadot (theta angular velocity (radians/s))',
            'gammadot (gamma angular velocity (radians/s))'])
        for j in range(0,len(time)):
            print time[j],solution[j,i]
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
    #ax.set_aspect('equal','box')
    ax.set_xlim(0,10)
    ax.set_ylim(0,10)
    ax.set_zlim(0,10)
    plt.plot(solution[:,0], solution[:,1], solution[:,2])
    plt.show()
    raw_input('Press enter to close.')
    plt.close(fig)

if __name__ == '__main__':
    main()
