
import numpy as np
from scipy.integrate import odeint #Integrates a system of ordinary differential equations given initial conditions.
import math, sys
import frisbee_object 
import model_object
import fris_integrator
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#Define initial conditions
x=0.
y=0.
z=1.
vx=20.
vy=0.
vz=0.
phi=0.
theta=-5.
gamma=0
phidot=0
thetadot=0
gammadot=-50

#State model parameters
PL0=0.3331
PLa=1.9124
PD0=0.1769
PDa=0.685 
PTya=0.4338
PTywy=-0.0144
PTy0=-0.0821
PTxwx=-0.0125
PTxwz=-0.00171
PTzwz=-0.0000341


#Declare final and initial times, and step size
t0=0.0
tf=3.0
n = 1000
dt=(tf-t0)/(n-1)
time=np.arange(t0,tf,dt)

frisbee=frisbee_object.Frisbee(x,y,z,vx,vy,vz,phi*np.pi/180.,theta*np.pi/180.,gamma*np.pi/180., phidot,thetadot,gammadot)
model=model_object.Model(PL0, PLa, PD0, PDa, PTya, PTywy, PTy0, PTxwx, PTxwz, PTzwz)
frisbee.model=model

initial_conditions=np.array(frisbee.get_positions()).copy()

def get_trajectory(frisbee):

	positions=frisbee.x,frisbee.y,frisbee.z,frisbee.vx,frisbee.vy,frisbee.vz, frisbee.phi,frisbee.theta,frisbee.gamma, frisbee.phidot,frisbee.thetadot,frisbee.gammadot
	trajectory=odeint(fris_integrator.equations_of_motion, initial_conditions, time)
    
	return time,trajectory

def make_plots(trajectory):
    fig=plt.figure()
    ax=fig.add_subplot(111, projection='3d')
    #ax.set_aspect('equal','box')
    ax.set_xlim(0,30)
    ax.set_ylim(-15,15)
    ax.set_zlim(0,30)
    plt.plot(trajectory[:,0], trajectory[:,1], trajectory[:,2])
    plt.draw()
    plt.pause(1)
    raw_input('Press enter to close.')
    plt.close(fig)


def main():

    solution=get_trajectory(frisbee)
    time,trajectory=solution[0],solution[1]
    print(time)
    print(solution)
    make_plots(trajectory)
    np.savetxt("solution.txt", solution[1])
    solution = np.loadtxt("solution.txt")

if __name__ == '__main__':
    main()
