#The following code is adapted from Hummel 2003 and unpublished code written by Tom McClintock
#Code simulates Discraft Ultrastar trajectory.

#Main code will:
#1) Define constants.
#2) Set initial conditions.
#3) Create Frisbee object using specified initial conditions.
#4) Call functions that calculate forces and torques for Frisbee object.
#5) Rotate forces and torques from lab frame into frisbee frame.
#6) Calculate all derivatives and numerically integrate.

#---------------------------------------------------------------------------#
# Import relevant modules:

import math
import frisbee_object as frisbee #Creates frisbee object and assigns it relevant 
#physical characteristics.
import new_fris_coefficients as coef #Contains functions for coefficients that 
#characterize frisbee.
import forces_and_torques as ft #Contains functions that calculate forces and 
#torques acting on Frisbee

#---------------------------------------------------------------------------#

#Define constant values:

g=-9.81 #m/s^2, force of gravity
m=0.175 #kg, mass of standard Discraft Ultrastar (Hummel 2003)
RHO=1.23 #kg/m^3, density of air
AREA=0.0568 #m^2, surface area of Discraft Ultrastar (Hummel 2003)
Izz=0.00235 #kg-m^2, moment of inertia about the z-axis
Ixx=Iyy=Ixy=0.00122 #kg-m^2, moment of inertia about the x and y axes
d=0.269 #m, diameter of Discraft Ultrastar (Hummel 2003)

#---------------------------------------------------------------------------#

#Set initial conditions:
#Notes:
#-->Psi is defined as the angle about the x-axis. Theta is defined as the angle about
#the y-axis. Phi is defined as the angle about the z-axis.
#-->Wind is currently not factored into code.

#Initial position coordinates.
x=1
y=1
z=0

#Initial velocity coordinates
vx=1
vy=1
vz=1

#Initial angles.
psi=1
theta=1
phi=1

#Initial angular velocities.
wx=1 #Corresponds to psi.
wy=1 #Corresponds to theta.
wz=1 #Corresponds to phi.

#---------------------------------------------------------------------------#

#Create a frisbee object with above initial conditions using frisbee_object code.
test_fris=frisbee.Frisbee(x,y,z,vx,vy,vz,wx,wy,wz,psi,theta,phi)

#Calculate angle of attack.
alpha=test_fris.attackangle()

#Call functions that calculate forces and torques, then multiply them by appropriate
#unit vectors.
