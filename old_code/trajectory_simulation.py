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
import numpy as np
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
rho=1.225 #kg/m^3, density of air
area=0.057 #m^2, surface area of Discraft Ultrastar (Hummel 2003)
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

#Calculate forces and torquest that act on Frisbee using appropriate functions,
#then rotate from lab frame to frisbee frame by unit vector multiplication.

#Lift force
Lift_total=(test_fris.F_lift())*(np.cross(test_fris.vhat(),test_fris.ybhat()))

#Drag force
Drag_total=(test_fris.F_drag())*-test_fris.vhat()

#Force of gravity
F_g=-m*g*np.array([0,0,1])

Total_force=Lift_total+Drag_total+F_g
'''
#Calculate forces and torques that act on the Frisbee by calling functions that contain
#the appropriate coefficients. Then rotate the forces from the lab frame into the frisbee
#frame by multiplying by correct unit vectors.

#Lift force
F_lift_lab=coef.coef_L(alpha, param_L_0, param_L_alpha)*0.5*rho*area*test_fris.velocity_dot()
F_lift_fris=F_lift_lab*(np.cross(test_fris.vhat(),test_fris.ybhat()))

#Drag force
F_drag_lab=coef.coef_D_total(alpha, param_D_alpha, alpha_0)*0.5*rho*area*test_fris.velocity_dot()
F_drag_fris=F_drag_lab*-test_fris.vhat()

#Force of gravity
F_g=-m*g*np.array([0,0,1])

#Total force
F_total=F_lift_fris+F_drag_fris+F_g
'''



