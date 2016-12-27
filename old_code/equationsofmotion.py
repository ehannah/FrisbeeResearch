'''
Equations of Motion: Numerical Integration
'''

import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#Constants
g=-9.81 #m/s^2
m=0.175 #kg
RHO=1.23 #density of air kg/m^3
AREA=0.0568 #m^2
Izz=0.00235 #kg-m^2
Ixx=Iyy=Ixy=0.00122 #kg-m^2
d=0.269 #m

#Coefficients/parameters that characterize the frisbee

#Lift and drag:
CLO=0.1
CLA=1.4
CDO=0.08
CDA=2.72
ALPHAO=-4

#Torques
CM0=-.08
CMA=0.43
CMq=(-0.5)*(10**(-3))
CRr=1.4*(10**(-2))
CRp=(-5.5)*(10**-3)
CNr=(-7.1)*(10**-6)
CMq=(-5.0)*(10**-3)

#Arbitrary alpha value and velocity
alpha=5 #degrees
velocity=14 #m/s

def torque_x(velocity, gamma_d, phi_d):
    Tx=(CRr*gamma_d+CRp*phi_d)*0.5*RHO*(velocity**2)*AREA*d
    return Tx

def phi_dd_rhs(Tx, theta, theta_d, phi_d, gamma_d):
    #phi_dd=(Tx+Izz*phi_d*math.degrees(math.cos(theta))*(phi_d*math.degrees(math.sin(theta))+gamma_d)-Iyy*phi_d*phi_d*math.degrees(math.sin(theta))*math.degrees(math.cos(theta)))/Ixy
    phi_dd = (Tx + Ixy*theta_d*phi_d*math.sin(theta) - Izz*theta_d*(phi_d*math.sin(theta)+gamma_d) + Ixy*theta_d*phi_d*math.sin(theta))*math.cos(theta)/Ixy
    return phi_dd

def torque_y(velocity, alpha, theta_d):
    Ty=(CM0+CMA*alpha+CMq*theta_d)*0.5*RHO*(velocity**2)*AREA*d
    return Ty

def theta_dd_rhs(Ty, theta, phi_d, gamma_d):
    #theta_dd=(Ty+Izz*phi_d*math.degrees(math.cos(theta))*(phi_d*math.degrees(math.sin(theta))+gamma_d)-Iyy*phi_d*phi_d*math.degrees(math.cos(theta))*math.degrees(math.sin(theta)))/Ixy
    theta_dd = (Ty + Izz*phi_d*math.cos(theta)*(phi_d*math.sin(theta)+gamma_d)-Iyy*(phi_d**2)*math.cos(theta)*math.sin(theta))/Ixy
    return theta_dd

def torque_z(gamma_d):
    Tz=(CNr*gamma_d)*0.5*RHO*(velocity**2)*AREA*d
    return Tz

def gamma_dd_rhs(Tz, theta, theta_d, phi_d, phi_dd):
    gamma_dd=(Tz-Izz*phi_dd*math.sin(theta)+theta_d*phi_d*math.cos(theta))/Izz
    return gamma_dd
#===================================================================================================
def main():
    '''
    Write a description of what happens when you run
    this file here.
    '''

    #Initial conditions (page 54)
    phi_coordinates=[]
    theta_coordinates=[]
    gamma_coordinates=[]

    phi=-.07 #rad
    theta=0.21 #rad
    gamma=5.03 #rad 

    phi_d=0.1#-14.94 #rad/sec
    theta_d=0.1#-1.48 #rad/sec
    gamma_d=54.25 #rad/sec

    phi_dd=0 #rad/sec^2
    theta_dd=0 #rad/sec^2
    gamma_dd=0 #rad/sec^2

    #Time
    time_coordinates=[]
    deltaT=0.001
    t=0

    i = 0

    while t<=3 and i < 20:
        i+=1
        #Torques
        Tx=torque_x(velocity, gamma_d, phi_d)
        Ty=torque_y(velocity, alpha, theta_d)
        Tz=torque_z(gamma_d)

        #Second Derivatives
        phi_dd=phi_dd_rhs(Tx, theta, theta_d, phi_d, gamma_d)
        theta_dd=theta_dd_rhs(Ty, theta, phi_d, gamma_d)
        gamma_dd=gamma_dd_rhs(Tz, theta, theta_d, phi_d, phi_dd)

        #Angular velocities
        phi_d=phi_d+phi_dd*deltaT
        theta_d=theta_d+theta_dd*deltaT
        gamma_d=gamma_d+gamma_d*deltaT

        print i, t, phi, phi_d, phi_dd

        #Angles
        phi=phi+phi_d*deltaT
        theta=theta+theta_d*deltaT
        gamma=gamma+gamma_d*deltaT

        #Trajectories
        phi_coordinates.append(phi)
        theta_coordinates.append(theta)
        gamma_coordinates.append(gamma)

        #Time
        t+=deltaT
        time_coordinates.append(t)


    #fig=plt.figure()
    #ax=fig.add_subplot(111, projection='3d')
    plt.plot(time_coordinates, phi_coordinates)
    plt.show()

if __name__ == '__main__':
    main()