
import numpy as np
import math
import matplotlib.pyplot as plt

'''

parameters - refer to the things we are looking for, e.g. CL0, CD0, CNz
   THESE SHOULD ALSO BE PASSED TO THE EQUATIONS OF MOTION!!!

coefficient functions - refer to the functions(!) that return the force/torque 
   coefficients, i.e. CL(v,alpha)

'''

'''
Lift 
'''
param_L_0=0.3331 #Lift paramter for alpha=0, constant value; reported in Hummel (2003)
param_L_alpha=1.9123 #Lift parameter corresponding angle of attack; repported in Hummel (2003)

def coef_L(alpha):
#Total lift coefficient, calculated by summing CL_alpha and (constant) CL_0
	return param_L_0 + param_L_alpha*alpha

alpha=-50
alpha_coordinates=[]
CL_coordinates=[]
while alpha<100:
	CL_coordinates.append(coef_L(alpha))
	alpha_coordinates.append(alpha)
	alpha+=0.1

plt.plot(alpha_coordinates,CL_coordinates)
plt.show()

'''
Drag
'''

param_D_0 = 0.1769 #Drag parameter at alpha=alpha_0 (minimum value of alpha). Constant value; reported in Hummel (2003)
param_D_alpha = 0.685 #Drag parameter corresponding to angle of attack; reported in Hummel (2003)

def coef_D_alpha(alpha):
	#Portion of drag coefficient dependent on angle of attack
	#Quadratic function of angle of attack (Hummel 2003)

	return param_D_alpha*(alpha-alpha_0)*(alpha-alpha_0)

def coef_D_total(alpha):
#Total drag coefficient, calculated by summing coef_D_alpha and (constant) PD_0	
	return param_D_0 + coef_D_alpha(alpha)

alpha_0=15
alpha=-50

alpha_coordinates=[]
CD_coordinates=[]
while alpha<100:
	CD_coordinates.append(coef_D_total(alpha))
	alpha_coordinates.append(alpha)
	alpha+=0.1

plt.plot(alpha_coordinates,CD_coordinates)
plt.show()

'''
Torque in y direction (pitch moment)
See page 12 eqn. 2.8b, Hummel 2003

'''


param_tau_y0=-0.0821 #Y-body torque parameter (pitch) at alpha=0, Hummel (2003)
param_tau_yalpha=0.4338#Y-body torque parameter corresponding to alpha
param_tau_ywy=-0.0144#Y-body torque parameter corresponding to y-direction angular velocity
wy=10#Angular velocity in y direction

def coef_tau_y_a(alpha):
	#Portion of torque coefficient in y direction dependent on of angle of attack
	return param_tau_yalpha*alpha

def coef_tau_y_wy(wy):
	#Portion of torque coefficient in y direction dependent on y angular velocity
	#wy=angular velocity, rad/sec
	return param_tau_ywy*wy

def coef_tau_y_total(alpha,wy):
	#Total pitch moment
	return coef_tau_y_a(alpha)+coef_tau_y_wy(wy)+param_tau_y0

wy=35
alpha=-50
wy_coordinates=[]
tau_y_coordinates=[]
while wy<100:
	tau_y_coordinates.append(coef_tau_y_total(alpha,wy))
	wy_coordinates.append(wy)
	wy+=0.1

plt.plot(wy_coordinates,tau_y_coordinates)
plt.show()

'''
Torque in x direction (roll moment)
See page 12 eqn. 2.8a, Hummel 2003

'''

param_tau_xwx=-0.0125 #Torque parameter for x direction, corresponds to x angular velocity
param_tau_xwz=0.00171#Torque parameter for x direciton, corresponds to z angular velocity

def coef_tau_x_total(wx,wz):
	return (param_tau_xwz*wz)+(param_tau_xwx*wx)

wx=35
wz=15
alpha=-50
alpha_coordinates=[]
tau_x_coordinates=[]
while alpha<100:
	tau_x_coordinates.append(coef_tau_x_total(wx, wz))
	alpha_coordinates.append(alpha)
	alpha+=0.1

plt.plot(alpha_coordinates,tau_x_coordinates)
plt.show()

wx=-50
wz=25
wx_coordinates=[]
tau_x_coordinates=[]
while wx<100:
	tau_x_coordinates.append(coef_tau_x_total(wx,wz))
	wx_coordinates.append(wx)
	wx+=0.1

plt.plot(wx_coordinates,tau_x_coordinates)
plt.show()

wx=25
wz=-50
wz_coordinates=[]
tau_x_coordinates=[]
while wz<100:
	tau_x_coordinates.append(coef_tau_x_total(wx,wz))
	wz_coordinates.append(wz)
	wz+=0.1

plt.plot(wz_coordinates,tau_x_coordinates)
plt.show()
'''
Torque in z direction (yaw moment)
See page 12 eqn. 2.8c, Hummel 2003

'''

param_tau_zwz=-0.0000341 #Torque parameter for z direction, corresponds to z angular velocity

def coef_tau_z_total(wz):
	return param_tau_zwz*wz

wz=15
alpha=-50
alpha_coordinates=[]
tau_x_coordinates=[]
while alpha<100:
	tau_x_coordinates.append(coef_tau_z_total(wz))
	alpha_coordinates.append(alpha)
	alpha+=0.1

plt.plot(alpha_coordinates,tau_x_coordinates)
plt.show()

wz=-50
wz_coordinates=[]
tau_z_coordinates=[]
while wz<100:
	tau_z_coordinates.append(coef_tau_z_total(wz))
	wz_coordinates.append(wz)
	wz+=0.1

plt.plot(wz_coordinates,tau_z_coordinates)
plt.show()


	