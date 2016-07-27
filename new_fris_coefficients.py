
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
#Total lift coefficient, calculated by summing param_L_alpha and (constant) param_L_0
	return param_L_0 + param_L_alpha*alpha

'''
Drag
'''

param_D_0 = 0.1769 #Drag parameter at alpha=alpha_0 (minimum value of alpha). Constant value; reported in Hummel (2003)
param_D_alpha = 0.685 #Drag parameter corresponding to angle of attack; reported in Hummel (2003)

def coef_D_alpha(alpha):
	#Portion of drag coefficient dependent on angle of attack
	#Quadratic function of angle of attack (Hummel 2003)
	#alpha_0 is a constant value, reported by Hummel 2003 to be -4 degrees

	return param_D_alpha*(alpha-alpha_0)*(alpha-alpha_0)

def coef_D_total(alpha):
#Total drag coefficient, calculated by summing coef_D_alpha and (constant) PD_0	
	return param_D_0 + coef_D_alpha(alpha)

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

'''
Torque in x direction (roll moment)
See page 12 eqn. 2.8a, Hummel 2003

'''

param_tau_xwx=-0.0125 #Torque parameter for x direction, corresponds to x angular velocity
param_tau_xwz=0.00171#Torque parameter for x direciton, corresponds to z angular velocity

def coef_tau_x_total(wx,wz):
	return (param_tau_xwz*wz)+(param_tau_xwx*wx)

'''
Torque in z direction (yaw moment)
See page 12 eqn. 2.8c, Hummel 2003

'''

param_tau_zwz=-0.0000341 #Torque parameter for z direction, corresponds to z angular velocity

def coef_tau_z_total(wz):
	return param_tau_zwz*wz
	