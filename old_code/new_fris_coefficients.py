
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

#param_L_0 is the lift paramter for alpha=0, constant value; reported in Hummel (2003)
#param_L_alpha is the lift parameter corresponding angle of attack; repported in Hummel (2003)
#alpha is the angle of attack, defined as the angle between the disc's velocity and the plane of the disc

def coef_L(alpha, param_L_0, param_L_alpha):
#Total lift coefficient, calculated by summing param_L_alpha and (constant) param_L_0
	return param_L_0 + param_L_alpha*alpha

'''
Drag
'''

#param_D_0 is the drag parameter at alpha=alpha_0 (minimum value of alpha). Constant value; reported in Hummel (2003)
#param_D_alpha is the drag parameter corresponding to angle of attack; reported in Hummel (2003)
#alpha is the angle of attack, as defined above

#alpha_0 is a constant value, reported by Hummel 2003 to be -4 degrees
alpha_0=-4

def coef_D_alpha(alpha, param_D_alpha, alpha_0):
	#Portion of drag coefficient dependent on angle of attack
	#Quadratic function of angle of attack (Hummel 2003)
	
	return param_D_alpha*(alpha-alpha_0)*(alpha-alpha_0)

def coef_D_total(alpha, param_D_alpha, param_D_0):
#Total drag coefficient, calculated by summing coef_D_alpha and (constant) PD_0	
	return param_D_0 + coef_D_alpha(alpha, param_D_alpha, alpha_0)

'''
Torque in y direction (pitch moment)
See page 12 eqn. 2.8b, Hummel 2003

'''

#param_tau_y0 is the y-body torque parameter (pitch) at alpha=0, Hummel (2003)
#param_tau_yalpha is the y-body torque parameter corresponding to alpha
#param_tau_ywy is the y-body torque parameter corresponding to y-direction angular velocity
#wy is angular velocity in y direction

def coef_tau_y_a(alpha, param_tau_yalpha):
	#Portion of torque coefficient in y direction dependent on of angle of attack
	return param_tau_yalpha*alpha

def coef_tau_y_wy(wy, param_tau_ywy):
	#Portion of torque coefficient in y direction dependent on y angular velocity
	#wy=angular velocity, rad/sec
	return param_tau_ywy*wy

def coef_tau_y_total(alpha,wy, param_tau_y0):
	#Total pitch moment
	return coef_tau_y_a(alpha)+coef_tau_y_wy(wy)+param_tau_y0

'''
Torque in x direction (roll moment)
See page 12 eqn. 2.8a, Hummel 2003

'''

#param_tau_xwx is a torque parameter for x direction, corresponds to x angular velocity
#param_tau_xwz=0.00171 is a torque parameter for x direciton, corresponds to z angular velocity

def coef_tau_x_total(wx,wz, param_tau_xwx, param_tau_xwz):
	return (param_tau_xwz*wz)+(param_tau_xwx*wx)

'''
Torque in z direction (yaw moment)
See page 12 eqn. 2.8c, Hummel 2003

'''

#param_tau_zwz is the torque parameter for z direction, corresponds to z angular velocity

def coef_tau_z_total(wz, param_tau_zwz):
	return param_tau_zwz*wz
	
